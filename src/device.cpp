#include <algorithm>
#include <atomic>
#include <chrono>
#include <librfnm/device.h>
#include <librfnm/rx_stream.h>
#include <spdlog/spdlog.h>
#include <libusb-1.0/libusb.h>

#ifndef _WIN32
#include <netinet/in.h>
#include <netinet/tcp.h>
#endif

#ifdef BUILD_RFNM_LOCAL_TRANSPORT
#include <poll.h>
#endif

using namespace rfnm;

struct rfnm::_usb_handle {
    libusb_device_handle* primary{};
    libusb_device_handle* boost{};
};

MSDLL device::device(enum transport transport, std::string address, enum debug_level dbg) {
    rx_s.qbuf_cnt = 0;

    //if (transport != TRANSPORT_USB) {
    //    spdlog::error("Transport not supported");
    //    throw std::runtime_error("Transport not supported");
    //}



    s = new struct status();
    usb_handle = new _usb_handle;

    int cnt = 0;
    int dev_cnt = 0;
    int r;
    std::vector<struct rfnm_dev_hwinfo> found;
    libusb_device** devs = NULL;

    // set default/native stream format
    s->transport_status.rx_stream_format = STREAM_FORMAT_CS16;
    s->transport_status.tx_stream_format = STREAM_FORMAT_CS16;
    if (transport == TRANSPORT_USB || transport == TRANSPORT_FIND) {
#if LIBUSB_API_VERSION >= 0x0100010A
        r = libusb_init_context(nullptr, nullptr, 0);
#else
        r = libusb_init(nullptr);
#endif
        if (r < 0) {
            spdlog::error("RFNMDevice::activateStream() -> failed to initialize libusb");
            goto error_usb;
        }

        dev_cnt = libusb_get_device_list(NULL, &devs);
        if (dev_cnt < 0) {
            spdlog::error("failed to get list of usb devices");
            goto error_usb;
        }

        for (int d = 0; d < dev_cnt; d++) {
            struct libusb_device_descriptor desc;
            int r = libusb_get_device_descriptor(devs[d], &desc);
            if (r < 0) {
                spdlog::error("failed to get usb dev descr");
                goto next;
            }

            if (desc.idVendor != RFNM_USB_VID || desc.idProduct != RFNM_USB_PID) {
                goto next;
            }

            r = libusb_open(devs[d], &usb_handle->primary);
            if (r) {
                spdlog::error("Found RFNM device, but couldn't open it {}", r);
                goto next;
            }

            if (address.length()) {
                uint8_t sn[9];
                if (libusb_get_string_descriptor_ascii(usb_handle->primary, desc.iSerialNumber, sn, 9) >= 0) {
                    sn[8] = '\0';
                    if (strcmp((const char*)sn, address.c_str())) {
                        spdlog::info("This serial {} doesn't match the requested {}", (const char*)sn, address);
                        goto next;
                    }
                }
                else {
                    spdlog::error("Couldn't read serial descr");
                    goto next;
                }
            }

            if (libusb_get_device_speed(libusb_get_device(usb_handle->primary)) < LIBUSB_SPEED_SUPER) {
                spdlog::error("You are connected using USB 2.0 (480 Mbps), however USB 3.0 (5000 Mbps) is required. "
                    "Please make sure that the cable and port you are using can work with USB 3.0 SuperSpeed");
                goto next;
            }

            r = libusb_claim_interface(usb_handle->primary, 0);
            if (r < 0) {
                spdlog::error("Found RFNM device, but couldn't claim the interface, {}, {}", r, libusb_strerror((libusb_error)r));
                goto next;
            }

            s->transport_status.theoretical_mbps = 3500;

            // The boost interface must belong to THE SAME BOARD as the primary: opening
            // by VID/PID alone grabs whichever board enumerates first, and on a multi-
            // board host that (a) reads another board's samples (observed: framing
            // garbage) and (b) arms the victim board's boost endpoints, whose queued
            // requests then eat its shared IN-inflight budget until its RX shipping
            // gates shut permanently (the duplex wedge's terminal state). Match serials.
            usb_handle->boost = nullptr;
            {
                libusb_device** blist;
                ssize_t bcnt = libusb_get_device_list(nullptr, &blist);
                unsigned char prim_serial[64] = {0}, cand_serial[64] = {0};
                struct libusb_device_descriptor pdesc;
                libusb_get_device_descriptor(libusb_get_device(usb_handle->primary), &pdesc);
                libusb_get_string_descriptor_ascii(usb_handle->primary, pdesc.iSerialNumber, prim_serial, sizeof(prim_serial) - 1);
                for (ssize_t bi = 0; bi < bcnt; bi++) {
                    struct libusb_device_descriptor bdesc;
                    if (libusb_get_device_descriptor(blist[bi], &bdesc)) {
                        continue;
                    }
                    if (bdesc.idVendor != RFNM_USB_VID || bdesc.idProduct != RFNM_USB_PID_BOOST) {
                        continue;
                    }
                    libusb_device_handle* bh = nullptr;
                    if (libusb_open(blist[bi], &bh) || !bh) {
                        continue;
                    }
                    cand_serial[0] = 0;
                    libusb_get_string_descriptor_ascii(bh, bdesc.iSerialNumber, cand_serial, sizeof(cand_serial) - 1);
                    if (strcmp((char*)prim_serial, (char*)cand_serial)) {
                        libusb_close(bh);
                        continue;
                    }
                    usb_handle->boost = bh;
                    break;
                }
                libusb_free_device_list(blist, 1);
            }
            // STOPGAP: do not claim the boost interface at all. Claiming arms the gadget's
            // boost endpoints, whose queued requests share the IN-inflight budget - and the
            // async RX engine does not pump boost endpoints yet, so armed-but-unpumped boost
            // requests pin the budget at its cap and gate RX shipping shut (the duplex
            // wedge's terminal state). Re-enable when the engine pumps 8 pipelines.
            if (usb_handle->boost) {
                libusb_close(usb_handle->boost);
                usb_handle->boost = nullptr;
            }

            spdlog::info("Max theoretical transport speed is {} Mbps", s->transport_status.theoretical_mbps);

            r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR),
                RFNM_B_REQUEST, RFNM_GET_SM_RESET, 0, NULL, 0, 500);
            if (r < 0) {
                spdlog::error("Couldn't reset state machine");
                goto next;
            }
            reset_device_state();

            s->transport_status.transport = TRANSPORT_USB;
            THREAD_COUNT = MAX_THREAD_COUNT;

            if (get(REQ_ALL)) {
                goto next;
            }

            libusb_free_device_list(devs, 1);

            for (int8_t i = 0; i < THREAD_COUNT; i++) {
                thread_data[i].ep_id = i + 1;
                thread_data[i].rx_active = 0;
                thread_data[i].tx_active = 0;
                thread_data[i].shutdown_req = 0;
            }

            for (int i = 0; i < THREAD_COUNT; i++) {
                thread_c[i] = std::thread(&device::threadfn, this, i);
            }

            // Success
            return;

        next:
            if (usb_handle->primary) {
                libusb_release_interface(usb_handle->primary, 0);
                libusb_close(usb_handle->primary);
                usb_handle->primary = NULL;
            }
            if (usb_handle->boost) {
                libusb_release_interface(usb_handle->boost, 0);
                libusb_close(usb_handle->boost);
                usb_handle->boost = NULL;
            }
        }

    error_usb:
        libusb_free_device_list(devs, 1);
        libusb_exit(NULL);
        delete usb_handle;

        if (transport != TRANSPORT_FIND) {
            delete s;
            throw std::runtime_error("RFNM culdn't find any USB devices");
        }

    }
    if (transport == TRANSPORT_LOCAL || transport == TRANSPORT_FIND) {
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
        int fd = open("/dev/rfnm_ctrl_ep", O_RDWR);
        if (fd < 0) {
            goto exit_local;
        }
        uint8_t ep_ctrl_buf[RFNM_SYSCTL_TRANSFER_SIZE];

        if (ioctl(fd, RFNM_IOCTL_BASE + (0xff & RFNM_GET_DEV_HWINFO), &ep_ctrl_buf) < 0) {
            close(fd);
            goto exit_close_local;
        }

        rfnm_ctrl_ep_ioctl = fd;

        struct rfnm_dev_hwinfo r_hwinfo;

        memcpy(&r_hwinfo, &ep_ctrl_buf[0], sizeof(struct rfnm_dev_hwinfo));

        if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
            uint32_t pv = r_hwinfo.protocol_version;
            spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED detected protocol is v{} while your librfnm is for v{}", pv, RFNM_PROTOCOL_VERSION);
            goto exit_close_local;
        }

        s->transport_status.theoretical_mbps = 3500;

        //spdlog::info("Max theoretical transport speed is {} Mbps", s->transport_status.theoretical_mbps);

        if (ioctl(fd, RFNM_IOCTL_BASE + (0xff & RFNM_GET_SM_RESET), &ep_ctrl_buf) < 0) {
            //close(fd);
            spdlog::error("Couldn't reset state machine");
            goto exit_close_local;
        }
        reset_device_state();

        s->transport_status.transport = TRANSPORT_LOCAL;
        THREAD_COUNT = 2;

        if (get(REQ_ALL)) {
            goto exit_close_local;
        }

        for (int8_t i = 0; i < THREAD_COUNT; i++) {
            thread_data[i].ep_id = i + 1;
            thread_data[i].rx_active = 0;
            thread_data[i].tx_active = 0;
            thread_data[i].shutdown_req = 0;
        }

        for (int i = 0; i < THREAD_COUNT; i++) {
            thread_c[i] = std::thread(&device::threadfn, this, i);
        }


        // Success
        return;

    exit_close_local:
        close(fd);
#endif
    }
exit_local:
    if (transport == TRANSPORT_TCP || transport == TRANSPORT_FIND) {
        try {
            // Reuse discovery to resolve the device address: it probes every host interface
            // and skips malformed or protocol-mismatched replies instead of failing on the
            // first bad one
            std::vector<struct dev_info> remote = find(TRANSPORT_TCP, address);
            if (remote.empty()) {
                goto exit_eth;
            }
            rfnm_eth_transport_ip_addr = remote[0].address;

            // Connect to the TCP control port
            rfnm_ctrl_ioctx_tcp = std::make_unique<asio::io_context>();
            rfnm_ctrl_socket_tcp = std::make_unique<asio::ip::tcp::socket>(*rfnm_ctrl_ioctx_tcp);
            asio::ip::tcp::endpoint tcp_ctrl_ep(asio::ip::make_address(rfnm_eth_transport_ip_addr), RFNM_TCP_CTRL_PORT);
            rfnm_ctrl_socket_tcp->connect(tcp_ctrl_ep);
            rfnm_ctrl_socket_tcp->set_option(asio::ip::tcp::no_delay(true));
            spdlog::info("Connected TCP control to {}:{}", rfnm_eth_transport_ip_addr, RFNM_TCP_CTRL_PORT);

            s->transport_status.theoretical_mbps = 1000;
            s->transport_status.transport = TRANSPORT_TCP;
            THREAD_COUNT = 2;

            uint8_t dummy_buf[10];
            if (control_transfer(RFNM_GET_SM_RESET, 1, dummy_buf, 50) != RFNM_API_OK) {
                spdlog::error("Failed to reset state machine");
                goto exit_eth;
            }
            spdlog::info("State machine reset via TCP control channel");
            reset_device_state();

            if (get(REQ_ALL)) {
                goto exit_eth;
            }

            // Connect the data socket after the state machine reset (which invalidates data
            // clients), and here rather than in a worker thread: a worker blocked in connect()
            // can't observe a shutdown request, which turned a device close during a stalled
            // connect into a use-after-free
            tcp_data_io_context = std::make_shared<asio::io_context>();
            tcp_data_socket = std::make_shared<asio::ip::tcp::socket>(*tcp_data_io_context);
            asio::ip::tcp::endpoint tcp_data_ep(asio::ip::make_address(rfnm_eth_transport_ip_addr), RFNM_TCP_DATA_PORT);
            tcp_data_socket->connect(tcp_data_ep);
            tcp_data_socket->set_option(asio::ip::tcp::no_delay(true));

            // Deliberately NOT setting SO_RCVBUF/SO_SNDBUF: an explicit value is clamped to
            // net.core.rmem_max (~208 KB on stock hosts) and locks the buffer there, pinning
            // the TCP receive window at ~256 KB - which capped streaming at ~1.6 Gbps (sender
            // 97.7% rwnd-limited). Kernel autotuning is exempt from that clamp and grows to
            // tcp_rmem max.

#if defined(__linux__) || defined(__APPLE__)
            // surface the negotiated MSS: jumbo frames decide whether the data path tops out
            // near ~77 or ~90 Msps on a 2.5G link, and a silently small MSS is otherwise
            // invisible to the user
            {
                int mss = 0;
                socklen_t mss_len = sizeof(mss);
                if (getsockopt(tcp_data_socket->native_handle(), IPPROTO_TCP, TCP_MAXSEG, &mss, &mss_len) == 0 && mss > 0) {
                    if (mss < 2000) {
                        spdlog::info("data path MSS {} (jumbo frames inactive): TCP streaming caps near ~77 Msps; "
                                     "set the host NIC MTU to 8000 on a jumbo-capable path for rates up to ~90 Msps", mss);
                    } else {
                        spdlog::info("data path MSS {} (jumbo frames active)", mss);
                    }
                }
            }
#endif

            tcp_data_connected = true;

            for (int8_t i = 0; i < THREAD_COUNT; i++) {
                thread_data[i].ep_id = i + 1;
                thread_data[i].rx_active = 0;
                thread_data[i].tx_active = 0;
                thread_data[i].shutdown_req = 0;
            }

            for (int i = 0; i < THREAD_COUNT; i++) {
                thread_c[i] = std::thread(&device::threadfn, this, i);
            }

            return;
        }
        catch (std::exception& e) {
            spdlog::error("TCP connection failed: {}", e.what());
        }
    }
exit_eth:
    throw std::runtime_error("Couldn't find any RFNM device");
}

MSDLL device::~device() {
    for (int8_t i = 0; i < THREAD_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
        thread_data[i].rx_active = 0;
        thread_data[i].tx_active = 0;
        thread_data[i].shutdown_req = 1;
        thread_data[i].cv.notify_all();
    }

    // A worker blocked in a data socket read can't observe shutdown_req, and close()
    // alone doesn't interrupt an in-progress blocking recv - shutdown() does. Checking
    // tcp_data_connected also guarantees the connecting thread finished the assignment.
    if (tcp_data_connected && tcp_data_socket) {
        asio::error_code ec;
        tcp_data_socket->shutdown(asio::ip::tcp::socket::shutdown_both, ec);
    }

    for (int8_t i = 0; i < THREAD_COUNT; i++) {
        if (thread_c[i].joinable()) {
            // Use async to implement timeout
            auto future = std::async(std::launch::async, [&]() {
                thread_c[i].join();
                });

            if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::timeout) {
                spdlog::warn("worker thread {} timed out, detaching", i);
                thread_c[i].detach();
            }
        }
    }

    // Clean up buffers
    if (rx_buffers_allocated) {
        rx_flush(0);
    }

    if (s->transport_status.transport == TRANSPORT_USB) {

        


        if (usb_handle && usb_handle->primary) {
            libusb_release_interface(usb_handle->primary, 0);
            libusb_close(usb_handle->primary);
        }
        if (usb_handle && usb_handle->boost) {
            libusb_release_interface(usb_handle->boost, 0);
            libusb_close(usb_handle->boost);
        }
        delete usb_handle;
        libusb_exit(NULL);
    }

    if (s->transport_status.transport == TRANSPORT_TCP) {
        tcp_data_connected = false;
        if (tcp_data_socket) {
            asio::error_code ec;
            tcp_data_socket->close(ec);
            tcp_data_socket.reset();
            tcp_data_io_context.reset();
        }
        if (rfnm_ctrl_ioctx_tcp)
            rfnm_ctrl_ioctx_tcp->stop();
        if (rfnm_ctrl_socket_tcp && rfnm_ctrl_socket_tcp->is_open()) {
            std::error_code ec;
            rfnm_ctrl_socket_tcp->close(ec);
        }
    }

    delete s;
}

static std::unordered_map<uint64_t, std::chrono::steady_clock::time_point> last_retx_time;
static std::mutex last_retx_mutex;
static constexpr auto RETRANS_BACKOFF = std::chrono::milliseconds(5);


void device::reorder_tx_queue_nolock(tx_buf_s& tx_s) {
    //std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);

    std::vector<tx_buf*> temp;

    while (!tx_s.in.empty()) {
        temp.push_back(tx_s.in.front());
        tx_s.in.pop();
    }

    std::sort(temp.begin(), temp.end(), [](const tx_buf* a, const tx_buf* b) {
        return a->usb_cc < b->usb_cc;
        });

    for (tx_buf* buf : temp) {
        tx_s.in.push(buf);
    }
}
#pragma pack(push,1)
struct retrans_req_entry {
    uint64_t seq;
    uint32_t adc;
};
#pragma pack(pop)




// ---- async USB TX engine: ordered endpoint + pipelined URBs -------------------------
// One sync-bulk thread is in-order but caps ~50-60 MSPS (transfer RTT gates it); many
// threads reach line rate but reorder the stream arbitrarily (holes). Async URBs
// pipelined on ONE endpoint give both: USB guarantees per-endpoint FIFO order, and
// K outstanding transfers keep the wire full. Single TX worker thread owns all state.
#define RFNM_TX_ASYNC_URBS 12
struct usb_tx_async_state {
    libusb_transfer* xfer[RFNM_TX_ASYNC_URBS];
    uint8_t* xbuf[RFNM_TX_ASYNC_URBS];
    rfnm::tx_buf* app[RFNM_TX_ASYNC_URBS];
    // 0 = free, 1 = in flight, 2 = done-ok, 3 = done-error (all set/read on the TX thread;
    // libusb callbacks run inside libusb_handle_events on the same thread)
    int state[RFNM_TX_ASYNC_URBS];
    int initialized;
};

static void rfnm_usb_tx_async_cb(struct libusb_transfer* t) {
    int* slot_state = (int*)t->user_data;
    *slot_state = (t->status == LIBUSB_TRANSFER_COMPLETED && t->actual_length == t->length) ? 2 : 3;
}

// async pipelined RX on all four IN endpoints (the RX mirror of the TX engine above).
// The sync one-shot reads discarded partially-received transfers on their 100 ms
// timeout - every discard is a silently lost packet, and under full-duplex load those
// losses fed the reorder machinery a constant stream of single-packet holes (the USB
// duplex collapse cycle). Async URBs persist across waits, so a slow scheduling turn
// delays a packet instead of destroying it. Per-endpoint reaping is sequential
// (next_reap ring) so each endpoint's stream stays in submission order; cross-endpoint
// skew is bounded by the pipeline depth and handled by the cc reorder queue as before.
#define RFNM_RX_ASYNC_EPS 4
#define RFNM_RX_ASYNC_URBS_PER_EP 6
struct usb_rx_async_state {
    libusb_transfer* xfer[RFNM_RX_ASYNC_EPS][RFNM_RX_ASYNC_URBS_PER_EP];
    uint8_t* xbuf[RFNM_RX_ASYNC_EPS][RFNM_RX_ASYNC_URBS_PER_EP];
    int state[RFNM_RX_ASYNC_EPS][RFNM_RX_ASYNC_URBS_PER_EP];  // 0 unsubmitted, 1 in flight, 2 done, 3 failed
    int next_reap[RFNM_RX_ASYNC_EPS];
    int initialized;
};
static void rfnm_usb_rx_async_cb(struct libusb_transfer* t) {
    int* slot_state = (int*)t->user_data;
    *slot_state = (t->status == LIBUSB_TRANSFER_COMPLETED) ? 2 : 3;
}
// -------------------------------------------------------------------------------------


void device::threadfn(size_t thread_index) {
    // sized for the local transport's grown cs16 packets; USB/TCP only use the prefix
    struct rfnm_rx_usb_buf* lrxbuf = (struct rfnm_rx_usb_buf*)new uint8_t[RFNM_LOCAL_RX_PACKET_SIZE]();
    struct rfnm_tx_usb_buf* ltxbuf = (struct rfnm_tx_usb_buf*)new uint8_t[RFNM_LOCAL_TX_PACKET_SIZE]();
    int transferred;
    auto& tpm = thread_data[thread_index];
    int r;

#ifdef BUILD_RFNM_LOCAL_TRANSPORT
    int data_ep_fp;
    if (s->transport_status.transport == TRANSPORT_LOCAL) {
        data_ep_fp = open("/dev/rfnm_data_ep", O_RDWR);
        if (data_ep_fp < 0) {
            return;
        }
    }
#endif

    if (s->transport_status.transport == TRANSPORT_TCP) {
        // Only two threads for TCP; the shared data socket was connected by the constructor
        if (thread_index >= 2) {
            delete[] (uint8_t*)lrxbuf;
            delete[] (uint8_t*)ltxbuf;
            return;
        }
    }

    while (!tpm.shutdown_req) {
        if (!tpm.rx_active && !tpm.tx_active) {
            {
                std::unique_lock lk(tpm.cv_mutex);
                // spurious wakeups are acceptable
                tpm.cv.wait(lk,
                    [this, thread_index] { return thread_data[thread_index].rx_active ||
                    thread_data[thread_index].tx_active ||
                    thread_data[thread_index].shutdown_req; });
            }
        }

        if (s->transport_status.transport == TRANSPORT_TCP && (tpm.ep_id % 2) == 0) {
            //   goto skip_rx;
                //receiver is blocking, so skip it in half the threads... 
        }



        if (tpm.rx_active) {
            // CHANGED: For TCP, only thread 1 handles RX
            if (s->transport_status.transport == TRANSPORT_TCP && thread_index != 1) {
                goto skip_rx;
            }
            // USB RX runs on one thread too: the async engine below keeps 24 URBs in
            // flight from a single reaper, which replaces the 16-thread sync-read pool
            if (s->transport_status.transport == TRANSPORT_USB && thread_index != 1) {
                goto skip_rx;
            }

            struct rx_buf* buf;

            if (getenv("RFNM_DEBUG_STALL") && thread_index == 1) {
                static thread_local auto last_census2 = std::chrono::high_resolution_clock::now();
                auto nowc2 = std::chrono::high_resolution_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(nowc2 - last_census2).count() > 1000) {
                    last_census2 = nowc2;
                    size_t rxin, rxout = 0, txin, txout;
                    { std::lock_guard<std::mutex> l(rx_s.in_mutex); rxin = rx_s.in.size(); }
                    for (int a = 0; a < 4; a++) { rxout += rx_s.out[a].size(); }
                    { std::lock_guard<std::mutex> l(tx_s.in_mutex); txin = tx_s.in.size(); }
                    { std::lock_guard<std::mutex> l(tx_s.out_mutex); txout = tx_s.out.size(); }
                    spdlog::info("CENSUS2 rx.in {} rx.out {} tx.in {} tx.out {}", rxin, rxout, txin, txout);
                }
            }

#if 0

            {
                std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);

                if (rx_s.in.empty()) {
                    goto skip_rx;
                }

                buf = rx_s.in.front();
                rx_s.in.pop();
            }
#else

            {
                std::unique_lock lk(rx_s.in_mutex);
                if (rx_s.in.empty()) {
                    //spdlog::error("rx_s.in.empty()");
                    rx_s.cv.wait_for(lk, std::chrono::microseconds(1000));
                    if (rx_s.in.empty()) {
                        //int qin = rx_s.in.size();
                        //spdlog::error("empty -> skipping rx");
                        goto skip_rx;
                    }
                }

                buf = rx_s.in.front();
                rx_s.in.pop();
            }


#endif
            if (s->transport_status.transport == TRANSPORT_USB) {
                if (!usb_handle->primary) {
                    spdlog::info("Thread {} detected force shutdown during USB op", thread_index);
                    break;
                }

                libusb_device_handle* lusb_handle = usb_handle->primary;
                if (1 && s->transport_status.usb_boost_connected) {
                    std::lock_guard<std::mutex> lockGuard(s_transport_pp_mutex);
                    //if (s->transport_status.boost_pp_rx) {
                    if ((tpm.ep_id % 2) == 0) {
                        lusb_handle = usb_handle->boost;
                    }
                    s->transport_status.boost_pp_rx = !s->transport_status.boost_pp_rx;
                }

                // async engine (boost handle unused here: single-cable benches leave it
                // disconnected and the pump keeps strict per-endpoint ordering on primary)
                static thread_local usb_rx_async_state arz = {};
                if (getenv("RFNM_DEBUG_STALL")) {
                    static thread_local auto last_census = std::chrono::high_resolution_clock::now();
                    auto nowc = std::chrono::high_resolution_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(nowc - last_census).count() > 1000) {
                        last_census = nowc;
                        int st[4] = {0,0,0,0};
                        for (int e = 0; e < RFNM_RX_ASYNC_EPS; e++) {
                            for (int q = 0; q < RFNM_RX_ASYNC_URBS_PER_EP; q++) {
                                st[arz.state[e][q] & 3]++;
                            }
                        }
                        size_t rxin, rxout = 0, txin, txout;
                        { std::lock_guard<std::mutex> l(rx_s.in_mutex); rxin = rx_s.in.size(); }
                        for (int a = 0; a < 4; a++) { rxout += rx_s.out[a].size(); }
                        { std::lock_guard<std::mutex> l(tx_s.in_mutex); txin = tx_s.in.size(); }
                        { std::lock_guard<std::mutex> l(tx_s.out_mutex); txout = tx_s.out.size(); }
                        spdlog::info("CENSUS rx.in {} rx.out {} tx.in {} tx.out {} rxURB f{}/i{}/d{}/e{}",
                            rxin, rxout, txin, txout, st[0], st[1], st[2], st[3]);
                    }
                }
                if (!arz.initialized) {
                    libusb_device_handle* prim_handle = usb_handle->primary;   // never the boost: on
                    // multi-board hosts the boost discovery can bind a DIFFERENT board's interface
                    for (int e = 0; e < RFNM_RX_ASYNC_EPS; e++) {
                        for (int q = 0; q < RFNM_RX_ASYNC_URBS_PER_EP; q++) {
                            arz.xfer[e][q] = libusb_alloc_transfer(0);
                            arz.xbuf[e][q] = (uint8_t*)malloc(RFNM_USB_RX_PACKET_SIZE);
                            libusb_fill_bulk_transfer(arz.xfer[e][q], prim_handle, ((e + 1) | LIBUSB_ENDPOINT_IN),
                                arz.xbuf[e][q], RFNM_USB_RX_PACKET_SIZE, rfnm_usb_rx_async_cb, &arz.state[e][q], 0);
                            arz.state[e][q] = libusb_submit_transfer(arz.xfer[e][q]) ? 3 : 1;
                        }
                        arz.next_reap[e] = 0;
                    }
                    arz.initialized = 1;
                }
                int got_ep = -1;
                static thread_local int rx_reap_rr = 0;
                static thread_local auto rx_last_progress = std::chrono::high_resolution_clock::now();
                auto rx_tstart = std::chrono::high_resolution_clock::now();
                // UDC stall recovery, host-driven: if no completion for 2 s while URBs
                // are pending, the gadget's UDC has stopped mapping requests to TRBs (a
                // dwc3 lost-event class bug). SET_INTERFACE re-runs the gadget's set_alt
                // (full endpoint re-arm + request requeue device-side) AND resets the
                // host xHCI endpoint state - the only recovery that resyncs BOTH sides
                // (device-only re-arm desyncs SS sequence state; session re-open works
                // for the same reason this does).
                if (std::chrono::duration_cast<std::chrono::milliseconds>(rx_tstart - rx_last_progress).count() > 2000) {
                    spdlog::warn("RX pipeline dead 2s: SET_INTERFACE resync");
                    for (int e = 0; e < RFNM_RX_ASYNC_EPS; e++) {
                        for (int q = 0; q < RFNM_RX_ASYNC_URBS_PER_EP; q++) {
                            if (arz.state[e][q] == 1) {
                                libusb_cancel_transfer(arz.xfer[e][q]);
                            }
                        }
                    }
                    for (int spin = 0; spin < 60; spin++) {
                        struct timeval tvc = { 0, 2000 };
                        libusb_handle_events_timeout(nullptr, &tvc);
                        int busy = 0;
                        for (int e = 0; e < RFNM_RX_ASYNC_EPS; e++) {
                            for (int q = 0; q < RFNM_RX_ASYNC_URBS_PER_EP; q++) {
                                busy += (arz.state[e][q] == 1);
                            }
                        }
                        if (!busy) {
                            break;
                        }
                    }
                    libusb_set_interface_alt_setting(lusb_handle, 0, 0);
                    for (int e = 0; e < RFNM_RX_ASYNC_EPS; e++) {
                        for (int q = 0; q < RFNM_RX_ASYNC_URBS_PER_EP; q++) {
                            arz.state[e][q] = libusb_submit_transfer(arz.xfer[e][q]) ? 3 : 1;
                        }
                        arz.next_reap[e] = 0;
                    }
                    rx_reap_rr = 0;
                    rx_last_progress = std::chrono::high_resolution_clock::now();
                }
                while (got_ep < 0) {
                    // fair rotation across endpoints: preferential draining of one endpoint
                    // exhausts the others' URB pools (all slots completed-but-unreaped), the
                    // gadget's requests on those endpoints starve, and the device ring jumps
                    // forward over the parked packets - giant cc holes and a wedged stream
                    for (int i = 0; i < RFNM_RX_ASYNC_EPS; i++) {
                        int e = (rx_reap_rr + i) % RFNM_RX_ASYNC_EPS;
                        int q = arz.next_reap[e];
                        if (arz.state[e][q] == 2) { got_ep = e; rx_reap_rr = (e + 1) % RFNM_RX_ASYNC_EPS; break; }
                        if (arz.state[e][q] == 3) {
                            // failed HEAD slot: consume the failure in order - resubmit and
                            // advance. Resubmitting a non-head slot would file it behind its
                            // siblings in the endpoint queue and permanently desync next_reap
                            // from the endpoint's completion order (wedge: heads in flight,
                            // completed slots parked unreapable behind them).
                            if (arz.xfer[e][q]->status == LIBUSB_TRANSFER_NO_DEVICE) {
                                spdlog::info("Thread {} USB device lost, exiting", thread_index);
                                goto usb_rx_dead;
                            }
                            // a device-side endpoint re-arm (UDC stall recovery) surfaces
                            // here as STALL: the pipe must be cleared before any resubmit
                            // can succeed, else the client stays dead while the board is
                            // healthy again
                            if (arz.xfer[e][q]->status == LIBUSB_TRANSFER_STALL) {
                                libusb_clear_halt(lusb_handle, arz.xfer[e][q]->endpoint);
                                spdlog::info("RX ep {:#x} halt cleared", arz.xfer[e][q]->endpoint);
                            }
                            arz.state[e][q] = libusb_submit_transfer(arz.xfer[e][q]) ? 3 : 1;
                            if (arz.state[e][q] == 1) {
                                arz.next_reap[e] = (q + 1) % RFNM_RX_ASYNC_URBS_PER_EP;
                            }
                        }
                    }
                    if (got_ep < 0) {
                        struct timeval tv = { 0, 2000 };
                        libusb_handle_events_timeout(nullptr, &tv);
                        auto rx_us = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::high_resolution_clock::now() - rx_tstart);
                        if (rx_us.count() > 100000) {
                            break;  // nothing completed in 100 ms: not an error, nothing was lost
                        }
                    }
                }
                if (got_ep < 0) {
                    std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                    rx_s.in.push(buf);
                    rx_s.cv.notify_one();
                    goto skip_rx;
                }
                if (0) {
                usb_rx_dead:
                    break;
                }
                rx_last_progress = std::chrono::high_resolution_clock::now();
                {
                    int q = arz.next_reap[got_ep];
                    transferred = arz.xfer[got_ep][q]->actual_length;
                    memcpy(lrxbuf, arz.xbuf[got_ep][q], transferred);
                    arz.state[got_ep][q] = libusb_submit_transfer(arz.xfer[got_ep][q]) ? 3 : 1;
                    arz.next_reap[got_ep] = (q + 1) % RFNM_RX_ASYNC_URBS_PER_EP;
                }
                r = 0;

                // variable-size packet: the device shortens the wire transfer to the valid
                // sample prefix and declares the count in the header (latency-deadline flush).
                // The transfer may also arrive PADDED past the declared payload (full-size
                // wire transfer carrying a partial packet) - accept any transfer that contains
                // at least the declared payload with a valid head; reject short/garbage ones.
                if (lrxbuf->magic != 0x7ab8bd6f || lrxbuf->elem_cnt > RFNM_USB_RX_PACKET_ELEM_CNT ||
                        (size_t)transferred < RFNM_USB_RX_PACKET_HEAD_SIZE + (size_t)lrxbuf->elem_cnt * 3) {
                    spdlog::error("thread loop RX usb wrong size, {}, ep {}, elem_cnt {}, magic {:x}", transferred, tpm.ep_id, (uint32_t)lrxbuf->elem_cnt, (uint32_t)lrxbuf->magic);
                    std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                    rx_s.in.push(buf);
                    rx_s.cv.notify_one();
                    goto skip_rx;
                }
            }

            if (s->transport_status.transport == TRANSPORT_LOCAL) {
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
#if 0
                if (ioctl(data_ep_fp, RFNM_IOCTL_BASE_DATA | 1, (uint8_t*)lrxbuf) < 0) {
                    //spdlog::error("thread loop RX no data");
                    {
                        std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                        rx_s.in.push(buf);
                        rx_s.cv.notify_one();
                    }
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                    goto skip_rx;
                }
#else

                struct pollfd pfd;
                pfd.fd = data_ep_fp;
                pfd.events = POLLIN;

                int ret = poll(&pfd, 1, /* timeout_ms= */ 5);
                if (ret < 0) {
                    perror("poll");
                    // handle error…
                }
                else if (ret == 0) {

                    // poll timeout: no local packet within 5 ms. Normal idle tick at low
                    // sample rates (a 2.4 MSps stream fills a packet slower than the poll
                    // period), so it is not logged - it used to spam ~115 lines/s here.

                    {
                        std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                        rx_s.in.push(buf);
                        rx_s.cv.notify_one();
                    }
                    goto skip_rx;



                }
                else if (pfd.revents & POLLIN) {
                    // safe to call RX ioctl once
                    if (ioctl(data_ep_fp, RFNM_IOCTL_BASE_DATA | 1, (uint8_t*)lrxbuf) < 0) {
                        spdlog::error("RX queue error");
                    }
                    else {
                        // got data in lrxbuf
                        //spdlog::info("RX {}", static_cast<uint64_t>(lrxbuf->usb_cc));

                    }
                }


#endif
#endif
            }

            if (s->transport_status.transport == TRANSPORT_TCP) {
                // CHANGED: Receive from TCP instead of UDP with mutex protection
                try {
                    std::lock_guard<std::mutex> lock(tcp_data_rx_mutex);
                    // asio::read will read exactly RFNM_USB_RX_PACKET_SIZE bytes or throw an exception
                    asio::read(*tcp_data_socket, asio::buffer((uint8_t*)lrxbuf, RFNM_USB_RX_PACKET_SIZE));

                    if (rx_s.usb_cc[lrxbuf->adc_id] != UINT64_MAX && lrxbuf->usb_cc < rx_s.usb_cc[lrxbuf->adc_id]) {
                        std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                        rx_s.in.push(buf);
                        rx_s.cv.notify_one();
                        goto skip_rx;
                    }

                    {
                        std::lock_guard<std::mutex> lockGuard(rx_s.benchmark_mutex);
                        rx_s.usb_cc_benchmark[tpm.ep_id % 4]++;
                    }

                }
                catch (std::exception& e) {
                    std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                    rx_s.in.push(buf);
                    rx_s.cv.notify_one();

                    //spdlog::error("TCP RX problem: {}", e.what());
                    goto skip_rx;
                }
            }


            if (lrxbuf->magic != 0x7ab8bd6f || lrxbuf->adc_id > 3 ||
                    lrxbuf->elem_cnt == 0 || lrxbuf->elem_cnt > RFNM_USB_RX_PACKET_ELEM_CNT) {
                if (getenv("RFNM_DEBUG_RX")) {
                    static std::atomic<int> dbg_rej{0};
                    int nr = ++dbg_rej;
                    if (nr <= 10 || nr % 500 == 0) {
                        spdlog::info("REJECT n {} magic {:x} adc {} elem_cnt {} transferred {}", nr,
                            (uint32_t)lrxbuf->magic, (uint32_t)lrxbuf->adc_id, (uint32_t)lrxbuf->elem_cnt, transferred);
                    }
                }
                std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                rx_s.in.push(buf);
                rx_s.cv.notify_one();
                goto skip_rx;
            }

            if (lrxbuf->fmt == RFNM_PACKET_FMT_CS16) {
                // local transport: the kernel already unpacked 12->16, only the output
                // format conversion remains (single unpack total on this SoC)
                if (s->transport_status.rx_stream_format == STREAM_FORMAT_CS8) {
                    convert_cs16_to_cs8(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
                }
                else if (s->transport_status.rx_stream_format == STREAM_FORMAT_CS16) {
                    convert_cs16_to_cs16(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
                }
                else if (s->transport_status.rx_stream_format == STREAM_FORMAT_CF32) {
                    convert_cs16_to_cf32(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
                }
            }
            else if (s->transport_status.rx_stream_format == STREAM_FORMAT_CS8) {
                unpack_12_to_cs8(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
            }
            else if (s->transport_status.rx_stream_format == STREAM_FORMAT_CS16) {
                unpack_12_to_cs16(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
            }
            else if (s->transport_status.rx_stream_format == STREAM_FORMAT_CF32) {
                unpack_12_to_cf32(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
            }

            buf->adc_cc = lrxbuf->adc_cc;
            buf->adc_id = lrxbuf->adc_id;
            buf->usb_cc = lrxbuf->usb_cc;
            buf->phytimer = lrxbuf->phytimer;
            buf->elem_cnt = lrxbuf->elem_cnt;

            if (getenv("RFNM_DEBUG_RX")) {
                static std::atomic<int> dbg_push{0};
                int n = ++dbg_push;
                if (n <= 15 || n % 100 == 0) {
                    spdlog::info("PUSH n {} adc {} usb_cc {} magic_ok", n, (uint32_t)lrxbuf->adc_id, (uint64_t)lrxbuf->usb_cc);
                }
            }
            if (getenv("RFNM_DEBUG_RX_CCLOG")) {
                static FILE *ccf = fopen(getenv("RFNM_DEBUG_RX_CCLOG"), "w");
                if (ccf) {
                    fprintf(ccf, "%llu %u %u\n", (unsigned long long)lrxbuf->usb_cc, (uint32_t)lrxbuf->adc_id, (uint32_t)lrxbuf->elem_cnt);
                }
            }
            {
                std::lock_guard<std::mutex> lockGuard(rx_s.out_mutex);
                rx_s.out[lrxbuf->adc_id].push(buf);

                //if (rx_s.out[lrxbuf->adc_id].size() > 50) {
                rx_s.cv.notify_one();
                //}
            }





        }

    skip_rx:

        if (tpm.tx_active) {
            // CHANGED: For TCP, only thread 0 handles TX
            if (s->transport_status.transport == TRANSPORT_TCP && thread_index != 0) {
                goto read_dev_status;
            }

            struct tx_buf* buf;
#if 0
            if (tx_s.in.empty()) {
                goto read_dev_status;
            }
#else

            {
                std::unique_lock lk(tx_s.in_mutex);
                if (tx_s.in.empty()) {
                    //spdlog::error("rx_s.in.empty()");
                    tx_s.cv.wait_for(lk, std::chrono::microseconds(1000));
                    if (tx_s.in.empty()) {
                        goto read_dev_status;
                    }
                }
            }

#endif


            // you can have the reorder either here or down there but let's keep it down there for now, mostly untested
            {
                //std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                //reorder_tx_queue_nolock(tx_s);
            }



            {
                std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);

                if (tx_s.in.empty()) {
                    goto read_dev_status;
                }
                buf = tx_s.in.front();
                tx_s.in.pop();
            }



            uint32_t tx_elems = buf->elem_cnt ? buf->elem_cnt : RFNM_USB_TX_PACKET_ELEM_CNT;
            uint32_t tx_multi = (tx_elems * 3) / LA_TX_BASE_BUFSIZE_12;
            uint32_t tx_wire_len = RFNM_USB_TX_PACKET_HEAD_SIZE + tx_multi * LA_TX_BASE_BUFSIZE_12;
            if (s->transport_status.transport == TRANSPORT_LOCAL) {
                // same SoC: no reason to pack 16->12 only for the kernel to unpack again
                memcpy((uint8_t*)ltxbuf + RFNM_USB_TX_PACKET_HEAD_SIZE, buf->buf, (size_t)tx_elems * 4);
                ltxbuf->fmt = RFNM_PACKET_FMT_CS16;
                tx_wire_len = RFNM_USB_TX_PACKET_HEAD_SIZE + tx_multi * LA_TX_BASE_BUFSIZE;
            } else {
                pack_cs16_to_12((uint8_t*)ltxbuf->buf, buf->buf, tx_elems);
                ltxbuf->fmt = RFNM_PACKET_FMT_PACKED12;
            }
            ltxbuf->dac_cc = buf->dac_cc;
            ltxbuf->dac_id = buf->dac_id;
            ltxbuf->usb_cc = buf->usb_cc;
            ltxbuf->phytimer = buf->phytimer;
            ltxbuf->multi = tx_multi;
            ltxbuf->magic = 0x758f4d4a;

            if (s->transport_status.transport == TRANSPORT_USB) {
                if (!usb_handle->primary) {
                    spdlog::info("Thread {} detected force shutdown during USB op", thread_index);
                    break;
                }
                libusb_device_handle* lusb_handle = usb_handle->primary;

                // async pipelined sends on ONE ordered endpoint (see engine notes above)
                static thread_local usb_tx_async_state az = {};
                if (!az.initialized) {
                    for (int q = 0; q < RFNM_TX_ASYNC_URBS; q++) {
                        az.xfer[q] = libusb_alloc_transfer(0);
                        az.xbuf[q] = (uint8_t*)malloc(RFNM_USB_TX_PACKET_SIZE);
                        az.state[q] = 0;
                    }
                    az.initialized = 1;
                }
                struct timeval tv_zero = {0, 0};
                libusb_handle_events_timeout(nullptr, &tv_zero);
                for (int q = 0; q < RFNM_TX_ASYNC_URBS; q++) {
                    if (az.state[q] == 2) {
                        std::lock_guard<std::mutex> lockGuard(tx_s.out_mutex);
                        tx_s.out.push(az.app[q]);
                        tx_s.cv.notify_one();
                        az.state[q] = 0;
                    } else if (az.state[q] == 3) {
                        spdlog::error("TX async transfer failed (status {} len {}/{}), requeueing app buf",
                            (int)az.xfer[q]->status, az.xfer[q]->actual_length, az.xfer[q]->length);
                        if (az.xfer[q]->status == LIBUSB_TRANSFER_STALL) {
                            libusb_clear_halt(lusb_handle, (1 | LIBUSB_ENDPOINT_OUT));
                            spdlog::info("TX ep halt cleared");
                        }
                        std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                        tx_s.in.push(az.app[q]);
                        reorder_tx_queue_nolock(tx_s);
                        az.state[q] = 0;
                    }
                }
                int slot = -1;
                for (int q = 0; q < RFNM_TX_ASYNC_URBS; q++) {
                    if (az.state[q] == 0) { slot = q; break; }
                }
                if (slot < 0) {
                    // pipeline full: wait for a completion, reap it, retry the SAME buf
                    // immediately - bouncing through the dev-status path per packet gated
                    // throughput at the worker pass rate (~1k pkt/s; 122.88M needs 6k).
                    {
                        std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                        tx_s.in.push(buf);
                        reorder_tx_queue_nolock(tx_s);
                    }
                    struct timeval tv_wait = {0, 2000};
                    libusb_handle_events_timeout(nullptr, &tv_wait);
                    for (int q = 0; q < RFNM_TX_ASYNC_URBS; q++) {
                        if (az.state[q] == 2) {
                            std::lock_guard<std::mutex> lockGuard(tx_s.out_mutex);
                            tx_s.out.push(az.app[q]);
                            tx_s.cv.notify_one();
                            az.state[q] = 0;
                        }
                    }
                    continue;
                }
                struct rfnm_tx_usb_buf* axbuf = (struct rfnm_tx_usb_buf*)az.xbuf[slot];
                memcpy(axbuf, ltxbuf, RFNM_USB_TX_PACKET_HEAD_SIZE);
                memcpy(axbuf->buf, ltxbuf->buf, tx_wire_len - RFNM_USB_TX_PACKET_HEAD_SIZE);
                az.app[slot] = buf;
                az.state[slot] = 1;
                libusb_fill_bulk_transfer(az.xfer[slot], lusb_handle, (1 | LIBUSB_ENDPOINT_OUT),
                    (uint8_t*)axbuf, tx_wire_len, rfnm_usb_tx_async_cb, &az.state[slot], 1000);
                r = libusb_submit_transfer(az.xfer[slot]);
                if (r) {
                    spdlog::error("TX async submit fail {}", r);
                    az.state[slot] = 0;
                    std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                    tx_s.in.push(buf);
                    reorder_tx_queue_nolock(tx_s);
                    goto read_dev_status;
                }
                // submitted in order; completion recycles the app buf on a later pass.
                // Keep draining the input queue while URB slots remain - one submit per
                // worker pass caps throughput at the pass rate (~1-3k pkt/s, measured
                // 21.7 MSPS at 122.88M); the wire needs 6k pkt/s of full packets.
                {
                    std::unique_lock lk(tx_s.in_mutex);
                    if (!tx_s.in.empty()) {
                        bool have_free = false;
                        for (int q = 0; q < RFNM_TX_ASYNC_URBS; q++) {
                            if (az.state[q] == 0) { have_free = true; break; }
                        }
                        if (have_free) {
                            lk.unlock();
                            continue;
                        }
                    }
                }
                goto read_dev_status;

                r = libusb_bulk_transfer(lusb_handle, (((tpm.ep_id % 4) + 1) | LIBUSB_ENDPOINT_OUT),
                    (uint8_t*)ltxbuf, tx_wire_len, &transferred, 100);
                if (r == LIBUSB_ERROR_NO_DEVICE || r == LIBUSB_ERROR_IO) {
                    // Device was reset or closed, exit gracefully
                    spdlog::info("Thread {} USB device lost, exiting", thread_index);
                    break;
                }
                else if (r) {
                    spdlog::error("TX bulk tx fail {} {}", tpm.ep_id, r);
                    std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                    tx_s.in.push(buf);
                    reorder_tx_queue_nolock(tx_s);
                    goto read_dev_status;
                }

                if (transferred != (int)tx_wire_len) {
                    spdlog::error("thread loop TX usb wrong size, {}, {}", transferred, tpm.ep_id);
                    std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                    tx_s.in.push(buf);
                    reorder_tx_queue_nolock(tx_s);
                    goto read_dev_status;
                }
            }

            if (s->transport_status.transport == TRANSPORT_LOCAL) {
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
#if 0
                if (ioctl(data_ep_fp, RFNM_IOCTL_BASE_DATA | 0, (uint8_t*)ltxbuf) < 0) {
                    //spdlog::error("thread loop TX busy");
                    //perror("ioctl failed");
                    //printf("KO %d\n", ltxbuf->usb_cc);
                    {
                        std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                        tx_s.in.push(buf);
                        reorder_tx_queue_nolock(tx_s);
                    }
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                    goto read_dev_status;
                }
                else {
                    //printf("ok %d\n", ltxbuf->usb_cc);
                }
#else
                //spdlog::error("tx loop");

                struct pollfd pfd;
                pfd.fd = data_ep_fp;
                pfd.events = POLLOUT;

                int ret = poll(&pfd, 1, /* timeout_ms= */ 5);
                if (ret < 0) {
                    perror("poll");
                    // handle error…
                }
                else if (ret == 0) {

                    spdlog::error("tx pool timeout");

                    // timeout

                    {
                        std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                        tx_s.in.push(buf);
                        reorder_tx_queue_nolock(tx_s);
                    }
                    goto read_dev_status;



                }
                else if (pfd.revents & POLLOUT) {
                    // safe to call RX ioctl once
                    if (ioctl(data_ep_fp, RFNM_IOCTL_BASE_DATA | 0, (uint8_t*)ltxbuf) < 0) {
                        spdlog::error("TX queue error");
                    }
                    else {
                        //spdlog::error("tx done");
                        // got data in lrxbuf
                    }
                }




#endif
#endif
            }

            if (s->transport_status.transport == TRANSPORT_TCP) {
                // CHANGED: Send over TCP instead of UDP with mutex protection
                try {
                    std::lock_guard<std::mutex> lock(tcp_data_tx_mutex);
                    asio::write(*tcp_data_socket, asio::buffer((uint8_t*)ltxbuf, tx_wire_len));
                }
                catch (std::exception& e) {
                    spdlog::error("TCP TX error: {}", e.what());
                    std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                    tx_s.in.push(buf);
                    reorder_tx_queue_nolock(tx_s);
                    goto read_dev_status;
                }
            }



            {
                std::lock_guard<std::mutex> lockGuard(tx_s.out_mutex);
                tx_s.out.push(buf);
                tx_s.cv.notify_one();
            }
        }

    read_dev_status:

        if (s->transport_status.transport == TRANSPORT_USB && thread_index > 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        {
#if 1
            using std::chrono::high_resolution_clock;
            using std::chrono::duration_cast;
            using std::chrono::duration;
            using std::chrono::milliseconds;

            auto tlast = s->last_dev_time;
            auto tnow = high_resolution_clock::now();
            auto ms_int = duration_cast<milliseconds>(tnow - tlast);

            // TX closed-loop pacing rides on dev_status freshness; with small TX packets a
            // 5 ms status lag forces deep pipelines (or underruns), so poll at >=1 ms.
            if (1 && ms_int.count() > 1) {


                if (s->transport_status.transport == TRANSPORT_TCP) {
                    // NOTE: Retransmission would need to be reimplemented for TCP
                    // TCP already handles retransmission at the protocol level
                    // This section can be removed or replaced with TCP-specific error handling
                }

                //if (s_dev_status_mutex.try_lock())
                std::unique_lock<std::mutex> lock(s_dev_status_mutex, std::try_to_lock);
                if (lock)
                {
                    //std::lock_guard<std::mutex> lockGuard(s_dev_status_mutex);


                    struct rfnm_dev_status dev_status[1];

                    if (/*(rand() % 10 == 0) ||*/ control_transfer(RFNM_GET_DEV_STATUS, sizeof(struct rfnm_dev_status), (unsigned char*)&dev_status[0], 50) != RFNM_API_OK) {
                        spdlog::error("control_transfer for RFNM_GET_DEV_STATUS failed");
                        //return RFNM_API_USB_FAIL;

                        if (1 && ms_int.count() > 25 && s->transport_status.transport != TRANSPORT_TCP) {
                            spdlog::error("stopping stream");

                            for (int8_t i = 0; i < THREAD_COUNT; i++) {
                                thread_data[i].rx_active = 0;
                                thread_data[i].tx_active = 0;
                                thread_data[i].shutdown_req = 1;
                            }
                        }
                    }
                    else {
                        memcpy(&s->dev_status, &dev_status[0], sizeof(struct rfnm_dev_status));
                        s->last_dev_time = high_resolution_clock::now();
                        uint64_t tt = s->dev_status.usb_dac_last_dqbuf[0];

                        /*spdlog::info("control_transfer for RFNM_GET_DEV_STATUS OK {} {} adc {} {} {} {}", tx_s.usb_cc, tt,

                        static_cast<uint64_t>(s->dev_status.usb_adc_last_qbuf[0]),
                    static_cast<uint64_t>(s->dev_status.usb_adc_last_qbuf[1]),
                static_cast<uint64_t>(s->dev_status.usb_adc_last_qbuf[2]),
            static_cast<uint64_t>(s->dev_status.usb_adc_last_qbuf[3])
        );*/

                    }

                    //s_dev_status_mutex.unlock();
                }
            }
#endif        
        }
    }

    //spdlog::error("exiting thread");
    if (lrxbuf) {
        delete[] (uint8_t*)lrxbuf;
    }
    //spdlog::error("past delete");
    if (ltxbuf) {
        delete[] (uint8_t*)ltxbuf;
    }
    //spdlog::error("past second delete");

    // TCP data socket teardown happens in the destructor after all threads have
    // joined, since the connection is shared by the RX and TX worker threads

    if (s->transport_status.transport == TRANSPORT_LOCAL) {
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
        close(data_ep_fp);
#endif
    }
}




// A device can enumerate at most once per transport
static bool sn_already_found(const std::vector<struct dev_info>& found, enum transport transport, const uint8_t* sn) {
    for (auto& dev : found) {
        if (dev.transport == transport &&
            std::memcmp(dev.hwinfo.motherboard.serial_number, sn, sizeof(dev.hwinfo.motherboard.serial_number)) == 0) {
            return true;
        }
    }
    return false;
}

MSDLL std::vector<struct dev_info> device::find(enum transport transport, std::string address) {
    int dev_cnt = 0;
    int r;
    std::vector<struct dev_info> found = {};

    if (transport == TRANSPORT_USB || transport == TRANSPORT_FIND) {
        libusb_device** devs = NULL;

#if LIBUSB_API_VERSION >= 0x0100010A
        r = libusb_init_context(nullptr, nullptr, 0);
#else
        r = libusb_init(nullptr);
#endif
        if (r < 0) {
            spdlog::error("RFNMDevice::activateStream() -> failed to initialize libusb");
            goto exit_usb;
        }

        dev_cnt = libusb_get_device_list(NULL, &devs);
        if (dev_cnt < 0) {
            spdlog::error("failed to get list of usb devices");
            goto exit_usb;
        }

        for (int d = 0; d < dev_cnt; d++) {
            struct libusb_device_descriptor desc;
            libusb_device_handle* thandle{};
            int r = libusb_get_device_descriptor(devs[d], &desc);
            if (r < 0) {
                spdlog::error("failed to get usb dev descr");
                continue;
            }

            if (desc.idVendor != RFNM_USB_VID || desc.idProduct != RFNM_USB_PID) {
                continue;
            }

            r = libusb_open(devs[d], &thandle);
            if (r) {
                spdlog::error("Found RFNM device, but couldn't open it {}", r);
                continue;
            }

            uint8_t sn[9];
            if (libusb_get_string_descriptor_ascii(thandle, desc.iSerialNumber, sn, 9) >= 0) {
                sn[8] = '\0';
            }
            else {
                // still list the device: an empty address opens the first available unit
                spdlog::error("Couldn't read serial descr");
                sn[0] = '\0';
            }

            if (address.length() && strcmp((const char*)sn, address.c_str())) {
                spdlog::info("This serial {} doesn't match the requested {}", (const char*)sn, address);
                goto next;
            }

            if (libusb_get_device_speed(libusb_get_device(thandle)) < LIBUSB_SPEED_SUPER) {
                spdlog::error("You are connected using USB 2.0 (480 Mbps), however USB 3.0 (5000 Mbps) is required. "
                    "Please make sure that the cable and port you are using can work with USB 3.0 SuperSpeed");
                goto next;
            }

            r = libusb_claim_interface(thandle, 0);
            if (r < 0) {
                spdlog::error("Found RFNM device, but couldn't claim the interface, {}, {}", r, libusb_strerror((libusb_error)r));
                goto next;
            }

            struct rfnm_dev_hwinfo r_hwinfo;

            r = libusb_control_transfer(thandle, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                RFNM_GET_DEV_HWINFO, 0, (unsigned char*)&r_hwinfo, sizeof(struct rfnm_dev_hwinfo), 50);
            if (r < 0) {
                spdlog::error("libusb_control_transfer for REQ_HWINFO failed");
                goto next;
            }

            if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
                uint32_t pv = r_hwinfo.protocol_version;
                spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED detected protocol is v{} while your librfnm is for v{}", pv, RFNM_PROTOCOL_VERSION);
                goto next;
            }

            if (sn_already_found(found, TRANSPORT_USB, r_hwinfo.motherboard.serial_number)) {
                spdlog::info("Skip USB device (duplicate SN): {}", (char*)r_hwinfo.motherboard.serial_number);
                goto next;
            }
            spdlog::info("Add USB device: {}", (char*)r_hwinfo.motherboard.serial_number);
            found.push_back({ TRANSPORT_USB, std::string((const char*)sn), r_hwinfo });

        next:
            libusb_release_interface(thandle, 0);
            libusb_close(thandle);
        }

    exit_usb:
        libusb_free_device_list(devs, 1);
        libusb_exit(NULL);
    }

    if (transport == TRANSPORT_LOCAL || transport == TRANSPORT_FIND) {
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
        int fd = open("/dev/rfnm_ctrl_ep", O_RDWR);
        if (fd < 0) {
            perror("open");
            goto exit_local;
        }

        uint8_t ep_ctrl_buf[RFNM_SYSCTL_TRANSFER_SIZE];

        if (ioctl(fd, RFNM_IOCTL_BASE + (0xff & RFNM_GET_DEV_HWINFO), &ep_ctrl_buf) < 0) {
            perror("ioctl GET_STATS");
            close(fd);
            goto exit_close_local;
        }

        struct rfnm_dev_hwinfo r_hwinfo;

        memcpy(&r_hwinfo, &ep_ctrl_buf[0], sizeof(struct rfnm_dev_hwinfo));

        if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
            uint32_t pv = r_hwinfo.protocol_version;
            spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED detected protocol is v{} while your librfnm is for v{}", pv, RFNM_PROTOCOL_VERSION);
            goto exit_close_local;
        }

        if (sn_already_found(found, TRANSPORT_LOCAL, r_hwinfo.motherboard.serial_number)) {
            spdlog::info("Skip local device (duplicate SN): {}", (char*)r_hwinfo.motherboard.serial_number);
            goto exit_close_local;
        }
        spdlog::info("Add local device: {}", (char*)r_hwinfo.motherboard.serial_number);
        found.push_back({ TRANSPORT_LOCAL, "", r_hwinfo });

    exit_close_local:
        close(fd);
#endif
    }

exit_local:
    if (transport == TRANSPORT_TCP || transport == TRANSPORT_FIND) {
        try {
            asio::io_context io_context;

            std::vector<std::string> probe_addrs;
            if (address.length()) {
                probe_addrs.push_back(address);
            }
            else {
                probe_addrs = get_broadcast_addresses();
            }

            if (!probe_addrs.empty()) {
                asio::ip::udp::socket socket(io_context);
                socket.open(asio::ip::udp::v4());
                socket.non_blocking(true);

                if (!address.length()) {
                    asio::socket_base::broadcast broadcast_option(true);
                    socket.set_option(broadcast_option);
                }

                uint8_t message[1];
                message[0] = ((int)RFNM_GET_DEV_HWINFO) & 0xff;

                for (auto& probe_addr : probe_addrs) {
                    asio::ip::udp::endpoint remote_endpoint(asio::ip::make_address(probe_addr), RFNM_UDP_CTRL_PORT);
                    asio::error_code ec;
                    socket.send_to(asio::buffer(message), remote_endpoint, 0, ec);
                    if (ec) {
                        // one unroutable interface must not abort discovery on the others
                        spdlog::info("Discovery probe to {} failed: {}", probe_addr, ec.message());
                    }
                }

                // A broadcast can be answered by any number of devices, so keep collecting
                // replies until the discovery window closes
                char reply[1152];
                auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
                while (std::chrono::steady_clock::now() < deadline) {
                    asio::ip::udp::endpoint sender_endpoint;
                    asio::error_code ec;
                    size_t reply_length = socket.receive_from(asio::buffer(reply), sender_endpoint, 0, ec);

                    if (ec == asio::error::would_block || ec == asio::error::try_again) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }
                    if (ec) {
                        spdlog::error("UDP receive failed: {}", ec.message());
                        break;
                    }

                    if (reply_length != sizeof(rfnm_dev_hwinfo) + 1) {
                        spdlog::error("Reply too short ({} bytes)", reply_length);
                        continue;
                    }
                    if (reply[0] != (RFNM_GET_DEV_HWINFO & 0xff)) {
                        spdlog::error("UDP control transfer conflict got {} should have been RFNM_GET_DEV_HWINFO", (uint8_t)reply[0]);
                        continue;
                    }

                    rfnm_dev_hwinfo r_hwinfo;
                    memcpy(&r_hwinfo, &reply[1], sizeof(rfnm_dev_hwinfo));
                    if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
                        uint32_t mv = r_hwinfo.protocol_version;
                        spdlog::error("Protocol version mismatch: received {}, expected {}", mv, RFNM_PROTOCOL_VERSION);
                        continue;
                    }

                    if (sn_already_found(found, TRANSPORT_TCP, r_hwinfo.motherboard.serial_number)) {
                        spdlog::info("Skip Network device (duplicate SN): {}", (char*)r_hwinfo.motherboard.serial_number);
                        continue;
                    }

                    spdlog::info("Add Network device: {}", (char*)r_hwinfo.motherboard.serial_number);
                    found.push_back({ TRANSPORT_TCP, sender_endpoint.address().to_string(), r_hwinfo });

                    // A directed probe can only ever get one reply
                    if (address.length()) {
                        break;
                    }
                }
            }
        }
        catch (std::exception& e) {
            spdlog::error("UDP problem: {}", e.what());
        }
    }

    return found;
}

MSDLL rfnm_api_failcode device::set_stream_format(enum stream_format format, size_t* bufsize, uint8_t* bytes_per_ele) {
    if (stream_format_locked) {
        if (bufsize) {
            *bufsize = RFNM_USB_RX_PACKET_ELEM_CNT * s->transport_status.rx_stream_format;
        }
        if (bytes_per_ele) {
            *bytes_per_ele = s->transport_status.rx_stream_format;
        }

        if (format == s->transport_status.rx_stream_format) {
            return RFNM_API_OK;
        }
        else {
            return RFNM_API_NOT_SUPPORTED;
        }
    }

    switch (format) {
    case STREAM_FORMAT_CS8:
    case STREAM_FORMAT_CS16:
    case STREAM_FORMAT_CF32:
        s->transport_status.rx_stream_format = format;
        s->transport_status.tx_stream_format = format;
        if (bufsize) {
            *bufsize = RFNM_USB_RX_PACKET_ELEM_CNT * format;
        }
        if (bytes_per_ele) {
            *bytes_per_ele = format;
        }
        break;
    default:
        if (bufsize) {
            *bufsize = 0;
        }
        if (bytes_per_ele) {
            *bytes_per_ele = 0;
        }
        return RFNM_API_NOT_SUPPORTED;
    }

    return RFNM_API_OK;
}

MSDLL rx_stream* device::rx_stream_create(uint8_t ch_ids) {
    // Honor device-reported channel availability: warn if a requested channel is marked
    // unavailable. The device remains the authoritative gate and will reject it on apply.
    for (uint32_t ch = 0; ch < MAX_RX_CHANNELS; ch++) {
        if ((ch_ids & (1u << ch)) && !is_rx_channel_available(ch)) {
            spdlog::warn("rx_stream_create: rx channel {} reports avail=0", ch);
        }
    }

    stream_format_locked = true;
    return new rx_stream(*this, ch_ids);
}

MSDLL rfnm_api_failcode device::rx_work_start() {
    rfnm_api_failcode ret = RFNM_API_OK;

    rx_stream_count++;

    // no need to start workers if they're already running
    if (rx_stream_count > 1) return ret;

    stream_format_locked = true;

    // allocate buffers if the user didn't allocate them themselves
    if (!rx_s.qbuf_cnt) {
        rx_buffers_allocated = true;
        size_t bufsize = RFNM_USB_RX_PACKET_ELEM_CNT * s->transport_status.rx_stream_format;

        for (size_t i = 0; i < MIN_RX_BUFCNT; i++) {
            rx_buf* rxbuf = new rx_buf();
            rxbuf->buf = new uint8_t[bufsize];
            rx_qbuf(rxbuf, true);
        }
    }

    // expected CC of UINT64_MAX is a special value meaning to accept whatever comes
    for (int adc_id = 0; adc_id < 4; adc_id++) {
        rx_s.usb_cc[adc_id] = UINT64_MAX;
    }

    for (int8_t i = 0; i < THREAD_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
        if (s->transport_status.transport == TRANSPORT_TCP) {
            thread_data[i].rx_active = (i == 1) ? 1 : 0;  // Only thread 1 does RX
        }
        else if (s->transport_status.transport == TRANSPORT_LOCAL) {
            thread_data[i].rx_active = ((i % 2) == 1) ? 1 : 0;
        }
        else {
            thread_data[i].rx_active = 1;
        }
        thread_data[i].cv.notify_all();
    }

    return ret;
}

MSDLL rfnm_api_failcode device::rx_work_stop() {
    rfnm_api_failcode ret = RFNM_API_OK;

    if (rx_stream_count > 0) rx_stream_count--;

    if (rx_stream_count == 0) {
        for (int8_t i = 0; i < THREAD_COUNT; i++) {
            std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
            thread_data[i].rx_active = 0;
        }
    }

    return ret;
}

MSDLL rfnm_api_failcode device::tx_work_start(enum tx_latency_policy policy) {
    rfnm_api_failcode ret = RFNM_API_OK;

    for (int8_t i = 0; i < THREAD_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
        if (s->transport_status.transport == TRANSPORT_TCP) {
            thread_data[i].tx_active = (i == 0) ? 1 : 0;  // Only thread 0 does TX
        }
        else if (s->transport_status.transport == TRANSPORT_LOCAL) {
            thread_data[i].tx_active = ((i % 2) == 0) ? 1 : 0;
        }
        else {
            // USB: a single TX thread, like TCP. 16 threads racing 4 endpoints send one
            // logical stream arbitrarily out of order - deeper than any device-side
            // resync horizon - punching holes in the stream. One thread on one endpoint
            // is spec-guaranteed in-order. (Throughput beyond one sync-transfer thread
            // comes later from async URBs on that one endpoint, not from more threads.)
            thread_data[i].tx_active = (i == 0) ? 1 : 0;
        }
        thread_data[i].cv.notify_all();
    }

    return ret;
}

MSDLL rfnm_api_failcode device::tx_work_stop() {
    rfnm_api_failcode ret = RFNM_API_OK;

    for (int8_t i = 0; i < THREAD_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
        thread_data[i].tx_active = 0;
    }

    return ret;
}

MSDLL rfnm_api_failcode device::rx_qbuf(struct rx_buf* buf, bool new_buffer) {
    {
        std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
        if (new_buffer) rx_s.qbuf_cnt++;
        rx_s.in.push(buf);
    }
    rx_s.cv.notify_one();
    return RFNM_API_OK;
}


MSDLL rfnm_api_failcode device::tx_qbuf(struct tx_buf* buf, uint32_t timeout_us) {
    // elem_cnt selects the packet size: 0 = full, else a multiple of 256 samples (the
    // base-buf granularity the device consumes)
    if (buf->elem_cnt && (buf->elem_cnt % (LA_TX_BASE_BUFSIZE_12 / 3) || buf->elem_cnt > RFNM_USB_TX_PACKET_ELEM_CNT)) {
        return RFNM_API_NOT_SUPPORTED;
    }

    //std::lock_guard<std::mutex> lockGuard1(tx_s.cc_mutex);
    std::lock_guard<std::mutex> lockGuard1(s_dev_status_mutex);

    if (tx_s.usb_cc - s->dev_status.usb_dac_last_dqbuf[0] > 2000) {
        //spdlog::info("RFNM_API_MIN_QBUF_QUEUE_FULL {} {}", tx_s.usb_cc, static_cast<unsigned long>(s->dev_status.usb_dac_last_dqbuf[0]));
        return RFNM_API_MIN_QBUF_QUEUE_FULL;
    }

    std::lock_guard<std::mutex> lockGuard2(tx_s.in_mutex);

    // bound the in-queue depth independently of the cc window: the cc window (2000)
    // is wider than a typical app buffer pool (1500), so a TX path slower than the
    // RX path could swallow the app's entire pool into this queue before the window
    // closed - RX then starves for buffers and a full-duplex relay deadlocks (the
    // TCP wedge: wire-limited TX, all 1499 buffers parked here). 256 packets is ~85 ms
    // of pipeline at full rate - never the throughput limiter, always pool-safe.
    if (tx_s.in.size() >= 256) {
        return RFNM_API_MIN_QBUF_QUEUE_FULL;
    }

    tx_s.qbuf_cnt++;
    tx_s.usb_cc++;

    buf->usb_cc = (uint32_t)tx_s.usb_cc;
    tx_s.in.push(buf);

    tx_s.cv.notify_one();

    return RFNM_API_OK;
}

MSDLL int device::single_ch_id_bitmap_to_adc_id(uint8_t ch_ids) {
    int ch_id = 0;
    while (ch_id < MAX_RX_CHANNELS) {
        if ((ch_ids & 0x1) == 1) {
            return s->rx.ch[ch_id].adc_id;
        }
        ch_id++;
        ch_ids = ch_ids >> 1;
    }
    return -1;
}

MSDLL void device::dqbuf_overwrite_cc(uint8_t adc_id, int acquire_lock) {
    if (acquire_lock) {
        rx_s.out_mutex.lock();
    }
    rx_s.in_mutex.lock();

    uint64_t old_cc = rx_s.usb_cc[adc_id];
    size_t queue_size = rx_s.out[adc_id].size();

    // use whatever buffer is at the top of the queue
    if (queue_size) {
        rx_s.usb_cc[adc_id] = rx_s.out[adc_id].top()->usb_cc;
    }
    else {
        rx_s.usb_cc[adc_id]++;
    }

    rx_s.in_mutex.unlock();
    if (acquire_lock) {
        rx_s.out_mutex.unlock();
    }

    if (rx_s.usb_cc[adc_id] > old_cc) {
        rx_s.usb_cc_dropped[adc_id] += (rx_s.usb_cc[adc_id] - old_cc);
    }



    // saturation produces thousands of drop events per second - rate-limit the log
    // (counters keep accumulating) and report drops as a share of all packets
    static std::atomic<int64_t> last_drop_log_ms{0};
    int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    int64_t prev_ms = last_drop_log_ms.load(std::memory_order_relaxed);
    if (now_ms - prev_ms >= 1000 && last_drop_log_ms.compare_exchange_strong(prev_ms, now_ms)) {
        uint64_t total_cc = rx_s.usb_cc_ok[adc_id] + rx_s.usb_cc_dropped[adc_id];
        spdlog::info("cc {} -> {} size {} adc {} remote {}; ok {} dropped {} ({:.4f}%)",
            old_cc, rx_s.usb_cc[adc_id], queue_size, adc_id, static_cast<uint64_t>(s->dev_status.usb_adc_last_qbuf[adc_id]),
            rx_s.usb_cc_ok[adc_id], rx_s.usb_cc_dropped[adc_id],
            total_cc > 0 ? (100.0 * rx_s.usb_cc_dropped[adc_id] / total_cc) : 0);
    }
}



MSDLL void device::get_rx_stream_stats(uint64_t &ok, uint64_t &dropped) {
    // unlocked reads: aligned 64-bit loads of monotonic counters, good enough for stats
    ok = 0;
    dropped = 0;
    for (int adc_id = 0; adc_id < 4; adc_id++) {
        ok += rx_s.usb_cc_ok[adc_id];
        dropped += rx_s.usb_cc_dropped[adc_id];
    }
}

MSDLL int device::dqbuf_is_cc_continuous(uint8_t adc_id, int acquire_lock) {
    struct rx_buf* buf;
    size_t queue_size;

    if (acquire_lock) {
        rx_s.out_mutex.lock();
    }

    queue_size = rx_s.out[adc_id].size();
    if (queue_size < 1) {
        if (acquire_lock) {
            rx_s.out_mutex.unlock();
        }
        return 0;
    }

    buf = rx_s.out[adc_id].top();

    // special case for first buffer of stream
    if (rx_s.usb_cc[adc_id] == UINT64_MAX) {
        int ret = 0;
        if (getenv("RFNM_DEBUG_RX")) {
            spdlog::info("CCINIT adc {} queue_size {}", adc_id, queue_size);
        }

        // wait for at least 10 buffers to come in case they are out-of-order
        if (queue_size >= 10) {
            rx_s.usb_cc[adc_id] = buf->usb_cc;
            //spdlog::info("initial cc {} adc {}", rx_s.usb_cc[adc_id], adc_id);
            ret = 1;
        }

        if (acquire_lock) {
            rx_s.out_mutex.unlock();
        }
        return ret;
    }

    int max_allowed_in_flight = RX_MAX_INFLIGHT_BUF_CNT;
    if (s->transport_status.transport == TRANSPORT_TCP) {
        max_allowed_in_flight = RX_MAX_INFLIGHT_BUF_CNT_ETH;
    }

    // the TCP/local stream carries the local ring's cc, which ticks independently of the usb
    // ring's cc (variable-size usb packets advance the usb counter faster) - compare like to like
    uint64_t dev_last_qbuf = s->dev_status.usb_adc_last_qbuf[adc_id];
    if (s->transport_status.transport == TRANSPORT_TCP || s->transport_status.transport == TRANSPORT_LOCAL) {
        dev_last_qbuf = s->dev_status.local_adc_last_qbuf[adc_id];
    }

    if (abs(((int64_t)dev_last_qbuf) - ((int64_t)rx_s.usb_cc[adc_id])) > max_allowed_in_flight) {
        spdlog::info("max allowed inflight exceeded, reset cc from {} to {}", rx_s.usb_cc[adc_id], static_cast<uint64_t>(dev_last_qbuf));
        rx_s.usb_cc[adc_id] = dev_last_qbuf;
    }

    //static int stale_high_cnt = 0;

    std::vector<uint64_t> discarded;
    while (queue_size > 0) {
        // discard anything OLDER than expected, INCLUDING expected-1: a duplicate of the
        // just-consumed cc (kernel ring wrap re-stamping a buffer held by a late in-flight
        // req) otherwise sits at the queue top forever - neither continuous nor discardable -
        // and wedges the stream behind it until the timeout mass-discard kills the session.
        // A stale-OLDER top is discardable even as the last queued buffer: it can never be
        // delivered, and the old queue_size > 1 guard left it parked at the top blocking
        // dqbuf entirely (the inflight reset jumps expected forward while old packets are
        // still in flight; the late arrival then starves the stream in a tight no-data
        // spin - the full-duplex USB collapse cycle). Far-NEWER tops keep the > 1 guard.
        if (buf->usb_cc < rx_s.usb_cc[adc_id] || (queue_size > 1 && buf->usb_cc > (rx_s.usb_cc[adc_id] + RX_RECOMB_BUF_LEN))) {

            /*if (discarded.empty() && buf->usb_cc > (rx_s.usb_cc[adc_id] + RX_RECOMB_BUF_LEN)) {
                stale_high_cnt++;
                // number of times we ran this cycle and dropped after our cc;
                // should happen during init only, so limit it
            }*/

            uint64_t usb_cc = buf->usb_cc;
            if (getenv("RFNM_DEBUG_RX")) {
                spdlog::info("DISCARD adc {} buf_cc {} expected_cc {} qsize {}", adc_id, usb_cc, rx_s.usb_cc[adc_id], queue_size);
            }
            std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
            rx_s.out[adc_id].pop();
            rx_s.in.push(buf);
            rx_s.cv.notify_one();

            // do not log stale for eth transport as it could be retransmitted packets
            if (s->transport_status.transport != TRANSPORT_TCP) {
                discarded.push_back(buf->usb_cc);
            }

            queue_size--;
            //spdlog::info("stale cc {} discarded from adc {}", usb_cc, adc_id);
            if (queue_size == 0) {
                break;	// drained the queue entirely; no top to inspect
            }
            buf = rx_s.out[adc_id].top();
        }
        else {
            break;
        }
    };

    if (queue_size == 0) {
        if (acquire_lock) {
            rx_s.out_mutex.unlock();
        }
        return 0;
    }



    if (!discarded.empty()) {
        // build comma‐separated list
        std::string list;
        list.reserve(discarded.size() * 6);
        for (size_t i = 0; i < discarded.size(); ++i) {
            if (i) list += ", ";
            list += std::to_string(discarded[i]);
        }
        int qout = rx_s.out[adc_id].size();
        int qin = rx_s.in.size();
        spdlog::info("stale adc {} cc {} qin {} qout {}: [{}]", adc_id, rx_s.usb_cc[adc_id], qin, qout, list);
    }

    if (acquire_lock) {
        rx_s.out_mutex.unlock();
    }

    if (rx_s.usb_cc[adc_id] == buf->usb_cc) {
        rx_s.usb_cc_ok[adc_id]++;
        return 1;
    }
    else {
        if (getenv("RFNM_DEBUG_RX") && buf->usb_cc != rx_s.usb_cc[adc_id]) {
        static int dbg_throttle;
        if (++dbg_throttle % 50 == 1) {
            spdlog::info("NOTCONT adc {} top_cc {} expected {} qsize {}", adc_id, buf->usb_cc, rx_s.usb_cc[adc_id], queue_size);
        }
    }
    // A hole at the queue head with a healthy backlog behind it means the expected
        // packet was lost in transit, not reordered (at most MAX_THREAD_COUNT reads are
        // in flight): step over it now. Gating this on RX_RECOMB_BUF_LEN hoarded ~200 ms
        // of stream behind a packet that would never arrive - under full-duplex USB,
        // where occasional single-packet losses are real, that became a stall/discard
        // cycle that collapsed the whole stream. usb_cc_dropped accounts for the holes.
        if (queue_size > 16) {
            dqbuf_overwrite_cc(adc_id, acquire_lock);
        }
        return 0;
    }
}

MSDLL rfnm_api_failcode device::rx_dqbuf(struct rx_buf** buf, uint8_t ch_ids, uint32_t timeout_us) {
    int is_single_ch, required_adc_id;

    if (rx_s.qbuf_cnt < MIN_RX_BUFCNT) {
        return RFNM_API_MIN_QBUF_CNT_NOT_SATIFIED;
    }

    switch (ch_ids) {
    case CH0:
    case CH1:
    case CH2:
    case CH3:
    case CH4:
    case CH5:
    case CH6:
    case CH7:
        is_single_ch = 1;
        break;
    case 0:
        is_single_ch = 0;
        ch_ids = 0xff;
        break;
    default:
        is_single_ch = 0;
        break;
    }

    if (is_single_ch) {
        required_adc_id = single_ch_id_bitmap_to_adc_id(ch_ids);
    }
    else {
        do {
            uint8_t mask = channel_flags[last_dqbuf_ch];
            required_adc_id = single_ch_id_bitmap_to_adc_id(ch_ids & mask);

            if (++last_dqbuf_ch == 8) {
                last_dqbuf_ch = 0;
            }
        } while (required_adc_id < 0);
    }

    if (!dqbuf_is_cc_continuous(required_adc_id, 1)) {
        if (!timeout_us) {
            return RFNM_API_DQBUF_NO_DATA;
        }

        {
            std::unique_lock lk(rx_s.out_mutex);
            rx_s.cv.wait_for(lk, std::chrono::microseconds(timeout_us),
                [this, required_adc_id] { return dqbuf_is_cc_continuous(required_adc_id, 0) ||
                rx_s.out[required_adc_id].size() > RX_RECOMB_BUF_LEN; }
            );
        }

        if (!dqbuf_is_cc_continuous(required_adc_id, 1)) {
            if (getenv("RFNM_DEBUG_RX")) {
                std::lock_guard<std::mutex> lockGuard(rx_s.out_mutex);
                uint64_t topcc = rx_s.out[required_adc_id].size() ? rx_s.out[required_adc_id].top()->usb_cc : 0;
                spdlog::info("STUCK adc {} expected_cc {} qsize {} top_cc {}", required_adc_id,
                    rx_s.usb_cc[required_adc_id], rx_s.out[required_adc_id].size(), topcc);
            }
            if (timeout_us >= 10000) {
                spdlog::info("cc timeout {} adc {}", rx_s.usb_cc[required_adc_id], required_adc_id);
            }

            return RFNM_API_DQBUF_NO_DATA;
        }
    }

    {
        std::lock_guard<std::mutex> lockGuard(rx_s.out_mutex);
        *buf = rx_s.out[required_adc_id].top();
        rx_s.out[required_adc_id].pop();
    }

    struct rx_buf* lb;
    lb = *buf;

    rx_s.usb_cc[required_adc_id]++;

    //if ((lb->usb_cc & 0xff) < 0x10) {
    //    std::lock_guard<std::mutex> lockGuard(rx_s.out_mutex);
    //    spdlog::info("cc {} {} {}", lb->usb_cc, lcc, rx_s.out.size());
    //}

    //spdlog::info("cc {} adc {}", lb->usb_cc, lb->adc_id);

    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::rx_flush(uint32_t timeout_us, uint8_t ch_ids) {
    std::this_thread::sleep_for(std::chrono::microseconds(timeout_us));

    for (int ch_id = 0; ch_id < MAX_RX_CHANNELS; ch_id++) {
        if (!(ch_ids & channel_flags[ch_id])) continue;

        int adc_id = s->rx.ch[ch_id].adc_id;

        std::lock_guard lock_out(rx_s.out_mutex);

        while (rx_s.out[adc_id].size()) {
            struct rx_buf* buf = rx_s.out[adc_id].top();
            rx_s.out[adc_id].pop();
            std::lock_guard lock_in(rx_s.in_mutex);
            rx_s.in.push(buf);
            rx_s.cv.notify_one();
        }

        rx_s.usb_cc[adc_id] = UINT64_MAX;
    }

    return RFNM_API_OK;
}

MSDLL enum rfnm_rf_path device::string_to_rf_path(std::string path) {
    std::transform(path.begin(), path.end(), path.begin(),
        [](unsigned char c) { return std::tolower(c); });

    if (!path.compare("embed") || !path.compare("emb") || !path.compare("embedded") || !path.compare("internal") || !path.compare("onboard")) {
        return RFNM_PATH_EMBED_ANT;
    }

    if (!path.compare("loop") || !path.compare("loopback")) {
        return RFNM_PATH_LOOPBACK;
    }

    if (path.find("sma") != std::string::npos) {
        path.replace(path.find("sma"), 3, "");
    }

    if (path.find("ant") != std::string::npos) {
        path.replace(path.find("ant"), 3, "");
    }

    if (path.find("-") != std::string::npos) {
        path.replace(path.find("-"), 1, "");
    }

    if (path.find("_") != std::string::npos) {
        path.replace(path.find("_"), 1, "");
    }

    if (path.find(" ") != std::string::npos) {
        path.replace(path.find(" "), 1, "");
    }

    if (path.length() != 1 || path.c_str()[0] < 'a' || path.c_str()[0] > 'h') {
        return RFNM_PATH_NULL;
    }

    return (enum rfnm_rf_path)(path.c_str()[0] - 'a');
}

MSDLL std::string device::rf_path_to_string(enum rfnm_rf_path path) {
    if (path == RFNM_PATH_NULL) {
        return "null";
    }
    else if (path == RFNM_PATH_EMBED_ANT) {
        return "embedded";
    }
    else if (path == RFNM_PATH_LOOPBACK) {
        return "loopback";
    }
    else {
        return std::string(1, 'A' + (int)(path));
    }
}

MSDLL rfnm_api_failcode device::tx_dqbuf(struct tx_buf** buf) {
    std::lock_guard<std::mutex> lockGuard(tx_s.out_mutex);

    if (tx_s.out.size()) {
        *buf = tx_s.out.front();
        tx_s.out.pop();
        return RFNM_API_OK;
    }
    else {
        return RFNM_API_DQBUF_NO_DATA;
    }
}

MSDLL rfnm_api_failcode device::control_transfer(enum rfnm_control_ep type, uint32_t size, uint8_t* buf, uint32_t timeout_ms) {
    uint32_t r = -1;
    if (s->transport_status.transport == TRANSPORT_USB) {
        switch (type) {
        case RFNM_GET_DEV_HWINFO:
        case RFNM_GET_TX_CH_LIST:
        case RFNM_GET_RX_CH_LIST:
        case RFNM_GET_SET_RESULT:
        case RFNM_GET_DEV_STATUS:
        case RFNM_GET_SM_RESET:
        case RFNM_GET_LOCAL_MEMINFO:
            r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                type, 0, (unsigned char*)buf, size, timeout_ms);
            break;
        case RFNM_SET_TX_CH_LIST:
        case RFNM_SET_RX_CH_LIST:
        case RFNM_SET_SAMP_RATE:
            r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_OUT) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                type, 0, (unsigned char*)buf, size, timeout_ms);
            break;
        }
        if (r < 0) {
            spdlog::error("libusb_control_transfer for req type {} failed with code {}", (int)type, r);
            return RFNM_API_USB_FAIL;
        }
        else {
            return RFNM_API_OK;
        }
    }
    if (s->transport_status.transport == TRANSPORT_LOCAL) {
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
        uint8_t ep_ctrl_buf[RFNM_SYSCTL_TRANSFER_SIZE];
        switch (type) {
        case RFNM_GET_DEV_HWINFO:
        case RFNM_GET_TX_CH_LIST:
        case RFNM_GET_RX_CH_LIST:
        case RFNM_GET_SET_RESULT:
        case RFNM_GET_DEV_STATUS:
        case RFNM_GET_SM_RESET:
        case RFNM_GET_LOCAL_MEMINFO:
            if (ioctl(rfnm_ctrl_ep_ioctl, RFNM_IOCTL_BASE + (0xff & type), &ep_ctrl_buf) < 0) {
                goto exit_error_local;
            }
            memcpy(buf, &ep_ctrl_buf[0], size);
            break;
        case RFNM_SET_TX_CH_LIST:
        case RFNM_SET_RX_CH_LIST:
        case RFNM_SET_SAMP_RATE:
            memcpy(&ep_ctrl_buf[0], buf, size);
            if (ioctl(rfnm_ctrl_ep_ioctl, RFNM_IOCTL_BASE + (0xff & type), &ep_ctrl_buf) < 0) {
                goto exit_error_local;
            }
            break;
        }
        return RFNM_API_OK;
    exit_error_local:
        spdlog::error("ioctl control transfer for req type {} failed with code {}", (int)type, r);
        return RFNM_API_USB_FAIL;
#endif
    }
    if (s->transport_status.transport == TRANSPORT_TCP) {
        // CHANGED: Use TCP instead of UDP for control transfers

        // Serialize the whole request/response transaction on the shared control socket.
        // Scoped to TCP only (USB/LOCAL don't share a stream socket) and held just for one
        // transaction, so apply()'s poll loop and the worker status poll interleave at
        // transaction granularity instead of corrupting each other's responses.
        std::lock_guard<std::mutex> ctrl_lock(rfnm_ctrl_socket_tcp_mutex);

        try {
            struct rfnm_tcp_ctrl_header header;
            header.cmd = type & 0xff;
            std::vector<uint8_t> message;

            switch (type) {
            case RFNM_SET_TX_CH_LIST:
            case RFNM_SET_RX_CH_LIST:
            case RFNM_SET_SAMP_RATE:
                /* SET commands: send header + data */
                header.size = size;

                // Log the control message being sent
                //spdlog::info("Sending TCP control SET command: cmd=0x{:02x}, size={}", header.cmd, header.size);

                // Send as a single message to ensure atomicity
                
                message.resize(sizeof(header) + size);
                memcpy(message.data(), &header, sizeof(header));
                memcpy(message.data() + sizeof(header), buf, size);

                asio::write(*rfnm_ctrl_socket_tcp,
                    asio::buffer(message.data(), message.size()));

                /* SET commands don't get responses in this protocol */
                return RFNM_API_OK;

            case RFNM_GET_DEV_HWINFO:
            case RFNM_GET_TX_CH_LIST:
            case RFNM_GET_RX_CH_LIST:
            case RFNM_GET_SET_RESULT:
            case RFNM_GET_DEV_STATUS:
            case RFNM_GET_SM_RESET:
            case RFNM_GET_LOCAL_MEMINFO:
                /* GET commands: send header only, receive header + data */
                header.size = 0;

                asio::write(*rfnm_ctrl_socket_tcp,
                    asio::buffer(&header, sizeof(header)));

                /* Read response header */
                struct rfnm_tcp_ctrl_header resp_header;
                asio::read(*rfnm_ctrl_socket_tcp,
                    asio::buffer(&resp_header, sizeof(resp_header)));

                if (resp_header.cmd != (type & 0xff)) {
                    spdlog::error("TCP control response mismatch: got {}, expected {}",
                        resp_header.cmd, type & 0xff);
                    return RFNM_API_USB_FAIL;
                }

                if (resp_header.size != size) {
                    spdlog::error("TCP control size mismatch: got {}, expected {}", static_cast<unsigned>(header.size, size));
                    return RFNM_API_USB_FAIL;
                }

                /* Read response data */
                asio::read(*rfnm_ctrl_socket_tcp, asio::buffer(buf, size));
                if (header.cmd != 0x06) {
                    //spdlog::info("TCP control GET response received: cmd=0x{:02x}, size={}", resp_header.cmd, resp_header.size);
                }
                

                return RFNM_API_OK;

            default:
                spdlog::error("Unknown control transfer type: {}", (int)type);
                return RFNM_API_NOT_SUPPORTED;
            }
        }
        catch (std::exception& e) {
            spdlog::error("TCP control transfer error: {}", e.what());
            return RFNM_API_USB_FAIL;
        }

    }

    return RFNM_API_USB_FAIL;
}

MSDLL rfnm_api_failcode device::get(enum req_type type) {
    int r;

    if (type & REQ_HWINFO) {
        struct rfnm_dev_hwinfo r_hwinfo;
        if (control_transfer(RFNM_GET_DEV_HWINFO, sizeof(struct rfnm_dev_hwinfo), (unsigned char*)&r_hwinfo, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        memcpy(&s->hwinfo, &r_hwinfo, sizeof(struct rfnm_dev_hwinfo));

        if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
            uint32_t pv = r_hwinfo.protocol_version;
            spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED detected protocol is v{} while your librfnm is for v{}", pv, RFNM_PROTOCOL_VERSION);
            return RFNM_API_SW_UPGRADE_REQUIRED;
        }
    }

    if (type & REQ_TX) {
        struct rfnm_dev_tx_ch_list r_chlist;

        if (control_transfer(RFNM_GET_TX_CH_LIST, sizeof(struct rfnm_dev_tx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        memcpy(&s->tx, &r_chlist, sizeof(struct rfnm_dev_tx_ch_list));
    }

    if (type & REQ_RX) {
        struct rfnm_dev_rx_ch_list r_chlist;

        if (control_transfer(RFNM_GET_RX_CH_LIST, sizeof(struct rfnm_dev_rx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        memcpy(&s->rx, &r_chlist, sizeof(struct rfnm_dev_rx_ch_list));
    }

    if (type & REQ_DEV_STATUS) {
        struct rfnm_dev_status dev_status;

        if (control_transfer(RFNM_GET_DEV_STATUS, sizeof(struct rfnm_dev_status), (unsigned char*)&dev_status, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        memcpy(&s->dev_status, &dev_status, sizeof(struct rfnm_dev_status));
    }

    return RFNM_API_OK;
}

MSDLL void device::reset_device_state() {
    cc_tx = 0;
    cc_rx = 0;
    stream_format_locked = false;
    tx_s.usb_cc = 0;
    tx_s.qbuf_cnt = 0;

    // Reset RX expected CC
    for (int adc_id = 0; adc_id < 4; adc_id++) {
        rx_s.usb_cc[adc_id] = UINT64_MAX;
        rx_s.usb_cc_dropped[adc_id] = 0;
        rx_s.usb_cc_ok[adc_id] = 0;
    }
}

MSDLL rfnm_api_failcode device::apply(uint16_t applies, bool confirm_execution, uint32_t timeout_us) {
    int r;
    uint8_t applies_ch_tx = applies & 0xff;
    uint8_t applies_ch_rx = (applies & 0xff00) >> 8;

    if (applies_ch_tx) {
        struct rfnm_dev_tx_ch_list r_chlist;
        memcpy(&r_chlist, &s->tx, sizeof(struct rfnm_dev_tx_ch_list));
        r_chlist.apply = applies_ch_tx;
        r_chlist.cc = ++cc_tx;

        if (control_transfer(RFNM_SET_TX_CH_LIST, sizeof(struct rfnm_dev_tx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
    }

    if (applies_ch_rx) {
        struct rfnm_dev_rx_ch_list r_chlist;
        memcpy(&r_chlist, &s->rx, sizeof(struct rfnm_dev_rx_ch_list));
        r_chlist.apply = applies_ch_rx;
        r_chlist.cc = ++cc_rx;

        if (control_transfer(RFNM_SET_RX_CH_LIST, sizeof(struct rfnm_dev_rx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
    }

    if (confirm_execution) {
        using std::chrono::high_resolution_clock;
        using std::chrono::duration_cast;
        using std::chrono::duration;
        using std::chrono::microseconds;

        auto tstart = high_resolution_clock::now();

        while (1) {
            struct rfnm_dev_get_set_result r_res;

            if (control_transfer(RFNM_GET_SET_RESULT, sizeof(struct rfnm_dev_get_set_result), (unsigned char*)&r_res, 50) != RFNM_API_OK) {
                return RFNM_API_USB_FAIL;
            }

            if (r_res.cc_rx == cc_rx && r_res.cc_tx == cc_tx) {
                for (int q = 0; q < MAX_TX_CHANNELS; q++) {
                    if ((channel_flags[q] & applies_ch_tx) && r_res.tx_ecodes[q]) {
                        return (rfnm_api_failcode)r_res.tx_ecodes[q];
                    }
                }
                for (int q = 0; q < MAX_RX_CHANNELS; q++) {
                    if ((channel_flags[q] & applies_ch_rx) && r_res.rx_ecodes[q]) {
                        return (rfnm_api_failcode)r_res.rx_ecodes[q];
                    }
                }
                return RFNM_API_OK;
            }

            auto tnow = high_resolution_clock::now();
            auto us_int = duration_cast<microseconds>(tnow - tstart);

            if (us_int.count() > timeout_us) {
                return RFNM_API_TIMEOUT;
            }

            // fast tunes finish in well under a millisecond, so poll tightly at first;
            // after a few ms back off while the device works through a slow path (e.g. a
            // reclock) so the control socket isn't hammered for seconds at 1ms intervals
            if (us_int.count() < 4000) {
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            } else {
                int64_t poll_ms = 1 + us_int.count() / 20000;
                std::this_thread::sleep_for(std::chrono::milliseconds(std::min<int64_t>(poll_ms, 50)));
            }
        }
    }

    return RFNM_API_OK;
}


MSDLL int16_t device::suggested_lpf_bw(uint64_t samp_rate, const struct rfnm_dev_hwinfo_clock* clock) {
    if (!samp_rate) {
        return 0;
    }

    // Effective ADC rate: trust the programmed chain state when it corresponds to this
    // rate; the state only updates when a stream (re)starts, so otherwise predict it
    // with the same ladder the driver uses (rfnm_lalib.c): deepest VSPA decimation whose
    // DCS (rate x 2^k) lands in the 100-200 MHz window, with the divide-by-2 bit as a
    // last resort for the bottom octave.
    uint64_t adc_rate = 0;
    if (clock && clock->dcs_clk) {
        uint64_t adc = clock->dcs_clk >> clock->rx_dcs_div;
        if ((adc >> clock->rx_decim_log2) == samp_rate) {
            adc_rate = adc;
        }
    }
    if (!adc_rate) {
        adc_rate = samp_rate;
        for (int k = 9; k >= 0; k--) {
            uint64_t dcs = samp_rate << k;
            if (dcs >= 100000000 && dcs <= 200000000) {
                adc_rate = (k == 9) ? (dcs >> 1) : dcs;
                break;
            }
        }
    }

    // Filter to the wanted band, not the ADC band: twice the sample rate keeps the band
    // in the filter's flat region while attenuating the rest ahead of the ADC (the
    // decimation stages have finite stopband and out-of-band energy eats ADC headroom).
    // Never wider than the ADC band - at 1x decimation the output IS the ADC band and
    // anything wider folds back into it - rounded down, and at least 2 MHz to stay
    // above the RFIC's own low-band filter floor.
    uint64_t bw = std::min(2 * samp_rate, adc_rate) / 1000000;
    return (int16_t)std::clamp<uint64_t>(bw, 2, 160);
}

MSDLL rfnm_api_failcode device::set_samp_rate(uint64_t freq, uint32_t timeout_us) {
    struct rfnm_dev_set_samp_rate r_sr;
    r_sr.freq = freq;
    r_sr.cc = ++cc_samp_rate;

    if (control_transfer(RFNM_SET_SAMP_RATE, sizeof(struct rfnm_dev_set_samp_rate), (unsigned char*)&r_sr, 50) != RFNM_API_OK) {
        return RFNM_API_USB_FAIL;
    }

    // Poll GET_SET_RESULT until the device reports this samp-rate command (cc match),
    // then return its status. Same cc correlation as apply()'s cc_tx/cc_rx + ecodes.
    auto tstart = std::chrono::high_resolution_clock::now();
    while (1) {
        struct rfnm_dev_get_set_result r_res;

        if (control_transfer(RFNM_GET_SET_RESULT, sizeof(struct rfnm_dev_get_set_result), (unsigned char*)&r_res, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }

        if (r_res.cc_samp_rate == cc_samp_rate) {
            // refresh the client-visible hwinfo with the device's value
            if (get(REQ_HWINFO)) {
                return RFNM_API_USB_FAIL;
            }
            return (rfnm_api_failcode)r_res.samp_rate_ecode;
        }

        auto tnow = std::chrono::high_resolution_clock::now();
        int64_t us_int = std::chrono::duration_cast<std::chrono::microseconds>(tnow - tstart).count();
        if (us_int > timeout_us) {
            return RFNM_API_TIMEOUT;
        }

        // same backoff as apply(): a samp-rate change triggers a multi-second reclock
        int64_t poll_ms = 1 + us_int / 20000;
        std::this_thread::sleep_for(std::chrono::milliseconds(std::min<int64_t>(poll_ms, 50)));
    }
}

MSDLL const struct rfnm_dev_hwinfo* device::get_hwinfo() {
    return &(s->hwinfo);
}

MSDLL const struct rfnm_dev_status* device::get_dev_status() {
    return &(s->dev_status);
}

MSDLL const struct transport_status* device::get_transport_status() {
    return &(s->transport_status);
}

MSDLL const struct rfnm_api_rx_ch* device::get_rx_channel(uint32_t channel) {
    if (channel < MAX_RX_CHANNELS) {
        return &(s->rx.ch[channel]);
    }
    else {
        return nullptr;
    }
}

MSDLL const struct rfnm_api_tx_ch* device::get_tx_channel(uint32_t channel) {
    if (channel < MAX_RX_CHANNELS) {
        return &(s->tx.ch[channel]);
    }
    else {
        return nullptr;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_status(uint32_t channel, enum rfnm_ch_enable enable,
    enum rfnm_ch_stream stream, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].enable = enable;
        s->rx.ch[channel].stream = stream;

        if (apply) {
            return device::apply(rx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

/*
MSDLL rfnm_api_failcode device::set_rx_channel_samp_freq_div(uint32_t channel, int16_t m, int16_t n, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].samp_freq_div_m = m;
        s->rx.ch[channel].samp_freq_div_n = n;

        if (apply) {
            return set(rx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}*/

MSDLL rfnm_api_failcode device::set_rx_channel_freq(uint32_t channel, int64_t freq, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].freq = freq;

        if (apply) {
            return device::apply(rx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_rfic_lpf_bw(uint32_t channel, int16_t bw, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].rfic_lpf_bw = bw;

        if (apply) {
            return device::apply(rx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_gain(uint32_t channel, int8_t gain, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].gain = gain;

        if (apply) {
            return device::apply(rx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_agc(uint32_t channel, enum rfnm_agc_type agc, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].agc = agc;

        if (apply) {
            return device::apply(rx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_fm_notch(uint32_t channel, enum rfnm_fm_notch fm_notch, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].fm_notch = fm_notch;

        if (apply) {
            return device::apply(rx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_bias_tee(uint32_t channel, enum rfnm_bias_tee bias_tee, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].bias_tee = bias_tee;

        if (apply) {
            return device::apply(rx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_path(uint32_t channel, enum rfnm_rf_path path, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].path = path;

        if (apply) {
            return device::apply(rx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_tx_channel_status(uint32_t channel, enum rfnm_ch_enable enable,
    enum rfnm_ch_stream stream, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].enable = enable;
        s->tx.ch[channel].stream = stream;

        if (apply) {
            return device::apply(tx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}
/*
MSDLL rfnm_api_failcode device::set_tx_channel_samp_freq_div(uint32_t channel, int16_t m, int16_t n, bool apply)  {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].samp_freq_div_n = n;
        s->tx.ch[channel].samp_freq_div_m = m;

        if (apply) {
            return set(tx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}*/

MSDLL rfnm_api_failcode device::set_tx_channel_freq(uint32_t channel, int64_t freq, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].freq = freq;

        if (apply) {
            return device::apply(tx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_tx_channel_rfic_lpf_bw(uint32_t channel, int16_t bw, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].rfic_lpf_bw = bw;

        if (apply) {
            return device::apply(tx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_tx_channel_power(uint32_t channel, int8_t power, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].power = power;

        if (apply) {
            return device::apply(tx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_tx_channel_bias_tee(uint32_t channel, enum rfnm_bias_tee bias_tee, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].bias_tee = bias_tee;

        if (apply) {
            return device::apply(tx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_tx_channel_path(uint32_t channel, enum rfnm_rf_path path, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].path = path;

        if (apply) {
            return device::apply(tx_channel_apply_flags[channel]);
        }
        else {
            return RFNM_API_OK;
        }
    }
    else {
        return RFNM_API_NOT_SUPPORTED;
    }
}


// Channel availability is authoritative from the device: rfnm_dgb_reg_{rx,tx}_ch() marks
// every registered channel avail=1, so the channel list reflects exactly which channels
// exist. Enumeration counts available channels directly.
MSDLL uint32_t device::get_rx_channel_count() {
    uint32_t count = 0;

    for (uint32_t i = 0; i < MAX_RX_CHANNELS; i++) {
        if (s->rx.ch[i].avail) {
            count++;
        }
    }

    return count;
}

MSDLL uint32_t device::get_tx_channel_count() {
    uint32_t count = 0;

    for (uint32_t i = 0; i < MAX_TX_CHANNELS; i++) {
        if (s->tx.ch[i].avail) {
            count++;
        }
    }

    return count;
}

MSDLL bool device::is_rx_channel_available(uint32_t channel) {
    return channel < MAX_RX_CHANNELS && s->rx.ch[channel].avail != 0;
}

MSDLL bool device::is_tx_channel_available(uint32_t channel) {
    return channel < MAX_TX_CHANNELS && s->tx.ch[channel].avail != 0;
}

#ifdef _WIN32
#include <winsock2.h>
#include <iphlpapi.h>
#pragma comment(lib, "Iphlpapi.lib")
#pragma comment(lib, "Ws2_32.lib")
#else
#include <ifaddrs.h>
#include <net/if.h>
#include <arpa/inet.h>
#endif

// Helper: Compute broadcast address from IP and netmask.
std::string rfnm::compute_broadcast_address(const std::string& ip_str, const std::string& mask_str)
{
    in_addr ip, mask, broadcast;
    if (inet_pton(AF_INET, ip_str.c_str(), &ip) != 1)
        throw std::runtime_error("Invalid IP address: " + ip_str);
    if (inet_pton(AF_INET, mask_str.c_str(), &mask) != 1)
        throw std::runtime_error("Invalid netmask: " + mask_str);

    // Compute broadcast = ip OR (~mask)
    broadcast.s_addr = ip.s_addr | ~(mask.s_addr);

    char buf[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &broadcast, buf, INET_ADDRSTRLEN) == nullptr)
        throw std::runtime_error("Failed to convert broadcast address to string");
    return std::string(buf);
}


static std::vector<std::string> rfnm::get_broadcast_addresses() {
    std::vector<std::string> broadcasts;
#ifdef _WIN32
    // Initialize Winsock (if not already done by your application).
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        spdlog::error("WSAStartup failed");
        return broadcasts;
    }

    // Allocate a buffer for adapter addresses.
    ULONG flags = GAA_FLAG_INCLUDE_PREFIX;
    ULONG family = AF_INET; // IPv4 only.
    ULONG bufferSize = 15000;
    std::vector<char> buffer(bufferSize);
    PIP_ADAPTER_ADDRESSES pAddresses = reinterpret_cast<PIP_ADAPTER_ADDRESSES>(buffer.data());

    DWORD ret = GetAdaptersAddresses(family, flags, nullptr, pAddresses, &bufferSize);
    if (ret == ERROR_BUFFER_OVERFLOW) {
        buffer.resize(bufferSize);
        pAddresses = reinterpret_cast<PIP_ADAPTER_ADDRESSES>(buffer.data());
        ret = GetAdaptersAddresses(family, flags, nullptr, pAddresses, &bufferSize);
    }
    if (ret != NO_ERROR) {
        spdlog::error("GetAdaptersAddresses failed: {}", ret);
        WSACleanup();
        return broadcasts;
    }

    // Iterate over the adapters.
    for (PIP_ADAPTER_ADDRESSES adapter = pAddresses; adapter; adapter = adapter->Next) {
        // Skip if not up or if it's a loopback.
        if (adapter->OperStatus != IfOperStatusUp)
            continue;
        if (adapter->IfType == IF_TYPE_SOFTWARE_LOOPBACK)
            continue;

        // Skip adapters with names containing "vEthernet".
        if (adapter->Description && wcsstr(adapter->Description, L"Virtual") != nullptr) {
            //spdlog::info("Skipping virtual adapter (Description): {}", adapter->Description);
            continue;
        }

        // Filter out non-physical interfaces based on type.
        // Allow Ethernet and Wireless (Wi-Fi). Add additional physical types if needed.
        if (adapter->IfType != IF_TYPE_ETHERNET_CSMACD &&
            adapter->IfType != IF_TYPE_IEEE80211) {
            spdlog::info("Skipping non-physical adapter: {}", adapter->AdapterName);
            continue;
        }

        // Iterate over unicast addresses.
        for (PIP_ADAPTER_UNICAST_ADDRESS ua = adapter->FirstUnicastAddress; ua; ua = ua->Next) {
            if (ua->Address.lpSockaddr->sa_family == AF_INET) {
                // Get the IP address.
                sockaddr_in* sa_in = reinterpret_cast<sockaddr_in*>(ua->Address.lpSockaddr);
                char ipBuf[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &(sa_in->sin_addr), ipBuf, INET_ADDRSTRLEN);
                std::string ipStr(ipBuf);

                // Get the netmask from the OnLinkPrefixLength.
                ULONG prefix = ua->OnLinkPrefixLength;
                uint32_t mask_int = 0xFFFFFFFF << (32 - prefix);
                in_addr mask_addr;
                mask_addr.s_addr = htonl(mask_int);
                char maskBuf[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &mask_addr, maskBuf, INET_ADDRSTRLEN);
                std::string maskStr(maskBuf);

                try {
                    //spdlog::info("Using adapter {}: IP = {}, Netmask = {}, Broadcast = {}", adapter->AdapterName, ipStr, maskStr, broadcast);
                    broadcasts.push_back(compute_broadcast_address(ipStr, maskStr));
                }
                catch (const std::exception& ex) {
                    spdlog::error("Error computing broadcast address: {}", ex.what());
                }
            }
        }
    }
    WSACleanup();
    return broadcasts;
#else
    // Linux / macOS implementation using getifaddrs.
    struct ifaddrs* ifaddr = nullptr, * ifa = nullptr;

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return broadcasts;
    }

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr)
            continue;
        if (ifa->ifa_addr->sa_family == AF_INET &&
            (ifa->ifa_flags & IFF_UP) &&
            !(ifa->ifa_flags & IFF_LOOPBACK))
        {
            // Filter out interfaces with names that suggest virtual or non-physical interfaces.
            // For example, skip if name starts with "vir", "vmnet", "docker", or contains "vEthernet".
            std::string ifname(ifa->ifa_name);
            if (ifname.find("vir") == 0 ||
                ifname.find("vmnet") == 0 ||
                ifname.find("docker") == 0 ||
                ifname.find("vEthernet") != std::string::npos ||
                ifname.find("lo") == 0) {
                spdlog::info("Skipping virtual/non-physical interface: {}", ifname);
                continue;
            }

            // Get the IP address.
            struct sockaddr_in* sa = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
            char ipBuf[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &sa->sin_addr, ipBuf, INET_ADDRSTRLEN);
            std::string ipStr(ipBuf);

            // Get netmask.
            struct sockaddr_in* nm = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_netmask);
            char maskBuf[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &nm->sin_addr, maskBuf, INET_ADDRSTRLEN);
            std::string maskStr(maskBuf);

            try {
                //spdlog::info("Using interface {}: IP = {}, Netmask = {}, Broadcast = {}", ifa->ifa_name, ipStr, maskStr, broadcast_ip);
                broadcasts.push_back(compute_broadcast_address(ipStr, maskStr));
            }
            catch (const std::exception& ex) {
                spdlog::error("Error computing broadcast address: {}", ex.what());
            }
        }
    }

    freeifaddrs(ifaddr);
    return broadcasts;
#endif
}

MSDLL const char* device::failcode_to_string(rfnm_api_failcode code) {
    switch (code) {
    case RFNM_API_OK:
        return "Success";
    case RFNM_API_PROBE_FAIL:
        return "Probe failed";
    case RFNM_API_TUNE_FAIL:
        return "Tune failed";
    case RFNM_API_GAIN_FAIL:
        return "Gain setting failed";
    case RFNM_API_TIMEOUT:
        return "Operation timed out";
    case RFNM_API_USB_FAIL:
        return "USB communication failed";
    case RFNM_API_DQBUF_OVERFLOW:
        return "Dequeue buffer overflow";
    case RFNM_API_NOT_SUPPORTED:
        return "Operation not supported";
    case RFNM_API_SW_UPGRADE_REQUIRED:
        return "Software upgrade required - protocol version mismatch";
    case RFNM_API_DQBUF_NO_DATA:
        return "No data available in dequeue buffer";
    case RFNM_API_MIN_QBUF_CNT_NOT_SATIFIED:
        return "Minimum queue buffer count not satisfied";
    case RFNM_API_MIN_QBUF_QUEUE_FULL:
        return "Minimum queue buffer is full";
    default:
        return "Unknown error code";
    }
}

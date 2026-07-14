// USB transport: open/recovery ladder, discovery, ep0 control transacts, and the two
// async URB engines with the RX dead-pipe watchdog. Worker passes are invoked from the
// shared pump (core/pump.cpp).

#include <cstring>
#include <thread>

#include <libusb-1.0/libusb.h>
#include <spdlog/spdlog.h>

#include "impl.h"

using namespace rfnm;

struct rfnm::_usb_handle {
    libusb_device_handle* primary{};
    libusb_device_handle* boost{};
    // Dedicated context + second handle to the SAME device for the sync control verbs.
    // On the shared default context a sync control transfer from a non-worker thread is
    // parked on libusb's event-waiters condvar behind the streaming workers' event loop,
    // and since libusb 1.0.24 that condvar is only signalled by libusb_unlock_events()
    // (end of an event-handler pass) - so every verb's latency rides the workers' poll
    // quanta and thread hand-offs instead of the wire. A private context has no competing
    // event handler: the calling thread polls its own fd and wakes on its own completion.
    libusb_context* ctrl_ctx{};
    libusb_device_handle* ctrl{};
};

static void rfnm_usb_tx_async_cb(struct libusb_transfer* t) {
    auto* slot_state = (std::atomic<int>*)t->user_data;
    slot_state->store((t->status == LIBUSB_TRANSFER_COMPLETED && t->actual_length == t->length) ? 2 : 3);
}

static void rfnm_usb_rx_async_cb(struct libusb_transfer* t) {
    auto* slot_state = (std::atomic<int>*)t->user_data;
    slot_state->store((t->status == LIBUSB_TRANSFER_COMPLETED) ? 2 : 3);
}

bool device::impl::usb_open(const std::string &address) {
    usb_handle = new _usb_handle;

    int dev_cnt = 0;
    int r;
    int open_attempt = 0;
    libusb_device** devs = NULL;

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
        open_attempt = 0;

    retry_open:
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(devs[d], &desc);
        if (r < 0) {
            spdlog::error("failed to get usb dev descr");
            goto next;
        }

        if (desc.idVendor != RFNM_USB_VID || desc.idProduct != RFNM_USB_PID) {
            goto next;
        }

        // RFNM_USB_BUS_ONLY=<n>: probe only devices on one usb bus. Multi-board bench
        // safety: open-path kill tests (and the reset-recovery below) must not be able
        // to touch another board even at the descriptor-read stage.
        {
            const char* bus_only = getenv("RFNM_USB_BUS_ONLY");
            if (bus_only && libusb_get_bus_number(devs[d]) != (uint8_t)atoi(bus_only)) {
                goto next;
            }
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
                goto ep0_retry;
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

        transport_status.theoretical_mbps = 3500;

        // The boost interface must belong to THE SAME BOARD as the primary: opening
        // by VID/PID alone grabs whichever board enumerates first, and on a multi-
        // board host that (a) reads another board's samples (framing garbage) and
        // (b) arms the victim board's boost endpoints, whose queued requests then eat
        // its shared IN-inflight budget until its RX shipping gates shut permanently
        // (the duplex wedge's terminal state). Match serials.
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

        spdlog::info("Max theoretical transport speed is {} Mbps", transport_status.theoretical_mbps);

        r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR),
            RFNM_B_REQUEST, RFNM_GET_SM_RESET, 0, NULL, 0, 500);
        if (r < 0) {
            spdlog::error("Couldn't reset state machine");
            goto ep0_retry;
        }
        reset_session();

        transport_status.transport = TRANSPORT_USB;

        {
            rfnm_api_failcode gr = get(REQ_ALL);
            if (gr == RFNM_API_USB_FAIL) {
                goto ep0_retry;
            }
            if (gr) {
                goto next;
            }
        }

        // Open the dedicated control-transfer handle (see _usb_handle): a second usbfs
        // open of the same device in a private context, matched by bus + address so a
        // multi-board host can't cross-wire boards. All control verbs are vendor
        // requests with recipient DEVICE, so no interface claim is needed (the data
        // interface claim on primary is exclusive and must not be duplicated). On any
        // failure fall back to primary - functionally identical, but control-verb
        // latency then couples to the streaming workers' event loop (8-40 ms SET_TXN
        // push stalls on gated/windowed sessions while the RX pipe is silent), so warn
        // instead of failing the open. This must stay AFTER the last goto ep0_retry /
        // goto next above: the primary is now fully accepted (serial matched, ep0
        // proven live), so the recovery paths - which reset the device and change its
        // address - can never run with a ctrl handle open, and the bus+address match
        // below can't go stale or leak across a retry_open pass.
#if LIBUSB_API_VERSION >= 0x0100010A
        r = libusb_init_context(&usb_handle->ctrl_ctx, nullptr, 0);
#else
        r = libusb_init(&usb_handle->ctrl_ctx);
#endif
        if (r == 0) {
            libusb_device* pdev = libusb_get_device(usb_handle->primary);
            uint8_t pbus = libusb_get_bus_number(pdev);
            uint8_t paddr = libusb_get_device_address(pdev);
            libusb_device** clist = nullptr;
            ssize_t ccnt = libusb_get_device_list(usb_handle->ctrl_ctx, &clist);
            for (ssize_t ci = 0; ci < ccnt; ci++) {
                if (libusb_get_bus_number(clist[ci]) == pbus && libusb_get_device_address(clist[ci]) == paddr) {
                    if (libusb_open(clist[ci], &usb_handle->ctrl)) {
                        usb_handle->ctrl = nullptr;
                    }
                    break;
                }
            }
            if (ccnt >= 0) {
                libusb_free_device_list(clist, 1);
            }
            if (!usb_handle->ctrl) {
                libusb_exit(usb_handle->ctrl_ctx);
                usb_handle->ctrl_ctx = nullptr;
            }
        }
        else {
            usb_handle->ctrl_ctx = nullptr;
        }
        if (!usb_handle->ctrl) {
            spdlog::warn("Couldn't open the dedicated control handle; control verbs will share the streaming event loop");
        }

        libusb_free_device_list(devs, 1);

        // Success
        return true;

    ep0_retry:
        // A host client killed mid-control-transfer can strand the gadget's ep0: every later open-path
        // control transfer on this device fails until a usb port reset. Reset the device currently being
        // probed (never any other) and retry the open sequence once; on a second failure give up as before.
        if (open_attempt++) {
            spdlog::error("Open-path control transfer failed again after usb reset, giving up on this device");
            goto next;
        }
        {
            uint8_t bus = libusb_get_bus_number(devs[d]);
            uint8_t ports[8] = { 0 };
            int port_depth = libusb_get_port_numbers(devs[d], ports, sizeof(ports));
            spdlog::info("Open-path control transfer failed, resetting usb device at bus {} port {} and retrying once", bus, port_depth > 0 ? ports[port_depth - 1] : 0);
            libusb_release_interface(usb_handle->primary, 0);
            r = libusb_reset_device(usb_handle->primary);
            libusb_close(usb_handle->primary);
            usb_handle->primary = NULL;
            if (r == 0) {
                goto retry_open;
            }
            if (r != LIBUSB_ERROR_NOT_FOUND && r != LIBUSB_ERROR_NO_DEVICE) {
                spdlog::error("usb reset failed, {}", libusb_strerror((libusb_error)r));
                goto next;
            }
            // The reset made the device re-enumerate, so the old libusb_device is stale. Re-scan for the
            // same physical device by bus + port path: the serial may have been unreadable pre-reset, so
            // identity is keyed on topology, not on the serial string.
            for (int scan = 0; scan < 10; scan++) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                libusb_free_device_list(devs, 1);
                devs = NULL;
                dev_cnt = libusb_get_device_list(NULL, &devs);
                for (int nd = 0; nd < dev_cnt; nd++) {
                    struct libusb_device_descriptor ndesc;
                    uint8_t cports[8] = { 0 };
                    if (libusb_get_device_descriptor(devs[nd], &ndesc)) {
                        continue;
                    }
                    if (ndesc.idVendor != RFNM_USB_VID || ndesc.idProduct != RFNM_USB_PID) {
                        continue;
                    }
                    if (libusb_get_bus_number(devs[nd]) != bus || libusb_get_port_numbers(devs[nd], cports, sizeof(cports)) != port_depth ||
                        memcmp(ports, cports, port_depth > 0 ? port_depth : 0)) {
                        continue;
                    }
                    d = nd;
                    goto retry_open;
                }
            }
            spdlog::error("usb device did not come back after reset");
            goto next;
        }

    next:
        if (usb_handle->ctrl) {
            libusb_close(usb_handle->ctrl);
            usb_handle->ctrl = nullptr;
        }
        if (usb_handle->ctrl_ctx) {
            libusb_exit(usb_handle->ctrl_ctx);
            usb_handle->ctrl_ctx = nullptr;
        }
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
    usb_handle = nullptr;
    return false;
}

void device::impl::usb_close() {
    if (usb_handle && usb_handle->ctrl) {
        libusb_close(usb_handle->ctrl);
    }
    if (usb_handle && usb_handle->ctrl_ctx) {
        libusb_exit(usb_handle->ctrl_ctx);
    }
    if (usb_handle && usb_handle->primary) {
        libusb_release_interface(usb_handle->primary, 0);
        libusb_close(usb_handle->primary);
    }
    if (usb_handle && usb_handle->boost) {
        libusb_release_interface(usb_handle->boost, 0);
        libusb_close(usb_handle->boost);
    }
    delete usb_handle;
    usb_handle = nullptr;
    libusb_exit(NULL);
}

void rfnm::usb_find_into(std::vector<struct dev_info> &found, const std::string &address) {
    int dev_cnt = 0;
    int r;
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

        {
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
        }

    next:
        libusb_release_interface(thandle, 0);
        libusb_close(thandle);
    }

exit_usb:
    libusb_free_device_list(devs, 1);
    libusb_exit(NULL);
}

rfnm_api_failcode device::impl::usb_ctrl_transfer(enum rfnm_control_ep type, uint32_t size, uint8_t* buf, uint32_t timeout_ms) {
    // int, not uint32_t: libusb returns negative error codes, and an unsigned compare
    // would read every failure (timeouts included) as success with a stale reply buffer
    int r = -1;
    // prefer the dedicated control handle/context (see _usb_handle): on the shared
    // default context these sync transfers park behind the streaming workers' event
    // loop and inherit its poll-quantum + hand-off latency
    libusb_device_handle* ctrl_handle = usb_handle->ctrl ? usb_handle->ctrl : usb_handle->primary;
    switch (type) {
    case RFNM_GET_DEV_HWINFO:
    case RFNM_GET_TX_CH_LIST:
    case RFNM_GET_RX_CH_LIST:
    case RFNM_GET_SET_RESULT:
    case RFNM_GET_DEV_STATUS:
    case RFNM_GET_SM_RESET:
    case RFNM_GET_LOCAL_MEMINFO:
        r = libusb_control_transfer(ctrl_handle, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
            type, 0, (unsigned char*)buf, size, timeout_ms);
        break;
    case RFNM_SET_TX_CH_LIST:
    case RFNM_SET_RX_CH_LIST:
    case RFNM_SET_SAMP_RATE:
    case RFNM_SET_TDD:
    case RFNM_SET_TXN:
        r = libusb_control_transfer(ctrl_handle, uint8_t(LIBUSB_ENDPOINT_OUT) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
            type, 0, (unsigned char*)buf, size, timeout_ms);
        break;
    default:
        // r stays -1: an unmapped verb fails loudly below instead of returning
        // OK with an untouched reply buffer
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

// TX stream stopped: pump completions and reap finished URB app buffers now - the
// last <=12 otherwise sit unreturned until the NEXT session's first submit, and a
// drain-then-close client never gets them back
void device::impl::usb_tx_drain_on_stop(worker_ctx &c) {
    auto &az = c.az;
    for (int pass = 0; pass < 10; pass++) {
        bool inflight = false;
        struct timeval tv_reap = {0, 10000};
        libusb_handle_events_timeout(nullptr, &tv_reap);
        for (int q = 0; q < RFNM_TX_ASYNC_URBS; q++) {
            if (az.state[q] == 1) {
                inflight = true;
            }
        }
        if (!inflight) {
            break;
        }
    }
    int reaped = 0;
    for (int q = 0; q < RFNM_TX_ASYNC_URBS; q++) {
        if (az.state[q] == 2 || az.state[q] == 3) {
            std::lock_guard<std::mutex> lockGuard(tx_s.out_mutex);
            tx_s.out.push(az.app[q]);
            az.state[q] = 0;
            reaped++;
        }
    }
    if (reaped) {
        tx_s.cv.notify_one();
    }
}

rx_pass_result device::impl::usb_rx_pass(worker_ctx &c) {
    auto &arz = c.arz;

    if (!usb_handle->primary) {
        spdlog::info("Thread {} detected force shutdown during USB op", c.index);
        return rx_pass_result::dead;
    }

    // async engine on the primary handle only (the second-cable boost ping-pong never
    // shipped, and the pump keeps strict per-endpoint ordering on primary)
    libusb_device_handle* lusb_handle = usb_handle->primary;
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
        c.rx_last_progress = std::chrono::high_resolution_clock::now();
    }
    int got_ep = -1;
    auto rx_tstart = std::chrono::high_resolution_clock::now();
    if (c.wd_generation != ses.rx_work_generation) {
        // new rx session: re-arm. The progress baseline must not carry a previous
        // session's age (a stale one fires a phantom resync on the first pass of a
        // restarted session), and the accounted ship-ccs adopt the device's CURRENT
        // counters - usb_adc_last_qbuf is monotonic across stream restarts (module
        // lifetime), so a pre-session value must never read as undelivered packets.
        std::lock_guard<std::mutex> lockGuard(s_dev_status_mutex);
        for (int adc_id = 0; adc_id < 4; adc_id++) {
            c.wd_qbuf_acked[adc_id] = dev_status.usb_adc_last_qbuf[adc_id];
        }
        c.wd_generation = ses.rx_work_generation;
        c.wd_dead_pending = false;
        c.wd_gap_counted = false;
        c.wd_resyncs_no_progress = 0;
        c.rx_last_progress = rx_tstart;
        c.wd_win_samples = 0;
        c.wd_rate_starved = false;
        c.wd_win_start = rx_tstart;
        c.wd_win_started = true;
    }
    if (!c.wd_win_started) {
        c.wd_win_start = rx_tstart;
        c.wd_win_started = true;
    }
    if (std::chrono::duration_cast<std::chrono::milliseconds>(rx_tstart - c.wd_win_start).count() >= 2000) {
        // the trickle-dead face of a dead pipe: a 0.5 Msps stream of partials reaps
        // URBs constantly, so a pure-silence gate never fires. Judge RATE instead:
        // samples delivered per 2 s window vs the set rate; under 2% (floor
        // 100 kS/s-equivalent) counts as dead for CONTINUOUS sessions (scheduled
        // sessions trickle by design), and only a healthy-rate window heals the
        // resync budget.
        uint64_t rate = ses.samp_rate_hz;
        double win_s = std::chrono::duration_cast<std::chrono::milliseconds>(rx_tstart - c.wd_win_start).count() / 1000.0;
        double floor_sps = rate ? std::max((double)rate * 0.02, 100000.0) : 0.0;
        bool starved_now = rate && !ses.rx_scheduled_session &&
                (double)c.wd_win_samples < floor_sps * win_s;
        if (starved_now && !c.wd_rate_starved) {
            spdlog::warn("RX rate-starved: {} samples in {:.1f} s vs {} S/s set ({}%% floor) - trickle-dead window",
                c.wd_win_samples, win_s, rate, 2);
        }
        c.wd_rate_starved = starved_now;
        if (!starved_now && c.wd_win_samples > 0) {
            // genuine-rate progress is the ONLY thing that heals the resync budget:
            // a trickle of URBs must not (that would keep the latch unreachable for
            // whole dead sessions)
            c.wd_resyncs_no_progress = 0;
        }
        c.wd_win_samples = 0;
        c.wd_win_start = rx_tstart;
    }
    // UDC stall recovery, host-driven: if the pipe is dead while URBs are pending,
    // the gadget's UDC has stopped mapping requests to TRBs (a dwc3 lost-event class
    // bug). SET_INTERFACE re-runs the gadget's set_alt (full endpoint re-arm +
    // request requeue device-side) AND resets the host xHCI endpoint state - the only
    // recovery that resyncs BOTH sides (device-only re-arm desyncs SS sequence state;
    // session re-open works for the same reason this does).
    //
    // "dead" is NOT just "silent": on a gated/scheduled session (rx_tdd_configure /
    // schedule_rx armed) the pipe is legitimately silent between windows, and a flat
    // silence timeout would fire a spurious resync every 2 s - each set_alt tearing
    // down the device's armed requests (-ESHUTDOWN) and destroying in-flight window
    // data. For those sessions the judge is the device-published ship position: the
    // module stamps every packet it stages for USB with a ship-time cc and publishes
    // it as dev_status.usb_adc_last_qbuf (worker-polled at the status pass on this
    // very thread, so it is fresh even while the bulk pipe is silent). During
    // host-side silence
    //   qbuf == accounted -> the device shipped nothing: silence is honest, wait
    //   qbuf >  accounted -> the device shipped packets we never received: dead
    // where accounted = max(newest cc received, session-arm/post-resync baseline).
    // A wedge can't hide from this: the device's stale-consumer stage gate still
    // ships up to its stale depth cap into a dead pipe, advancing qbuf. The mismatch
    // must persist for 500 ms before firing so a window landing right at the 2 s
    // deadline gets reaped, not resynced. Continuous sessions keep the flat 2 s
    // backstop unchanged (a resync there is cheap and possibly the only way out).
    bool rx_pipe_dead = false;
    if (!ses.rx_pipe_dead_latch.load(std::memory_order_relaxed) &&
            (c.wd_rate_starved ||
             std::chrono::duration_cast<std::chrono::milliseconds>(rx_tstart - c.rx_last_progress).count() > 2000)) {
        if (!ses.rx_scheduled_session) {
            rx_pipe_dead = true;
        } else {
            uint64_t qbuf_now[4];
            bool status_fresh;
            {
                std::lock_guard<std::mutex> lockGuard(s_dev_status_mutex);
                for (int adc_id = 0; adc_id < 4; adc_id++) {
                    qbuf_now[adc_id] = dev_status.usb_adc_last_qbuf[adc_id];
                }
                status_fresh = std::chrono::duration_cast<std::chrono::milliseconds>(rx_tstart - last_dev_time).count() < 500;
            }
            int dead_adc = -1;
            for (int adc_id = 0; adc_id < 4; adc_id++) {
                uint64_t accounted = rx_s.usb_cc_max_seen[adc_id] > c.wd_qbuf_acked[adc_id] ? rx_s.usb_cc_max_seen[adc_id] : c.wd_qbuf_acked[adc_id];
                if (qbuf_now[adc_id] > accounted) {
                    dead_adc = adc_id;
                }
            }
            if (!status_fresh) {
                // stale counters (control-path hiccup, no status refresh for 500 ms):
                // can't judge the pipe either way - defer to the next pass
            } else if (dead_adc >= 0) {
                if (!c.wd_dead_pending) {
                    c.wd_dead_pending = true;
                    c.wd_dead_since = rx_tstart;
                } else if (std::chrono::duration_cast<std::chrono::milliseconds>(rx_tstart - c.wd_dead_since).count() > 500) {
                    spdlog::warn("RX pipe dead in scheduled session: adc {} device shipped cc {} but host got {} (acked {})",
                        dead_adc, qbuf_now[dead_adc], rx_s.usb_cc_max_seen[dead_adc], c.wd_qbuf_acked[dead_adc]);
                    rx_pipe_dead = true;
                }
            } else {
                c.wd_dead_pending = false;
                if (!c.wd_gap_counted) {
                    // honest inter-window silence: suppressed exactly where a flat
                    // timeout would fire spuriously. Counted once per episode.
                    c.wd_gap_counted = true;
                    transport_status.rx_gap_suppressed_cnt++;
                    spdlog::debug("RX watchdog: scheduled-session gap >2s, device idle (qbuf {} {} {} {}), resync suppressed",
                        qbuf_now[0], qbuf_now[1], qbuf_now[2], qbuf_now[3]);
                }
            }
        }
    } else {
        c.wd_dead_pending = false;
        c.wd_gap_counted = false;
    }
    if (rx_pipe_dead && c.wd_resyncs_no_progress >= 2) {
        // the resync budget is spent with zero progress after it. SET_INTERFACE can
        // only re-arm the endpoints BELOW the stopped board-side producer - each
        // further one just ESHUTDOWN-storms the gadget back into the same frozen
        // state. Fail hard instead: latch, wake every dqbuf waiter, and go quiet. No
        // SM reset from in here by design - the client owns recovery (reopen).
        if (!ses.rx_pipe_dead_latch.exchange(true)) {
            spdlog::error("RX pipe still dead after {} SET_INTERFACE resyncs: board-side producer parked, failing hard (RX_PIPE_DEAD) - client must reopen",
                c.wd_resyncs_no_progress);
            rx_s.cv.notify_all();
        }
        rx_pipe_dead = false;
        c.wd_dead_pending = false;
    }
    if (rx_pipe_dead) {
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
        // write off everything staged up to now: the cancel/set_alt teardown just
        // discarded any in-flight packets, and their ccs must not read as a dead
        // pipe again on the next silent stretch
        {
            std::lock_guard<std::mutex> lockGuard(s_dev_status_mutex);
            for (int adc_id = 0; adc_id < 4; adc_id++) {
                c.wd_qbuf_acked[adc_id] = dev_status.usb_adc_last_qbuf[adc_id];
            }
        }
        transport_status.rx_dead_pipe_resync_cnt++;
        c.wd_resyncs_no_progress++;
        c.wd_dead_pending = false;
        c.rx_reap_rr = 0;
        c.rx_last_progress = std::chrono::high_resolution_clock::now();
        // fresh rate window: the resynced pipe gets a full 2 s to prove a real
        // rate before the next starved verdict (or the latch at budget 2)
        c.wd_win_samples = 0;
        c.wd_win_start = c.rx_last_progress;
        c.wd_rate_starved = false;
    }
    while (got_ep < 0) {
        // fair rotation across endpoints: preferential draining of one endpoint
        // exhausts the others' URB pools (all slots completed-but-unreaped), the
        // gadget's requests on those endpoints starve, and the device ring jumps
        // forward over the parked packets - giant cc holes and a wedged stream
        for (int i = 0; i < RFNM_RX_ASYNC_EPS; i++) {
            int e = (c.rx_reap_rr + i) % RFNM_RX_ASYNC_EPS;
            int q = arz.next_reap[e];
            if (arz.state[e][q] == 2) { got_ep = e; c.rx_reap_rr = (e + 1) % RFNM_RX_ASYNC_EPS; break; }
            if (arz.state[e][q] == 3) {
                // failed HEAD slot: consume the failure in order - resubmit and
                // advance. Resubmitting a non-head slot would file it behind its
                // siblings in the endpoint queue and permanently desync next_reap
                // from the endpoint's completion order (wedge: heads in flight,
                // completed slots parked unreapable behind them).
                if (arz.xfer[e][q]->status == LIBUSB_TRANSFER_NO_DEVICE) {
                    spdlog::info("Thread {} USB device lost, exiting", c.index);
                    return rx_pass_result::dead;
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
        return rx_pass_result::skip;
    }
    c.rx_last_progress = std::chrono::high_resolution_clock::now();
    // a reaped URB is liveness for the SILENCE gate only. The resync budget heals
    // exclusively on a healthy-RATE window above - a per-URB reset would make the
    // trickle-dead face unlatchable (partials reap constantly at 0.5 Msps).
    {
        int q = arz.next_reap[got_ep];
        c.transferred = arz.xfer[got_ep][q]->actual_length;
        memcpy(c.lrxbuf, arz.xbuf[got_ep][q], c.transferred);
        arz.state[got_ep][q] = libusb_submit_transfer(arz.xfer[got_ep][q]) ? 3 : 1;
        arz.next_reap[got_ep] = (q + 1) % RFNM_RX_ASYNC_URBS_PER_EP;
    }

    // variable-size packet: the device shortens the wire transfer to the valid
    // sample prefix and declares the count in the header (latency-deadline flush).
    // The transfer may also arrive PADDED past the declared payload (full-size
    // wire transfer carrying a partial packet) - accept any transfer that contains
    // at least the declared payload with a valid head; reject short/garbage ones.
    // Fillers are EXPECTED, not errors: ZLPs (the gadget's alt-set pre-queue and
    // its flushmode drain both send zero-length requests) and, from older device
    // firmware, full-size zero URBs (magic 0) - recycle those silently.
    if (c.lrxbuf->magic != 0x7ab8bd6f || c.lrxbuf->elem_cnt > RFNM_USB_RX_PACKET_ELEM_CNT ||
            (size_t)c.transferred < RFNM_USB_RX_PACKET_HEAD_SIZE + (size_t)c.lrxbuf->elem_cnt * 3) {
        if (c.transferred != 0 && !(c.lrxbuf->magic == 0 && c.lrxbuf->elem_cnt == 0)) {
            spdlog::error("thread loop RX usb wrong size, {}, ep {}, elem_cnt {}, magic {:x}", c.transferred,
                thread_data[c.index].ep_id, (uint32_t)c.lrxbuf->elem_cnt, (uint32_t)c.lrxbuf->magic);
        }
        return rx_pass_result::skip;
    }
    c.wd_win_samples += c.lrxbuf->elem_cnt;    // rate-window intake (validated packets only)
    return rx_pass_result::deliver;
}

tx_pass_result device::impl::usb_tx_pass(worker_ctx &c, struct tx_buf *buf, uint32_t tx_wire_len) {
    auto &az = c.az;

    if (!usb_handle->primary) {
        spdlog::info("Thread {} detected force shutdown during USB op", c.index);
        return tx_pass_result::dead;
    }
    libusb_device_handle* lusb_handle = usb_handle->primary;

    // async pipelined sends on ONE ordered endpoint (see the engine notes in impl.h)
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
        // immediately - bouncing through the dev-status path per packet gates
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
        return tx_pass_result::again;
    }
    struct rfnm_tx_usb_buf* axbuf = (struct rfnm_tx_usb_buf*)az.xbuf[slot];
    memcpy(axbuf, c.ltxbuf, RFNM_USB_TX_PACKET_HEAD_SIZE);
    memcpy(axbuf->buf, c.ltxbuf->buf, tx_wire_len - RFNM_USB_TX_PACKET_HEAD_SIZE);
    az.app[slot] = buf;
    az.state[slot] = 1;
    libusb_fill_bulk_transfer(az.xfer[slot], lusb_handle, (1 | LIBUSB_ENDPOINT_OUT),
        (uint8_t*)axbuf, tx_wire_len, rfnm_usb_tx_async_cb, &az.state[slot], 1000);
    int r = libusb_submit_transfer(az.xfer[slot]);
    if (r) {
        spdlog::error("TX async submit fail {}", r);
        az.state[slot] = 0;
        std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
        tx_s.in.push(buf);
        reorder_tx_queue_nolock(tx_s);
        return tx_pass_result::done;
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
                return tx_pass_result::again;
            }
        }
    }
    return tx_pass_result::done;
}

void device::impl::usb_engine_teardown(worker_ctx &c) {
    auto &az = c.az;
    auto &arz = c.arz;

    // cancel-and-drain this worker's URB engine before the thread returns: a URB
    // completing (even as CANCELLED) after the thread state is gone is a
    // use-after-free. Bounded; a vanished device resolves cancels in one event pass
    // with LIBUSB_TRANSFER_NO_DEVICE.
    for (int q = 0; q < RFNM_TX_ASYNC_URBS; q++) {
        if (az.initialized && az.state[q] == 1 && az.xfer[q]) {
            libusb_cancel_transfer(az.xfer[q]);
        }
    }
    for (int e = 0; e < RFNM_RX_ASYNC_EPS; e++) {
        for (int q = 0; q < RFNM_RX_ASYNC_URBS_PER_EP; q++) {
            if (arz.initialized && arz.state[e][q] == 1 && arz.xfer[e][q]) {
                libusb_cancel_transfer(arz.xfer[e][q]);
            }
        }
    }
    bool inflight = true;
    for (int pass = 0; pass < 50 && inflight; pass++) {
        struct timeval tv_drain = {0, 10000};
        libusb_handle_events_timeout(nullptr, &tv_drain);
        inflight = false;
        for (int q = 0; q < RFNM_TX_ASYNC_URBS; q++) {
            if (az.initialized && az.state[q] == 1) {
                inflight = true;
            }
        }
        for (int e = 0; e < RFNM_RX_ASYNC_EPS; e++) {
            for (int q = 0; q < RFNM_RX_ASYNC_URBS_PER_EP; q++) {
                if (arz.initialized && arz.state[e][q] == 1) {
                    inflight = true;
                }
            }
        }
    }
    if (az.initialized) {
        // hand the last TX app buffers back so the client's accounting closes
        int reaped = 0;
        for (int q = 0; q < RFNM_TX_ASYNC_URBS; q++) {
            if (az.state[q] == 2 || az.state[q] == 3) {
                std::lock_guard<std::mutex> lockGuard(tx_s.out_mutex);
                tx_s.out.push(az.app[q]);
                az.state[q] = 0;
                reaped++;
            }
        }
        if (reaped) {
            tx_s.cv.notify_one();
        }
    }
    if (inflight) {
        spdlog::warn("USB teardown: URBs still in flight after the drain window - leaking engine buffers deliberately (no completion into freed memory)");
    } else {
        for (int q = 0; q < RFNM_TX_ASYNC_URBS; q++) {
            if (az.xfer[q]) {
                libusb_free_transfer(az.xfer[q]);
                az.xfer[q] = nullptr;
            }
            free(az.xbuf[q]);
            az.xbuf[q] = nullptr;
        }
        for (int e = 0; e < RFNM_RX_ASYNC_EPS; e++) {
            for (int q = 0; q < RFNM_RX_ASYNC_URBS_PER_EP; q++) {
                if (arz.xfer[e][q]) {
                    libusb_free_transfer(arz.xfer[e][q]);
                    arz.xfer[e][q] = nullptr;
                }
                free(arz.xbuf[e][q]);
                arz.xbuf[e][q] = nullptr;
            }
        }
        az.initialized = 0;
        arz.initialized = 0;
    }
}

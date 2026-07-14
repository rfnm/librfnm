// The shared worker pump: two workers per device (0 = TX + status-poll cadence,
// 1 = RX + status backstop) driving the per-transport passes in core/{usb,tcp,local}.cpp
// plus the shared RX deliver tail and the dev-status poll.

#include <algorithm>
#include <cstring>
#include <future>

#include <spdlog/spdlog.h>

#ifdef BUILD_RFNM_LOCAL_TRANSPORT
#include <fcntl.h>
#include <unistd.h>
#endif

#include "impl.h"
#include "pack.h"

using namespace rfnm;

// The RX worker's buffer-starvation nap (see impl.h for the pairing rule).
std::mutex rfnm::rfnm_rx_in_nap_mutex;
std::condition_variable rfnm::rfnm_rx_in_nap_cv;

void device::impl::reorder_tx_queue_nolock(tx_queue_s& q) {
    std::vector<tx_buf*> temp;

    while (!q.in.empty()) {
        temp.push_back(q.in.front());
        q.in.pop();
    }

    std::sort(temp.begin(), temp.end(), [](const tx_buf* a, const tx_buf* b) {
        return a->usb_cc < b->usb_cc;
        });

    for (tx_buf* buf : temp) {
        q.in.push(buf);
    }
}

// sampled TX payload peak scan (1-in-16 packets): the 12-bit wire drops the low 4
// bits, so a peak far below int16 full scale means few effective bits on air - and a
// peak AT full scale means the producer already clipped upstream. Worker-thread-only
// writer; readers tolerate torn telemetry.
void device::impl::tx_headroom_scan(const uint8_t* buf, size_t elems) {
    if ((tx_headroom_ctr++ & 15) != 0) {
        return;
    }
    const int16_t* d = (const int16_t*)buf;
    int32_t peak = tx_peak_abs;
    for (size_t j = 0; j < elems * 2; j++) {
        int32_t v = d[j];
        if (v < 0) v = -v;
        if (v > peak) peak = v;
    }
    tx_peak_abs = peak;
    tx_headroom_scanned += elems;
    if (!tx_headroom_warned_sat && peak >= 32760) {
        tx_headroom_warned_sat = true;
        spdlog::warn("TX payload at int16 full scale (peak {}): the producer's scaling already "
                "saturated BEFORE the 12-bit wire - distortion is upstream, not the radio", peak);
    }
    if (!tx_headroom_warned_low && tx_headroom_scanned > 2000000 && peak > 0 && peak < 4096) {
        tx_headroom_warned_low = true;
        int bits = 0;
        for (int32_t p = peak; p; p >>= 1) bits++;
        spdlog::warn("TX peak only {}/32767 after {} samples (~{} effective wire bits): the 12-bit "
                "wire keeps bits [15:4] - raise digital amplitude toward full scale",
                peak, tx_headroom_scanned, bits > 4 ? bits - 4 : 0);
    }
}

// The shared RX deliver tail: validate the staged packet, convert to the session's
// stream format, publish into the ordered out queue. false = rejected (the caller
// recycles the app buffer); the local pool slot is returned on both outcomes.
bool device::impl::rx_deliver(worker_ctx &c, struct rx_buf *buf) {
    struct rfnm_rx_usb_buf *lrxbuf = c.lrxbuf;

    if (lrxbuf->magic != 0x7ab8bd6f || lrxbuf->adc_id > 3 ||
            lrxbuf->elem_cnt == 0 || lrxbuf->elem_cnt > RFNM_USB_RX_PACKET_ELEM_CNT) {
        if (getenv("RFNM_DEBUG_RX")) {
            static std::atomic<int> dbg_rej{0};
            int nr = ++dbg_rej;
            if (nr <= 10 || nr % 500 == 0) {
                spdlog::info("REJECT n {} magic {:x} adc {} elem_cnt {} transferred {}", nr,
                    (uint32_t)lrxbuf->magic, (uint32_t)lrxbuf->adc_id, (uint32_t)lrxbuf->elem_cnt, c.transferred);
            }
        }
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
        local_rx_put(c);
#endif
        return false;
    }

    if (lrxbuf->fmt == RFNM_PACKET_FMT_CS16) {
        // local transport: the kernel already unpacked 12->16, only the output
        // format conversion remains (single unpack total on this SoC)
        if (transport_status.rx_stream_format == STREAM_FORMAT_CS8) {
            convert_cs16_to_cs8(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
        }
        else if (transport_status.rx_stream_format == STREAM_FORMAT_CS16) {
            convert_cs16_to_cs16(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
        }
        else if (transport_status.rx_stream_format == STREAM_FORMAT_CF32) {
            convert_cs16_to_cf32(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
        }
    }
    else if (transport_status.rx_stream_format == STREAM_FORMAT_CS8) {
        unpack_12_to_cs8(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
    }
    else if (transport_status.rx_stream_format == STREAM_FORMAT_CS16) {
        unpack_12_to_cs16(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
    }
    else if (transport_status.rx_stream_format == STREAM_FORMAT_CF32) {
        unpack_12_to_cf32(buf->buf, (uint8_t*)lrxbuf->buf, lrxbuf->elem_cnt);
    }

    buf->rx_flags = lrxbuf->rx_flags;
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
        if (buf->usb_cc > rx_s.usb_cc_max_seen[lrxbuf->adc_id]) {
            rx_s.usb_cc_max_seen[lrxbuf->adc_id] = buf->usb_cc;
        }

        rx_s.cv.notify_one();
    }

#ifdef BUILD_RFNM_LOCAL_TRANSPORT
    local_rx_put(c);
#endif

    return true;
}

void device::impl::status_poll_pass(worker_ctx &c) {
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;

    auto tlast = last_dev_time;
    auto tnow = high_resolution_clock::now();
    auto ms_int = duration_cast<milliseconds>(tnow - tlast);

    // TX closed-loop pacing rides on dev_status freshness; with small TX packets a
    // 5 ms status lag forces deep pipelines (or underruns), so poll at >=1 ms.
    if (ms_int.count() > 1) {

        // Poll-cadence gates. (a) The TX-submit worker (0) never blocks on a status
        // RTT while TX work is queued - worker 1 keeps the 1 ms freshness during RX
        // sessions; (b) the blocking control transfer runs WITHOUT s_dev_status_mutex
        // (tx_qbuf takes that mutex to stamp every packet - holding it across a ctrl
        // RTT at 1 kHz stalls stamping and pickup by up to the RTT; the mutex guards
        // only the memcpy). Rare duplicate polls (both workers inside the 1 ms window)
        // serialize in libusb and just refresh twice - harmless.
        // Sparse-burst lead: "while TX work is queued" is the wrong gate for a burst
        // feeder (NR-UE shape) - its bursts always arrive while the queue is EMPTY,
        // so worker 0 would sit inside a blocking status RTT and the burst waits it
        // out before submit, spending up to the RTT out of a ~1.5 ms write lead. On
        // USB: when worker 1 runs an RX session it already keeps the 1 ms freshness,
        // so worker 0 skips the poll outright; solo TX-only sessions drop to a 3 ms
        // backstop cadence (pacing feedback lag ~6 packets vs the 2000-packet cc
        // window). RFNM_STATUS_POLL_LEGACY=1 restores the queued-work-only gate.
        bool tx_work_pending = false;
        static const bool poll_legacy = getenv("RFNM_STATUS_POLL_LEGACY") != nullptr;
        if (c.index == 0) {
            {
                std::lock_guard<std::mutex> lg(tx_s.in_mutex);
                tx_work_pending = !tx_s.in.empty();
            }
            if (!tx_work_pending && !poll_legacy && thread_data[c.index].tx_active &&
                    transport_status.transport == TRANSPORT_USB) {
                if (thread_data[1].rx_active) {
                    tx_work_pending = true;    // worker 1 owns freshness
                } else if (ms_int.count() <= 3) {
                    tx_work_pending = true;    // solo backstop cadence
                }
            }
        }
        // over TCP the ctrl RTT (wire + sched, ~100-300 us vs USB's 56 us) rides the
        // SAME thread as the data recv - a 1 kHz blocking poll on worker 1 stalls one
        // data packet per RTT per millisecond of a full-rate stream. Worker 0 owns
        // the 1 ms cadence for TCP (its gate above only skips while TX work is
        // queued); worker 1 backstops at 3 ms staleness so RX-only sessions and
        // saturated-TX shapes (worker 0 permanently queued) never go stale.
        if (c.index == 1 && !poll_legacy &&
                transport_status.transport == TRANSPORT_TCP &&
                ms_int.count() <= 3) {
            tx_work_pending = true;    // reuse the skip flag
        }
        if (!tx_work_pending)
        {
            struct rfnm_dev_status new_status[1];

            if (ctrl_transfer(RFNM_GET_DEV_STATUS, sizeof(struct rfnm_dev_status), (unsigned char*)&new_status[0], 50) != RFNM_API_OK) {
                // Budgeted, unified across transports: 4 consecutive failures AND
                // >1 s without a good status = the board is gone - say exactly that
                // once, latch the RX dead-pipe verdict so blocked dqbuf callers fail
                // with RX_PIPE_DEAD instead of timing out forever, and stop.
                c.status_fail_streak++;
                if (c.status_fail_streak >= 4 && ms_int.count() > 1000) {
                    spdlog::error("device status unreachable ({} consecutive failures, {} ms since last good status) - declaring the {} link dead, stopping the session (clients see RX_PIPE_DEAD)",
                        c.status_fail_streak, (long)ms_int.count(),
                        transport_status.transport == TRANSPORT_TCP ? "TCP" :
                        (transport_status.transport == TRANSPORT_USB ? "USB" : "LOCAL"));
                    ses.rx_pipe_dead_latch = true;
                    rx_s.cv.notify_all();
                    for (size_t i = 0; i < RFNM_WORKER_COUNT; i++) {
                        thread_data[i].rx_active = 0;
                        thread_data[i].tx_active = 0;
                        thread_data[i].shutdown_req = 1;
                    }
                } else {
                    spdlog::error("control_transfer for RFNM_GET_DEV_STATUS failed (streak {})", c.status_fail_streak);
                }
            }
            else {
                c.status_fail_streak = 0;
                std::lock_guard<std::mutex> lockGuard(s_dev_status_mutex);
                memcpy(&dev_status, &new_status[0], sizeof(struct rfnm_dev_status));
                last_dev_time = high_resolution_clock::now();
                // device-time cache: phytimer_now paired with the host clock
                ptmr_at_status = dev_status.phytimer_now;
                host_at_status = std::chrono::steady_clock::now();
                ptmr_at_status_valid = (ptmr_at_status != 0);
            }
        }
    }
}

void device::impl::worker(size_t index) {
    worker_ctx c;
    c.index = index;
    // USB/TCP stage packets here; the local transport points lrxbuf/ltxbuf into the
    // mmap'd kernel pools instead, so the frees below must use the heap handles
    c.lrxbuf_heap = new uint8_t[RFNM_LOCAL_RX_PACKET_SIZE]();
    c.ltxbuf_heap = new uint8_t[RFNM_LOCAL_TX_PACKET_SIZE]();
    c.lrxbuf = (struct rfnm_rx_usb_buf*)c.lrxbuf_heap;
    c.ltxbuf = (struct rfnm_tx_usb_buf*)c.ltxbuf_heap;
    auto& tpm = thread_data[index];

#ifdef BUILD_RFNM_LOCAL_TRANSPORT
    if (transport_status.transport == TRANSPORT_LOCAL) {
        c.data_ep_fp = open("/dev/rfnm_data_ep", O_RDWR);
        if (c.data_ep_fp < 0) {
            return;
        }
    }
#endif

    while (!tpm.shutdown_req) {
        if (!tpm.rx_active && !tpm.tx_active) {
            {
                std::unique_lock lk(tpm.cv_mutex);
                // spurious wakeups are acceptable
                tpm.cv.wait(lk,
                    [this, index] { return thread_data[index].rx_active ||
                    thread_data[index].tx_active ||
                    thread_data[index].shutdown_req; });
            }
        }

        if (transport_status.transport == TRANSPORT_USB && c.prev_tx_active && !tpm.tx_active && c.az.initialized) {
            usb_tx_drain_on_stop(c);
        }
        c.prev_tx_active = tpm.tx_active;

        if (tpm.rx_active) {
            // RX = worker 1 on every transport: USB and TCP run single-reaper async
            // engines / a single data socket, and the local pool ioctls need no
            // second reader either
            if (index != 1) {
                goto skip_rx;
            }

            {
                struct rx_buf* buf;

                if (getenv("RFNM_DEBUG_STALL") && index == 1) {
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

                {
                    std::unique_lock lk(rx_s.in_mutex);
                    if (rx_s.in.empty()) {
                        lk.unlock();
                        {
                            std::unique_lock nap(rfnm_rx_in_nap_mutex);
                            rfnm_rx_in_nap_cv.wait_for(nap, std::chrono::microseconds(1000));
                        }
                        lk.lock();
                        if (rx_s.in.empty()) {
                            goto skip_rx;
                        }
                    }

                    buf = rx_s.in.front();
                    rx_s.in.pop();
                }

                rx_pass_result res;
                if (transport_status.transport == TRANSPORT_USB) {
                    res = usb_rx_pass(c);
                }
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
                else if (transport_status.transport == TRANSPORT_LOCAL) {
                    res = local_rx_fetch(c);
                }
#endif
                else {
                    res = tcp_rx_pass(c);
                }

                if (res == rx_pass_result::dead) {
                    break;
                }
                if (res == rx_pass_result::skip || !rx_deliver(c, buf)) {
                    std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                    rx_s.in.push(buf);
                    rfnm_rx_in_nap_cv.notify_all();
                    goto skip_rx;
                }
            }
        }

    skip_rx:

        if (tpm.tx_active) {
            // TX = worker 0 on every transport (one ordered submitter)
            if (index != 0) {
                goto read_dev_status;
            }

            {
                struct tx_buf* buf;

                {
                    std::unique_lock lk(tx_s.in_mutex);
                    if (tx_s.in.empty()) {
                        tx_s.cv.wait_for(lk, std::chrono::microseconds(1000));
                        if (tx_s.in.empty()) {
                            goto read_dev_status;
                        }
                    }
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
                tx_headroom_scan(buf->buf, tx_elems);  // sampled meter, same payload every transport
                uint32_t tx_wire_len = RFNM_USB_TX_PACKET_HEAD_SIZE + tx_multi * LA_TX_BASE_BUFSIZE_12;
                if (transport_status.transport == TRANSPORT_LOCAL) {
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
                    if (!local_tx_prepare(c, buf, tx_elems, tx_wire_len)) {
                        goto read_dev_status;   // buf requeued inside
                    }
#endif
                } else {
                    pack_cs16_to_12((uint8_t*)c.ltxbuf->buf, buf->buf, tx_elems);
                    c.ltxbuf->fmt = RFNM_PACKET_FMT_PACKED12;
                }
                c.ltxbuf->tx_flags = buf->tx_flags;
                c.ltxbuf->dac_id = buf->dac_id;
                c.ltxbuf->usb_cc = buf->usb_cc;
                c.ltxbuf->phytimer = buf->phytimer;
                c.ltxbuf->multi = tx_multi;
                c.ltxbuf->magic = 0x758f4d4a;

                if (transport_status.transport == TRANSPORT_USB) {
                    tx_pass_result r = usb_tx_pass(c, buf, tx_wire_len);
                    if (r == tx_pass_result::dead) {
                        break;
                    }
                    if (r == tx_pass_result::again) {
                        continue;
                    }
                    goto read_dev_status;   // buf is owned by the URB engine (or requeued on failure)
                }

                if (transport_status.transport == TRANSPORT_LOCAL) {
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
                    local_tx_send(c);
#endif
                }

                if (transport_status.transport == TRANSPORT_TCP) {
                    if (!tcp_tx_send(c, tx_wire_len)) {
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
        }

    read_dev_status:

        status_poll_pass(c);
    }

    if (transport_status.transport == TRANSPORT_USB && (c.az.initialized || c.arz.initialized)) {
        usb_engine_teardown(c);
    }

    delete[] c.lrxbuf_heap;
    delete[] c.ltxbuf_heap;

    // TCP data socket teardown happens in the destructor after all threads have
    // joined, since the connection is shared by the RX and TX worker threads

#ifdef BUILD_RFNM_LOCAL_TRANSPORT
    if (transport_status.transport == TRANSPORT_LOCAL) {
        // a held RX slot leaks back to the kernel only at last-close reclaim; return
        // it here so an aborted pass doesn't pin a pool packet for the session
        local_rx_put(c);
        close(c.data_ep_fp);
    }
#endif
}

void device::impl::start_workers() {
    for (size_t i = 0; i < RFNM_WORKER_COUNT; i++) {
        thread_data[i].ep_id = i + 1;
        thread_data[i].rx_active = 0;
        thread_data[i].tx_active = 0;
        thread_data[i].shutdown_req = 0;
    }

    for (size_t i = 0; i < RFNM_WORKER_COUNT; i++) {
        thread_c[i] = std::thread([this, i] { worker(i); });
    }
}

bool device::impl::shutdown_workers() {
    for (size_t i = 0; i < RFNM_WORKER_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
        thread_data[i].rx_active = 0;
        thread_data[i].tx_active = 0;
        thread_data[i].shutdown_req = 1;
        thread_data[i].cv.notify_all();
    }

    // A worker blocked in a data socket read can't observe shutdown_req, and close()
    // alone doesn't interrupt an in-progress blocking recv - shutdown() does. Checking
    // tcp_data_connected also guarantees the connecting thread finished the assignment.
    if (tcp_data_connected && net::sock_valid(tcp_data_socket)) {
        net::sock_shutdown(tcp_data_socket);
    }

    bool threads_detached = false;
    for (size_t i = 0; i < RFNM_WORKER_COUNT; i++) {
        if (thread_c[i].joinable()) {
            // bounded join: a worker wedged in a transport call must not hang the dtor
            auto future = std::async(std::launch::async, [&]() {
                thread_c[i].join();
                });

            if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::timeout) {
                spdlog::warn("worker thread {} timed out, detaching", i);
                thread_c[i].detach();
                // a detached worker still dereferences this impl's state (queues,
                // sockets, caches): the caller must leak the session state
                // deliberately and loudly; process teardown reclaims it
                threads_detached = true;
            }
        }
    }
    return threads_detached;
}

// Stream lifecycle (worker arming) + the public queue surface (thin bodies over the
// core machinery in core/rx_queue.cpp and core/tx_queue.cpp).

#include <spdlog/spdlog.h>

#include <librfnm/rx_stream.h>

#include "../core/impl.h"

using namespace rfnm;

MSDLL rx_stream* device::rx_stream_create(uint8_t ch_ids) {
    // Honor device-reported channel availability: warn if a requested channel is marked
    // unavailable. The device remains the authoritative gate and will reject it on apply.
    for (uint32_t ch = 0; ch < MAX_RX_CHANNELS; ch++) {
        if ((ch_ids & (1u << ch)) && !is_rx_channel_available(ch)) {
            spdlog::warn("rx_stream_create: rx channel {} reports avail=0", ch);
        }
    }

    pimpl->stream_format_locked = true;
    return new rx_stream(*this, ch_ids);
}

MSDLL rfnm_api_failcode device::rx_work_start() {
    impl& m = *pimpl;
    rfnm_api_failcode ret = RFNM_API_OK;

    m.rx_stream_count++;

    // no need to start workers if they're already running
    if (m.rx_stream_count > 1) return ret;

    m.stream_format_locked = true;

    // allocate buffers if the user didn't allocate them themselves
    if (!m.rx_s.qbuf_cnt) {
        m.rx_buffers_allocated = true;
        size_t bufsize = RFNM_USB_RX_PACKET_ELEM_CNT * m.transport_status.rx_stream_format;

        for (size_t i = 0; i < MIN_RX_BUFCNT; i++) {
            rx_buf* rxbuf = new rx_buf();
            rxbuf->buf = new uint8_t[bufsize];
            m.rx_qbuf(rxbuf, true);
        }
    }

    // expected CC of UINT64_MAX is a special value meaning to accept whatever comes
    for (int adc_id = 0; adc_id < 4; adc_id++) {
        m.rx_s.usb_cc[adc_id] = UINT64_MAX;
        m.rx_s.usb_cc_max_seen[adc_id] = 0;
        m.rx_s.delivered_cc_valid[adc_id] = false;
    }

    // tell the USB RX engine to re-arm its dead-pipe watchdog baselines for the new
    // session (bumped before the workers wake so the first RX pass sees the new gen);
    // a fresh session also gets a fresh dead-pipe verdict
    m.ses.rx_pipe_dead_latch = false;
    m.ses.rx_work_generation++;

    for (size_t i = 0; i < RFNM_WORKER_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(m.thread_data[i].cv_mutex);
        m.thread_data[i].rx_active = (i == 1) ? 1 : 0;  // worker 1 owns RX on every transport
        m.thread_data[i].cv.notify_all();
    }

    return ret;
}

MSDLL rfnm_api_failcode device::rx_work_stop() {
    impl& m = *pimpl;
    rfnm_api_failcode ret = RFNM_API_OK;

    if (m.rx_stream_count > 0) m.rx_stream_count--;

    if (m.rx_stream_count == 0) {
        for (size_t i = 0; i < RFNM_WORKER_COUNT; i++) {
            std::lock_guard<std::mutex> lockGuard(m.thread_data[i].cv_mutex);
            m.thread_data[i].rx_active = 0;
        }
    }

    return ret;
}

MSDLL rfnm_api_failcode device::tx_work_start(enum tx_latency_policy policy) {
    impl& m = *pimpl;
    rfnm_api_failcode ret = RFNM_API_OK;

    for (size_t i = 0; i < RFNM_WORKER_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(m.thread_data[i].cv_mutex);
        // worker 0 owns TX on every transport: one thread on one ordered endpoint is
        // in-order by construction (throughput comes from the async URB pipeline on
        // that endpoint, never from more submitter threads)
        m.thread_data[i].tx_active = (i == 0) ? 1 : 0;
        m.thread_data[i].cv.notify_all();
    }

    return ret;
}

MSDLL rfnm_api_failcode device::tx_work_stop() {
    impl& m = *pimpl;
    rfnm_api_failcode ret = RFNM_API_OK;

    for (size_t i = 0; i < RFNM_WORKER_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(m.thread_data[i].cv_mutex);
        m.thread_data[i].tx_active = 0;
    }

    return ret;
}

MSDLL rfnm_api_failcode device::rx_qbuf(struct rx_buf* buf, bool new_buffer) {
    return pimpl->rx_qbuf(buf, new_buffer);
}

MSDLL rfnm_api_failcode device::rx_dqbuf(struct rx_buf** buf, uint8_t ch_ids, uint32_t timeout_us) {
    return pimpl->rx_dqbuf(buf, ch_ids, timeout_us);
}

MSDLL rfnm_api_failcode device::rx_flush(uint32_t timeout_us, uint8_t ch_ids) {
    return pimpl->rx_flush(timeout_us, ch_ids);
}

MSDLL void device::get_rx_stream_stats(uint64_t &ok, uint64_t &dropped) {
    impl& m = *pimpl;
    // unlocked reads: aligned 64-bit loads of monotonic counters, good enough for stats
    ok = 0;
    dropped = 0;
    for (int adc_id = 0; adc_id < 4; adc_id++) {
        ok += m.rx_s.usb_cc_ok[adc_id];
        dropped += m.rx_s.usb_cc_dropped[adc_id];
    }
}

MSDLL rfnm_api_failcode device::tx_qbuf(struct tx_buf* buf, uint32_t timeout_us) {
    return pimpl->tx_qbuf(buf, timeout_us);
}

MSDLL rfnm_api_failcode device::tx_dqbuf(struct tx_buf** buf) {
    return pimpl->tx_dqbuf(buf);
}

MSDLL rfnm_api_failcode device::tx_feed_seek(uint64_t samples) {
    return pimpl->tx_feed_seek(samples);
}

MSDLL rfnm_api_failcode device::tx_feed_seek_to(uint64_t abs_samples) {
    return pimpl->tx_feed_seek_to(abs_samples);
}

MSDLL uint64_t device::tx_feed_pos() {
    return pimpl->tx_feed_pos();
}

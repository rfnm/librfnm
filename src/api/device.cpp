// Device lifecycle + control plane: open/close orchestration across the transports,
// discovery dispatch, the control-transfer dispatcher, config fetch (get) and the
// atomic apply protocol with its cc-correlated result poll.

#include <algorithm>
#include <cstring>

#include <spdlog/spdlog.h>

#include "../core/impl.h"

using namespace rfnm;

// A device can enumerate at most once per transport
bool rfnm::sn_already_found(const std::vector<struct dev_info>& found, enum transport transport, const uint8_t* sn) {
    for (auto& dev : found) {
        if (dev.transport == transport &&
            std::memcmp(dev.hwinfo.motherboard.serial_number, sn, sizeof(dev.hwinfo.motherboard.serial_number)) == 0) {
            return true;
        }
    }
    return false;
}

MSDLL device::device(enum transport transport, std::string address, uint32_t flags) {
    pimpl = new impl();
    impl& m = *pimpl;

    m.open_flags = flags;

    // default/native stream format
    m.transport_status.rx_stream_format = STREAM_FORMAT_CS16;
    m.transport_status.tx_stream_format = STREAM_FORMAT_CS16;

    if (transport == TRANSPORT_USB || transport == TRANSPORT_FIND) {
        if (m.usb_open(address)) {
            m.start_workers();
            return;
        }
        if (transport != TRANSPORT_FIND) {
            delete pimpl;
            throw std::runtime_error("RFNM culdn't find any USB devices");
        }
    }

    if (transport == TRANSPORT_LOCAL || transport == TRANSPORT_FIND) {
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
        if (m.local_open()) {
            m.start_workers();
            return;
        }
#endif
    }

    if (transport == TRANSPORT_TCP || transport == TRANSPORT_FIND) {
        if (m.tcp_open(address)) {
            m.start_workers();
            return;
        }
    }

    // every transport declined and cleaned up after itself
    delete pimpl;
    throw std::runtime_error("Couldn't find any RFNM device");
}

MSDLL device::~device() {
    impl& m = *pimpl;

    if (m.shutdown_workers()) {
        // a live (detached) worker still references the queues, caches and transport
        // handles - freeing any of it here is a use-after-free. Leak the session
        // deliberately and loudly; process exit reclaims it.
        spdlog::error("leaking device session state: a worker thread outlived teardown (detached) - not freeing pools/transport/state under it");
        return;
    }

    // Clean up buffers. Pure-auto contract: when the device allocated the pool (the
    // user queued nothing before rx_work_start), it owns every buffer in the queues.
    if (m.rx_buffers_allocated) {
        m.rx_flush(0, 0xFF);
        std::lock_guard<std::mutex> lk_in(m.rx_s.in_mutex);
        std::lock_guard<std::mutex> lk_out(m.rx_s.out_mutex);
        while (!m.rx_s.in.empty()) {
            rx_buf* b = m.rx_s.in.front();
            m.rx_s.in.pop();
            delete[] b->buf;
            delete b;
        }
        for (int a = 0; a < 4; a++) {
            while (!m.rx_s.out[a].empty()) {
                rx_buf* b = m.rx_s.out[a].top();
                m.rx_s.out[a].pop();
                delete[] b->buf;
                delete b;
            }
        }
    }

    if (m.transport_status.transport == TRANSPORT_USB) {
        m.usb_close();
    }

    if (m.transport_status.transport == TRANSPORT_TCP) {
        m.tcp_close();
    }

    delete pimpl;
}

MSDLL std::vector<struct dev_info> device::find(enum transport transport, std::string address) {
    std::vector<struct dev_info> found = {};

    if (transport == TRANSPORT_USB || transport == TRANSPORT_FIND) {
        usb_find_into(found, address);
    }

    if (transport == TRANSPORT_LOCAL || transport == TRANSPORT_FIND) {
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
        local_find_into(found);
#endif
    }

    if (transport == TRANSPORT_TCP || transport == TRANSPORT_FIND) {
        tcp_find_into(found, address);
    }

    return found;
}

rfnm_api_failcode device::impl::ctrl_transfer(enum rfnm_control_ep type, uint32_t size, uint8_t* buf, uint32_t timeout_ms) {
    switch (transport_status.transport) {
    case TRANSPORT_USB:
        return usb_ctrl_transfer(type, size, buf, timeout_ms);
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
    case TRANSPORT_LOCAL:
        return local_ctrl_transfer(type, size, buf, timeout_ms);
#endif
    case TRANSPORT_TCP:
        return tcp_ctrl_transfer(type, size, buf, timeout_ms);
    default:
        return RFNM_API_USB_FAIL;
    }
}

rfnm_api_failcode device::impl::get(enum req_type type) {
    if (type & REQ_HWINFO) {
        struct rfnm_dev_hwinfo r_hwinfo;
        if (ctrl_transfer(RFNM_GET_DEV_HWINFO, sizeof(struct rfnm_dev_hwinfo), (unsigned char*)&r_hwinfo, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        memcpy(&hwinfo, &r_hwinfo, sizeof(struct rfnm_dev_hwinfo));

        if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
            uint32_t pv = r_hwinfo.protocol_version;
            spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED detected protocol is v{} while your librfnm is for v{}", pv, RFNM_PROTOCOL_VERSION);
            return RFNM_API_SW_UPGRADE_REQUIRED;
        }
    }

    if (type & REQ_TX) {
        struct rfnm_dev_tx_ch_list r_chlist;

        if (ctrl_transfer(RFNM_GET_TX_CH_LIST, sizeof(struct rfnm_dev_tx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        memcpy(&tx, &r_chlist, sizeof(struct rfnm_dev_tx_ch_list));
    }

    if (type & REQ_RX) {
        struct rfnm_dev_rx_ch_list r_chlist;

        if (ctrl_transfer(RFNM_GET_RX_CH_LIST, sizeof(struct rfnm_dev_rx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        memcpy(&rx, &r_chlist, sizeof(struct rfnm_dev_rx_ch_list));
    }

    if (type & REQ_DEV_STATUS) {
        struct rfnm_dev_status r_status;

        if (ctrl_transfer(RFNM_GET_DEV_STATUS, sizeof(struct rfnm_dev_status), (unsigned char*)&r_status, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        // the worker poll writes dev_status under s_dev_status_mutex and tx_qbuf
        // STAMPS from it under the same lock - an unlocked memcpy here could hand
        // tx_qbuf a torn tx_t0/tx_epoch pair exactly at a self-heal re-mint
        // (silently wrong-slot TX). Same discipline as the worker's own memcpy.
        {
            std::lock_guard<std::mutex> lockGuard(s_dev_status_mutex);
            memcpy(&dev_status, &r_status, sizeof(struct rfnm_dev_status));
            last_dev_time = std::chrono::high_resolution_clock::now();
            // device-time cache: phytimer_now paired with the host clock
            ptmr_at_status = dev_status.phytimer_now;
            host_at_status = std::chrono::steady_clock::now();
            ptmr_at_status_valid = (ptmr_at_status != 0);
        }
    }

    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::get(enum req_type type) {
    return pimpl->get(type);
}

// THE one session create/reset point, called at open right after the SM reset (the
// only SM-reset moment in a device's life). The impl is freshly zero-constructed at
// open, so this is idempotent there - it exists so the reset SET is named in exactly
// one place instead of smeared across call sites.
void device::impl::reset_session() {
    ses.reset();
    stream_format_locked = false;
    tx_s.usb_cc = 0;
    tx_s.qbuf_cnt = 0;

    for (int adc_id = 0; adc_id < 4; adc_id++) {
        rx_s.usb_cc[adc_id] = UINT64_MAX;
        rx_s.usb_cc_dropped[adc_id] = 0;
        rx_s.usb_cc_ok[adc_id] = 0;
        rx_s.usb_cc_max_seen[adc_id] = 0;
        rx_s.delivered_cc_valid[adc_id] = false;
    }
}

MSDLL rfnm_api_failcode device::apply(uint16_t applies, bool confirm_execution, uint32_t timeout_us) {
    impl& m = *pimpl;
    uint8_t applies_ch_tx = applies & 0xff;
    uint8_t applies_ch_rx = (applies & 0xff00) >> 8;

    if (applies_ch_tx) {
        struct rfnm_dev_tx_ch_list r_chlist;
        memcpy(&r_chlist, &m.tx, sizeof(struct rfnm_dev_tx_ch_list));
        r_chlist.apply = applies_ch_tx;
        r_chlist.cc = ++m.ses.cc_tx;

        if (m.ctrl_transfer(RFNM_SET_TX_CH_LIST, sizeof(struct rfnm_dev_tx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
    }

    if (applies_ch_rx) {
        struct rfnm_dev_rx_ch_list r_chlist;
        memcpy(&r_chlist, &m.rx, sizeof(struct rfnm_dev_rx_ch_list));
        r_chlist.apply = applies_ch_rx;
        r_chlist.cc = ++m.ses.cc_rx;

        if (m.ctrl_transfer(RFNM_SET_RX_CH_LIST, sizeof(struct rfnm_dev_rx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
    }

    if (confirm_execution) {
        using std::chrono::high_resolution_clock;
        using std::chrono::duration_cast;
        using std::chrono::microseconds;

        auto tstart = high_resolution_clock::now();

        while (1) {
            struct rfnm_dev_get_set_result_ext r_res;

            if (m.ctrl_transfer(RFNM_GET_SET_RESULT, sizeof(struct rfnm_dev_get_set_result_ext), (unsigned char*)&r_res, 50) != RFNM_API_OK) {
                return RFNM_API_USB_FAIL;
            }

            if (r_res.base.cc_rx == m.ses.cc_rx && r_res.base.cc_tx == m.ses.cc_tx) {
                m.ses.last_set_res = r_res;
                for (size_t q = 0; q < MAX_TX_CHANNELS; q++) {
                    if ((channel_flags[q] & applies_ch_tx) && r_res.base.tx_ecodes[q]) {
                        spdlog::error("apply rejected: {}", get_apply_error(q, true));
                        return (rfnm_api_failcode)r_res.base.tx_ecodes[q];
                    }
                }
                for (size_t q = 0; q < MAX_RX_CHANNELS; q++) {
                    if ((channel_flags[q] & applies_ch_rx) && r_res.base.rx_ecodes[q]) {
                        spdlog::error("apply rejected: {}", get_apply_error(q, false));
                        return (rfnm_api_failcode)r_res.base.rx_ecodes[q];
                    }
                }
                if (applies_ch_tx && (r_res.timing_break & 0x2)) {
                    // the device says THIS apply actually broke the TX timeline (v5
                    // timing_break bit1: a real stream send / re-mint), so the feed
                    // axis rebases: feed position 0 airs at the NEW anchor. The mint
                    // is NOT always synchronous with the apply result (Blue: yes;
                    // geul: ~800 ms async FE programming) - so the session anchor
                    // latch is INVALIDATED here, not re-latched: it pins at the
                    // client's next tx_feed_seek_to (after apply_timing_settled) or
                    // at the first stamped packet. Stamping from a possibly-stale
                    // refresh here would fork seeks from stamps by the re-mint delta.
                    // A FE-only TX apply (deduped word, bit1 clear) MUST leave the
                    // feed axis alone: zeroing feed_pos while the device kept its
                    // timeline forks the client's feed accounting from the lib's by
                    // exactly the already-fed span - every subsequent stamp lands
                    // that many samples early.
                    m.get(REQ_DEV_STATUS);
                    std::lock_guard<std::mutex> lg(m.tx_s.in_mutex);
                    m.ses.feed_pos = 0;
                    m.ses.tx_anchor_valid = false;
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

MSDLL const struct rfnm_dev_hwinfo* device::get_hwinfo() {
    return &(pimpl->hwinfo);
}

MSDLL const struct rfnm_dev_status* device::get_dev_status() {
    return &(pimpl->dev_status);
}

MSDLL const struct transport_status* device::get_transport_status() {
    return &(pimpl->transport_status);
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

    if (!path.compare("terminated") || !path.compare("term")) {
        return RFNM_PATH_TERMINATED;
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
    else if (path == RFNM_PATH_TERMINATED) {
        return "terminated";
    }
    else {
        return std::string(1, 'A' + (int)(path));
    }
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
    case RFNM_API_SCHED_NO_ANCHOR:
        return "Schedule rejected: no TX anchor published yet (stream not started)";
    case RFNM_API_SCHED_MISALIGNED:
        return "Schedule rejected: tick not on the slot grid";
    case RFNM_API_SCHED_LATE:
        return "Schedule rejected: below the producer lead floor";
    case RFNM_API_SCHED_FULL:
        return "Schedule rejected: request ring full";
    case RFNM_API_SCHED_ORDER:
        return "Schedule rejected: non-monotonic tick";
    case RFNM_API_SCHED_NOT_RESET:
        return "Schedule rejected: ring never reset (call schedule_reset first)";
    case RFNM_API_RX_PIPE_DEAD:
        return "RX pipe unrecoverably dead for this session (resync budget exhausted)";
    case RFNM_API_TX_NOT_ANCHORED:
        return "Positional write refused: no TX anchor latched or published";
    default:
        return "Unknown error code";
    }
}

// Channel configuration surface: the staged setters + atomic apply masks, sample-rate
// reconfig with its cc-correlated result poll, stream format, and the human-readable
// apply-rejection composer.

#include <algorithm>
#include <cstdio>
#include <cstring>

#include <spdlog/spdlog.h>

#include "../core/impl.h"
#include "../core/timebase.h"

using namespace rfnm;

MSDLL rfnm_api_failcode device::set_stream_format(enum stream_format format, size_t* bufsize, uint8_t* bytes_per_ele) {
    impl& m = *pimpl;
    if (m.stream_format_locked) {
        if (bufsize) {
            *bufsize = RFNM_USB_RX_PACKET_ELEM_CNT * m.transport_status.rx_stream_format;
        }
        if (bytes_per_ele) {
            *bytes_per_ele = m.transport_status.rx_stream_format;
        }

        if (format == m.transport_status.rx_stream_format) {
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
        m.transport_status.rx_stream_format = format;
        m.transport_status.tx_stream_format = format;
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
    impl& m = *pimpl;
    struct rfnm_dev_set_samp_rate r_sr;
    r_sr.freq = freq;
    r_sr.cc = ++m.ses.cc_samp_rate;

    if (m.ctrl_transfer(RFNM_SET_SAMP_RATE, sizeof(struct rfnm_dev_set_samp_rate), (unsigned char*)&r_sr, 50) != RFNM_API_OK) {
        return RFNM_API_USB_FAIL;
    }

    // Poll GET_SET_RESULT until the device reports this samp-rate command (cc match),
    // then return its status. Same cc correlation as apply()'s cc_tx/cc_rx + ecodes.
    auto tstart = std::chrono::high_resolution_clock::now();
    while (1) {
        struct rfnm_dev_get_set_result_ext r_res;

        if (m.ctrl_transfer(RFNM_GET_SET_RESULT, sizeof(struct rfnm_dev_get_set_result_ext), (unsigned char*)&r_res, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }

        if (r_res.base.cc_samp_rate == m.ses.cc_samp_rate) {
            m.ses.last_set_res = r_res;
            // refresh the client-visible hwinfo with the device's value
            if (m.get(REQ_HWINFO)) {
                return RFNM_API_USB_FAIL;
            }
            if (r_res.base.samp_rate_ecode == 0) {
                // cache for the RX dead-pipe watchdog's trickle judgment
                m.ses.samp_rate_hz = freq;
            } else {
                spdlog::error("set_samp_rate rejected: {} Hz outside the supported set or no synthesizable "
                    "DCS plan (hwinfo.clock advertises [{}..{}] Hz; concurrent duplex additionally "
                    "requires shared-DCS rate x2 in [50e6..100e6] - defect #27)",
                    freq, (uint64_t)m.hwinfo.clock.samp_rate_min, (uint64_t)m.hwinfo.clock.samp_rate_max);
            }
            return (rfnm_api_failcode)r_res.base.samp_rate_ecode;
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

static const char* rfnm_rej_field_name(uint8_t f) {
    switch (f) {
    case RFNM_REJ_FREQ: return "freq";
    case RFNM_REJ_LPF_BW: return "rfic_lpf_bw";
    case RFNM_REJ_POWER: return "power";
    case RFNM_REJ_GAIN: return "gain";
    case RFNM_REJ_DC_TRIM: return "rfic dc trim";
    case RFNM_REJ_ENABLE: return "enable";
    case RFNM_REJ_STREAM: return "stream";
    case RFNM_REJ_AGC: return "agc";
    case RFNM_REJ_BIAS_TEE: return "bias_tee";
    case RFNM_REJ_FM_NOTCH: return "fm_notch";
    case RFNM_REJ_PATH: return "path";
    case RFNM_REJ_RATE: return "sample rate / stream plan";
    case RFNM_REJ_CH_MISSING: return "channel presence";
    case RFNM_REJ_DEVICE: return "device apply";
    default: return "(unattributed)";
    }
}

MSDLL std::string device::get_apply_error(uint8_t channel, bool tx) {
    impl& m = *pimpl;
    char msg[320];
    const char* dir = tx ? "tx" : "rx";

    if (channel >= 8) {
        return "invalid channel index";
    }
    int32_t ecode = tx ? m.ses.last_set_res.base.tx_ecodes[channel] : m.ses.last_set_res.base.rx_ecodes[channel];
    uint8_t field = tx ? m.ses.last_set_res.tx_reject_field[channel] : m.ses.last_set_res.rx_reject_field[channel];
    if (!ecode) {
        snprintf(msg, sizeof(msg), "%s%u: no error recorded on the last apply", dir, channel);
        return msg;
    }

    switch (field) {
    case RFNM_REJ_GAIN: {
        auto& ch = m.rx.ch[channel];
        snprintf(msg, sizeof(msg), "rx%u gain out of range: requested %d, allowed [%d..%d] dB",
                channel, (int)ch.gain, (int)ch.gain_range.min, (int)ch.gain_range.max);
        break;
    }
    case RFNM_REJ_POWER: {
        auto& ch = m.tx.ch[channel];
        snprintf(msg, sizeof(msg), "tx%u power out of range: requested %d, allowed [%d..%d]",
                channel, (int)ch.power, (int)ch.power_range.min, (int)ch.power_range.max);
        break;
    }
    case RFNM_REJ_FREQ: {
        int64_t f  = tx ? m.tx.ch[channel].freq     : m.rx.ch[channel].freq;
        int64_t lo = tx ? m.tx.ch[channel].freq_min : m.rx.ch[channel].freq_min;
        int64_t hi = tx ? m.tx.ch[channel].freq_max : m.rx.ch[channel].freq_max;
        snprintf(msg, sizeof(msg), "%s%u freq out of range: requested %.6f MHz, allowed [%.3f..%.3f] MHz",
                dir, channel, f / 1e6, lo / 1e6, hi / 1e6);
        break;
    }
    case RFNM_REJ_LPF_BW: {
        int bw = tx ? m.tx.ch[channel].rfic_lpf_bw : m.rx.ch[channel].rfic_lpf_bw;
        snprintf(msg, sizeof(msg), "%s%u rfic_lpf_bw invalid: requested %d MHz (must be >= 0)", dir, channel, bw);
        break;
    }
    case RFNM_REJ_DC_TRIM: {
        auto& ch = m.rx.ch[channel];
        snprintf(msg, sizeof(msg), "rx%u rfic dc trim out of range: requested i=%d q=%d, allowed [-63..63]",
                channel, (int)ch.rfic_dc_i, (int)ch.rfic_dc_q);
        break;
    }
    case RFNM_REJ_ENABLE:
    case RFNM_REJ_STREAM:
    case RFNM_REJ_AGC:
    case RFNM_REJ_BIAS_TEE:
    case RFNM_REJ_FM_NOTCH:
    case RFNM_REJ_PATH:
        snprintf(msg, sizeof(msg), "%s%u %s: invalid enum value for this field", dir, channel, rfnm_rej_field_name(field));
        break;
    case RFNM_REJ_RATE:
        if (ecode == RFNM_API_TIMEOUT) {
            snprintf(msg, sizeof(msg), "%s%u stream apply timed out (device busy / reclock in progress)", dir, channel);
        } else {
            snprintf(msg, sizeof(msg), "%s%u stream plan rejected: the applied rate/direction combination has no "
                    "DCS plan (concurrent duplex supports shared-DCS rate x2 in [50e6..100e6] only - defect #27; "
                    "hwinfo.clock advertises the single-direction limits)", dir, channel);
        }
        break;
    case RFNM_REJ_CH_MISSING:
        snprintf(msg, sizeof(msg), "%s%u is not present on this device (apply mask named an absent channel)", dir, channel);
        break;
    case RFNM_REJ_DEVICE:
        snprintf(msg, sizeof(msg), "%s%u device-layer apply failed: %s (not a validation error - see board dmesg)",
                dir, channel, failcode_to_string((rfnm_api_failcode)ecode));
        break;
    default:
        snprintf(msg, sizeof(msg), "%s%u rejected: %s (no field attribution)", dir, channel,
                failcode_to_string((rfnm_api_failcode)ecode));
        break;
    }
    return msg;
}

MSDLL const struct rfnm_api_rx_ch* device::get_rx_channel(uint32_t channel) {
    if (channel < MAX_RX_CHANNELS) {
        return &(pimpl->rx.ch[channel]);
    }
    else {
        return nullptr;
    }
}

MSDLL const struct rfnm_api_tx_ch* device::get_tx_channel(uint32_t channel) {
    if (channel < MAX_TX_CHANNELS) {
        return &(pimpl->tx.ch[channel]);
    }
    else {
        return nullptr;
    }
}

// Channel availability is authoritative from the device: rfnm_dgb_reg_{rx,tx}_ch() marks
// every registered channel avail=1, so the channel list reflects exactly which channels
// exist. Enumeration counts available channels directly.
MSDLL uint32_t device::get_rx_channel_count() {
    uint32_t count = 0;

    for (uint32_t i = 0; i < MAX_RX_CHANNELS; i++) {
        if (pimpl->rx.ch[i].avail) {
            count++;
        }
    }

    return count;
}

MSDLL uint32_t device::get_tx_channel_count() {
    uint32_t count = 0;

    for (uint32_t i = 0; i < MAX_TX_CHANNELS; i++) {
        if (pimpl->tx.ch[i].avail) {
            count++;
        }
    }

    return count;
}

MSDLL bool device::is_rx_channel_available(uint32_t channel) {
    return channel < MAX_RX_CHANNELS && pimpl->rx.ch[channel].avail != 0;
}

MSDLL bool device::is_tx_channel_available(uint32_t channel) {
    return channel < MAX_TX_CHANNELS && pimpl->tx.ch[channel].avail != 0;
}

MSDLL rfnm_api_failcode device::set_rx_channel_status(uint32_t channel, enum rfnm_ch_enable enable,
    enum rfnm_ch_stream stream, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        pimpl->rx.ch[channel].enable = enable;
        pimpl->rx.ch[channel].stream = stream;

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

// Force off every RX channel the device reports enabled - recovery from a client
// that died without cleanup, whose stale enabled channels block new stream creation.
// The apply mask names only those channels: the device validates every channel named
// in a mask, and the untouched defaults of a never-configured channel fail that
// validation. Deliberately NOT called from rx_stream::start(): a second stream on the
// same device object would tear down channels the first one is using - call it after
// open, before staging your own channel configuration.
MSDLL rfnm_api_failcode device::rx_disable_stale_channels(uint32_t timeout_us) {
    rfnm_api_failcode ret = get(REQ_RX);
    if (ret != RFNM_API_OK) {
        return ret;
    }

    uint16_t applies = 0;
    for (uint32_t channel = 0; channel < MAX_RX_CHANNELS; channel++) {
        if (!is_rx_channel_available(channel)) {
            continue;
        }
        if (pimpl->rx.ch[channel].enable != RFNM_CH_RF_OFF) {
            pimpl->rx.ch[channel].enable = RFNM_CH_RF_OFF;
            pimpl->rx.ch[channel].stream = RFNM_CH_STREAM_AUTO;
            applies |= rx_channel_apply_flags[channel];
        }
    }

    if (!applies) {
        return RFNM_API_OK;
    }
    return apply(applies, true, timeout_us);
}

MSDLL rfnm_api_failcode device::set_rx_channel_freq(uint32_t channel, int64_t freq, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        pimpl->rx.ch[channel].freq = freq;

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
        pimpl->rx.ch[channel].rfic_lpf_bw = bw;

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
        pimpl->rx.ch[channel].gain = gain;

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
        pimpl->rx.ch[channel].agc = agc;

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
        pimpl->rx.ch[channel].fm_notch = fm_notch;

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

MSDLL rfnm_api_failcode device::set_rx_channel_fe_mode(uint32_t channel, enum rfnm_rx_fe_mode fe_mode, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        pimpl->rx.ch[channel].fe_mode = fe_mode;

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
        pimpl->rx.ch[channel].bias_tee = bias_tee;

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
        pimpl->rx.ch[channel].path = path;

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
        pimpl->tx.ch[channel].enable = enable;
        pimpl->tx.ch[channel].stream = stream;

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

MSDLL rfnm_api_failcode device::set_tx_channel_freq(uint32_t channel, int64_t freq, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        pimpl->tx.ch[channel].freq = freq;

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
        pimpl->tx.ch[channel].rfic_lpf_bw = bw;

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
        pimpl->tx.ch[channel].power = power;

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
        pimpl->tx.ch[channel].bias_tee = bias_tee;

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
        pimpl->tx.ch[channel].path = path;

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

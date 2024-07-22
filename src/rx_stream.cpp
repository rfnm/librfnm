#include <librfnm/rx_stream.h>
#include <spdlog/spdlog.h>

using namespace rfnm;

MSDLL rx_stream::rx_stream(device &rfnm, uint8_t ch_ids) : dev(rfnm) {
    for (uint32_t channel = 0; channel < dev.get_rx_channel_count(); channel++) {
        if (!(ch_ids & channel_flags[channel])) continue;
        channels.push_back(channel);
    }

    outbufsize = RFNM_USB_RX_PACKET_ELEM_CNT * dev.get_transport_status()->rx_stream_format;

    int16_t m = 1, n = 1;
    if (channels.size() > 0) {
        m = dev.get_rx_channel(channels[0])->samp_freq_div_m;
        n = dev.get_rx_channel(channels[0])->samp_freq_div_n;
    }

    // check that all channels have matching sample rates before we allocate anything
    for (uint32_t channel : channels) {
        if (dev.get_rx_channel(channel)->samp_freq_div_m != m ||
                dev.get_rx_channel(channel)->samp_freq_div_n != n) {
            spdlog::error("stream sample rate mismatch");
            throw std::runtime_error("stream sample rate mismatch");
        }
    }

    phytimer_ticks_per_sample = 4 * n;
    ns_per_sample = n * 1e9 / dev.get_hwinfo()->clock.dcs_clk;
}

MSDLL rx_stream::~rx_stream() {
    stop();
}

template <class T>
static void measQuadDcOffset(const T *buf, size_t n, T *offsets, float filter_coeff) {
    assert((n & 0x7) == 0);

    float accum[8] = {};

    for (size_t i = 0; i < n; i += 8) {
        #pragma GCC unroll 8
        for (size_t j = 0; j < 8; j++) {
            accum[j] += buf[i+j];
        }
    }

    float f = 8.0f / n;
    for (size_t j = 0; j < 8; j++) {
        accum[j] *= f;
        offsets[j] = accum[j] * filter_coeff + offsets[j] * (1.0f - filter_coeff);
    }
}

template <class T>
static void applyQuadDcOffset(T *buf, size_t n, const T *offsets) {
    assert((n & 0x7) == 0);

    for (size_t i = 0; i < n; i += 8) {
        #pragma GCC unroll 8
        for (size_t j = 0; j < 8; j++) {
            buf[i+j] -= offsets[j];
        }
    }
}

MSDLL rfnm_api_failcode rx_stream::start() {
    rfnm_api_failcode ret = RFNM_API_OK;

    // starting rx worker without any channels streaming will cause errors
    if (channels.size() == 0) return ret;

    dev.rx_work_start();
    stream_active = true;

    uint16_t apply_mask = 0;
    uint8_t chan_mask = 0;
    for (uint32_t channel : channels) {
        // can't have multiple streams running on a channel
        if (dev.get_rx_channel(channel)->enable != RFNM_CH_OFF) {
            ret = RFNM_API_NOT_SUPPORTED;
            goto error;
        }

        dev.set_rx_channel_active(channel, RFNM_CH_ON, RFNM_CH_STREAM_AUTO, false);
        apply_mask |= rx_channel_apply_flags[channel];
        chan_mask |= channel_flags[channel];
    }

    // flush old junk before streaming new data
    ret = dev.rx_flush(20000, chan_mask);
    if (ret) goto error;

    // start ADCs
    ret = dev.set(apply_mask);
    if (ret) goto error;

    // work around stale buffer firmware bug by discarding first few buffers
    rx_dqbuf_multi(500000, true);
    rx_qbuf_multi();
    rx_dqbuf_multi(100000, true);
    rx_qbuf_multi();
    rx_dqbuf_multi(50000, true);
    rx_qbuf_multi();

    // First sample can sometimes take a while to come, so fetch it here before normal streaming
    ret = rx_dqbuf_multi(50000, true);
    if (ret) goto error;

error:
    if (ret) {
        dev.rx_work_stop();
        stream_active = false;
    }

    return ret;
}

MSDLL rfnm_api_failcode rx_stream::stop() {
    rfnm_api_failcode ret = RFNM_API_OK;

    if (!stream_active) return ret;

    // stop the receiver threads
    dev.rx_work_stop();
    stream_active = false;

    // stop the ADCs
    uint16_t apply_mask = 0;
    uint8_t chan_mask = 0;
    for (uint32_t channel : channels) {
        dev.set_rx_channel_active(channel, RFNM_CH_OFF, RFNM_CH_STREAM_AUTO, false);
        apply_mask |= rx_channel_apply_flags[channel];
        chan_mask |= channel_flags[channel];
    }
    ret = dev.set(apply_mask);

    // flush buffers
    dev.rx_flush(0, chan_mask);
    rx_qbuf_multi();

    return ret;
}

MSDLL void rx_stream::set_auto_dc_offset(bool enabled, uint8_t channel_mask) {
    for (uint32_t channel : channels) {
        if (channel_mask & channel_flags[channel]) {
            dc_correction[channel] = enabled;
        }
    }
}

MSDLL rfnm_api_failcode rx_stream::read(void * const * buffs, size_t elems_to_read,
        size_t &elems_read, uint64_t &timestamp_ns, uint32_t timeout_us) {
    rfnm_api_failcode ret = RFNM_API_OK;

    auto timeout = std::chrono::system_clock::now() + std::chrono::microseconds(timeout_us);
    size_t bytes_per_ele = dev.get_transport_status()->rx_stream_format;
    size_t read_elems = 0;
    bool need_more_data = false;

    do {
        // only copy as many samples as we have ready on all channels
        size_t samples_avail = SIZE_MAX;
        size_t samples_to_copy = 0;
        for (uint32_t channel : channels) {
            if (samples_left[channel] < samples_avail) {
                samples_avail = samples_left[channel];
            }
        }

        if (samples_avail < elems_to_read - read_elems) {
            need_more_data = true;
            samples_to_copy = samples_avail;
        } else {
            need_more_data = false;
            samples_to_copy = elems_to_read - read_elems;
        }

        // copy equal amounts of data for each channel
        size_t buf_idx = 0;
        for (uint32_t channel : channels) {
            size_t ch_samples_to_copy = samples_to_copy;
            uint8_t *dst = reinterpret_cast<uint8_t *>(buffs[buf_idx++]) + read_elems * bytes_per_ele;

            // large samples_left value means prepend zero padding for alignment purposes
            if (samples_left[channel] > RFNM_USB_RX_PACKET_ELEM_CNT) {
                size_t pad_samples = samples_left[channel] - RFNM_USB_RX_PACKET_ELEM_CNT;
                if (pad_samples > ch_samples_to_copy) {
                    pad_samples = ch_samples_to_copy;
                }

                std::memset(dst, 0, pad_samples * bytes_per_ele);
                dst += pad_samples * bytes_per_ele;
                samples_left[channel] -= pad_samples;
                ch_samples_to_copy -= pad_samples;
            }

            uint8_t *src = pending_rx_buf[channel]->buf +
                (RFNM_USB_RX_PACKET_ELEM_CNT - samples_left[channel]) * bytes_per_ele;
            std::memcpy(dst, src, ch_samples_to_copy * bytes_per_ele);
            samples_left[channel] -= ch_samples_to_copy;
        }

        read_elems += samples_to_copy;

        if (need_more_data) {
            uint32_t wait_us = 0;
            auto time_remaining = timeout - std::chrono::system_clock::now();
            if (time_remaining > std::chrono::duration<int64_t>::zero()) {
                wait_us = std::chrono::duration_cast<std::chrono::microseconds>(time_remaining).count();
            }

            ret = rx_dqbuf_multi(wait_us);
            if (ret) break;
        }
    } while (need_more_data);

    timestamp_ns = (uint64_t)(sample_counter * ns_per_sample);
    elems_read = read_elems;
    sample_counter += read_elems;

    return ret;
}

rfnm_api_failcode rx_stream::rx_dqbuf_multi(uint32_t timeout_us, bool first) {
    rfnm_api_failcode ret = RFNM_API_OK;
    auto timeout = std::chrono::system_clock::now() + std::chrono::microseconds(timeout_us);
    const int32_t rounding_ticks = phytimer_ticks_per_sample / 2;
    uint32_t first_phytimer;
    bool first_phytimer_set = false;

    for (uint32_t channel : channels) {
        if (samples_left[channel] > 0) continue;

        if (pending_rx_buf[channel]) {
            dev.rx_qbuf(pending_rx_buf[channel]);
            pending_rx_buf[channel] = nullptr;
        }

        uint32_t wait_us = 0;
        auto time_remaining = timeout - std::chrono::system_clock::now();
        if (time_remaining > std::chrono::duration<int64_t>::zero()) {
            wait_us = std::chrono::duration_cast<std::chrono::microseconds>(time_remaining).count();
        }

        ret = dev.rx_dqbuf(&pending_rx_buf[channel], channel_flags[channel], wait_us);
        if (ret) break;

        samples_left[channel] = RFNM_USB_RX_PACKET_ELEM_CNT;

        if (dc_correction[channel]) {
            // periodically recalibrate DC offset to account for drift
            if ((pending_rx_buf[channel]->usb_cc & 0xF) == 0 || first) {
                float filter_factor = first ? 1.0f : 0.1f;

                switch (dev.get_transport_status()->rx_stream_format) {
                case STREAM_FORMAT_CS8:
                    measQuadDcOffset(reinterpret_cast<int8_t *>(pending_rx_buf[channel]->buf),
                            RFNM_USB_RX_PACKET_ELEM_CNT * 2, dc_offsets[channel].i8, filter_factor);
                    break;
                case STREAM_FORMAT_CS16:
                    measQuadDcOffset(reinterpret_cast<int16_t *>(pending_rx_buf[channel]->buf),
                            RFNM_USB_RX_PACKET_ELEM_CNT * 2, dc_offsets[channel].i16, filter_factor);
                    break;
                case STREAM_FORMAT_CF32:
                    measQuadDcOffset(reinterpret_cast<float *>(pending_rx_buf[channel]->buf),
                            RFNM_USB_RX_PACKET_ELEM_CNT * 2, dc_offsets[channel].f32, filter_factor);
                    break;
                }
            }

            switch (dev.get_transport_status()->rx_stream_format) {
            case STREAM_FORMAT_CS8:
                applyQuadDcOffset(reinterpret_cast<int8_t *>(pending_rx_buf[channel]->buf),
                        RFNM_USB_RX_PACKET_ELEM_CNT * 2, dc_offsets[channel].i8);
                break;
            case STREAM_FORMAT_CS16:
                applyQuadDcOffset(reinterpret_cast<int16_t *>(pending_rx_buf[channel]->buf),
                        RFNM_USB_RX_PACKET_ELEM_CNT * 2, dc_offsets[channel].i16);
                break;
            case STREAM_FORMAT_CF32:
                applyQuadDcOffset(reinterpret_cast<float *>(pending_rx_buf[channel]->buf),
                        RFNM_USB_RX_PACKET_ELEM_CNT * 2, dc_offsets[channel].f32);
                break;
            }
        }

        if (first) {
            if (!first_phytimer_set) {
                first_phytimer = pending_rx_buf[channel]->phytimer;
                first_phytimer_set = true;
            } else {
                int32_t samp_delta = static_cast<int32_t>(pending_rx_buf[channel]->phytimer - first_phytimer + rounding_ticks) /
                                     static_cast<int32_t>(phytimer_ticks_per_sample);
                //spdlog::info("second channel delayed by {} samples", samp_delta);
                samples_left[channel] += 55; //samp_delta;
            }
        } else {
            int32_t samp_delta = static_cast<int32_t>(pending_rx_buf[channel]->phytimer - last_phytimer[channel] + rounding_ticks) /
                                 static_cast<int32_t>(phytimer_ticks_per_sample);
            int32_t shift_samples = 0;

            // tolerance of +- 64 samples to deal with phytimer jitter
            // note that phytimer is dequeue time, subject to interrupt servicing time
            // phytimer timestamp is at a variable offset from buffer start time
            if (samp_delta < RFNM_USB_RX_PACKET_ELEM_CNT - 64) {
                // samples were repeated (strange, shouldn't happen)
                shift_samples = samp_delta - RFNM_USB_RX_PACKET_ELEM_CNT;
                spdlog::info("channel {} repeat {} samples", channel, -shift_samples);

                if (shift_samples < -RFNM_USB_RX_PACKET_ELEM_CNT) {
                    shift_samples = 0;
                    spdlog::info("phytimer flyback ignored");
                }
            } else if (samp_delta > RFNM_USB_RX_PACKET_ELEM_CNT + 64) {
                // samples were lost
                shift_samples = samp_delta - RFNM_USB_RX_PACKET_ELEM_CNT;
                spdlog::info("channel {} skip {} samples", channel, shift_samples);

                if (shift_samples > RFNM_USB_RX_PACKET_ELEM_CNT * 16) {
                    shift_samples = 0;
                    spdlog::info("phytimer jump ignored");
                }
            }

            samples_left[channel] += shift_samples;
        }

        last_phytimer[channel] = pending_rx_buf[channel]->phytimer;
    }

    return ret;
}

void rx_stream::rx_qbuf_multi() {
    for (uint32_t channel : channels) {
        if (pending_rx_buf[channel]) {
            dev.rx_qbuf(pending_rx_buf[channel]);
            pending_rx_buf[channel] = nullptr;
            samples_left[channel] = 0;
        }
    }
}

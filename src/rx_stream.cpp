#include <librfnm/rx_stream.h>
#include <spdlog/spdlog.h>

using namespace rfnm;

MSDLL rx_stream::rx_stream(device &rfnm, uint8_t ch_ids) : dev(rfnm) {
    for (uint32_t channel = 0; channel < dev.get_rx_channel_count(); channel++) {
        if (!(ch_ids & channel_flags[channel])) continue;
        channels.push_back(channel);
    }

    outbufsize = RFNM_USB_RX_PACKET_ELEM_CNT * dev.get_transport_status()->rx_stream_format;

    for (uint32_t channel : channels) {
        partial_rx_buf[channel].buf = new uint8_t[outbufsize];
        phytimer_ticks_per_sample[channel] = 4 * dev.get_rx_channel(channel)->samp_freq_div_n;
        ns_per_sample[channel] = dev.get_rx_channel(channel)->samp_freq_div_n * 1e9 / dev.get_hwinfo()->clock.dcs_clk;
        sample_counter[channel] = 0;
    }
}

MSDLL rx_stream::~rx_stream() {
    stop();

    for (uint32_t channel : channels) {
        delete[] partial_rx_buf[channel].buf;
    }
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
            dev.rx_work_stop();
            stream_active = false;
            return RFNM_API_NOT_SUPPORTED;
        }

        dev.set_rx_channel_active(channel, RFNM_CH_ON, RFNM_CH_STREAM_AUTO, false);
        apply_mask |= rx_channel_apply_flags[channel];
        chan_mask |= channel_flags[channel];
    }

    // flush old junk before streaming new data
    ret = dev.rx_flush(20, chan_mask);
    if (ret) return ret;

    // start ADCs
    ret = dev.set(apply_mask);
    if (ret) return ret;

    // work around stale buffer firmware bug by discarding first few buffers
    rx_dqbuf_multi(500);
    rx_qbuf_multi();
    rx_dqbuf_multi(100);
    rx_qbuf_multi();
    rx_dqbuf_multi(50);
    rx_qbuf_multi();

    uint32_t first_phytimer;
    bool first_phytimer_set = false;

    for (uint32_t channel : channels) {
        // First sample can sometimes take a while to come, so fetch it here before normal streaming
        // This first chunk is also useful for initial calibration
        struct rx_buf* lrxbuf;
        ret = dev.rx_dqbuf(&lrxbuf, channel_flags[channel], 250);
        if (ret) return ret;

        last_phytimer[channel] = lrxbuf->phytimer;

        // handle staggered ADC start times
        if (!first_phytimer_set) {
            first_phytimer = lrxbuf->phytimer;
            first_phytimer_set = true;
        } else {
            uint32_t rounding_ticks = phytimer_ticks_per_sample[channel] / 2;
            uint32_t samp_delta = (lrxbuf->phytimer - first_phytimer + rounding_ticks) /
                                  phytimer_ticks_per_sample[channel];
            sample_counter[channel] = samp_delta;
        }

        std::memcpy(partial_rx_buf[channel].buf, lrxbuf->buf, outbufsize);
        partial_rx_buf[channel].left = outbufsize;
        partial_rx_buf[channel].offset = 0;
        dev.rx_qbuf(lrxbuf);

        // Compute initial DC offsets
        switch (dev.get_transport_status()->rx_stream_format) {
        case LIBRFNM_STREAM_FORMAT_CS8:
            measQuadDcOffset(reinterpret_cast<int8_t *>(partial_rx_buf[channel].buf),
                    RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].i8, 1.0f);
            break;
        case LIBRFNM_STREAM_FORMAT_CS16:
            measQuadDcOffset(reinterpret_cast<int16_t *>(partial_rx_buf[channel].buf),
                    RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].i16, 1.0f);
            break;
        case LIBRFNM_STREAM_FORMAT_CF32:
            measQuadDcOffset(reinterpret_cast<float *>(partial_rx_buf[channel].buf),
                    RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].f32, 1.0f);
            break;
        }

        // Apply DC correction on first chunk if requested
        if (dc_correction[channel]) {
            switch (dev.get_transport_status()->rx_stream_format) {
            case LIBRFNM_STREAM_FORMAT_CS8:
                applyQuadDcOffset(reinterpret_cast<int8_t *>(partial_rx_buf[channel].buf),
                        RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].i8);
                break;
            case LIBRFNM_STREAM_FORMAT_CS16:
                applyQuadDcOffset(reinterpret_cast<int16_t *>(partial_rx_buf[channel].buf),
                        RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].i16);
                break;
            case LIBRFNM_STREAM_FORMAT_CF32:
                applyQuadDcOffset(reinterpret_cast<float *>(partial_rx_buf[channel].buf),
                        RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].f32);
                break;
            }
        }
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
    size_t read_elems[MAX_RX_CHANNELS] = {};
    size_t buf_idx = 0;
    bool need_more_data = false;
    bool time_set = false;
    uint64_t first_sample;
    size_t first_chan = SIZE_MAX;

    for (uint32_t channel : channels) {
        if (first_chan == SIZE_MAX) {
            first_chan = channel;
        }

        if (partial_rx_buf[channel].left) {
            size_t can_write_bytes = elems_to_read * bytes_per_ele;
            if (can_write_bytes > partial_rx_buf[channel].left) {
                can_write_bytes = partial_rx_buf[channel].left;
            }

            std::memcpy(((uint8_t*)buffs[buf_idx]), partial_rx_buf[channel].buf + partial_rx_buf[channel].offset, can_write_bytes);

            size_t elems_written = can_write_bytes / bytes_per_ele;
            read_elems[channel] += elems_written;

            if (!time_set) {
                first_sample = sample_counter[channel];
                time_set = true;
            }

            partial_rx_buf[channel].left -= can_write_bytes;
            partial_rx_buf[channel].offset += can_write_bytes;

            sample_counter[channel] += elems_written;
        }

        if (read_elems[channel] < elems_to_read) {
            need_more_data = true;
        }

        buf_idx++;
    }

    while (need_more_data) {
        uint32_t wait_ms = 0;
        auto time_remaining = timeout - std::chrono::system_clock::now();
        if (time_remaining > std::chrono::duration<int64_t>::zero()) {
            wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_remaining).count();
        }

        if (rx_dqbuf_multi(wait_ms)) {
            break;
        }

        // TODO: align buffers for each channel based on phy timer value

        need_more_data = false;
        buf_idx = 0;

        for (uint32_t channel : channels) {
            uint32_t rounding_ticks = phytimer_ticks_per_sample[channel] / 2;
            uint32_t samp_delta = (pending_rx_buf[channel]->phytimer - last_phytimer[channel] + rounding_ticks) /
                                  phytimer_ticks_per_sample[channel];
            last_phytimer[channel] = pending_rx_buf[channel]->phytimer;

            // tolerance of +- 64 samples to deal with phytimer jitter
            // note that phytimer is dequeue time, subject to interrupt servicing time
            // phytimer timestamp is at a variable offset from buffer start time
            if (samp_delta < RFNM_USB_RX_PACKET_ELEM_CNT - 64) {
                // samples were repeated (strange, shouldn't happen)
                sample_counter[channel] -= RFNM_USB_RX_PACKET_ELEM_CNT - samp_delta;
                spdlog::info("channel {} repeat {} samples", channel, RFNM_USB_RX_PACKET_ELEM_CNT - samp_delta);
            } else if (samp_delta > RFNM_USB_RX_PACKET_ELEM_CNT + 64) {
                // samples were lost
                sample_counter[channel] += samp_delta - RFNM_USB_RX_PACKET_ELEM_CNT;
                spdlog::info("channel {} skip {} samples", channel, samp_delta - RFNM_USB_RX_PACKET_ELEM_CNT);
            }

            if (!time_set) {
                first_sample = sample_counter[channel];
                time_set = true;
            }

            size_t overflowing_by_elems = 0;
            size_t can_copy_bytes = outbufsize;

            if (dc_correction[channel]) {
                // periodically recalibrate DC offset to account for drift
                if ((pending_rx_buf[channel]->usb_cc & 0xF) == 0) {
                    switch (dev.get_transport_status()->rx_stream_format) {
                    case LIBRFNM_STREAM_FORMAT_CS8:
                        measQuadDcOffset(reinterpret_cast<int8_t *>(pending_rx_buf[channel]->buf),
                                RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].i8, 0.1f);
                        break;
                    case LIBRFNM_STREAM_FORMAT_CS16:
                        measQuadDcOffset(reinterpret_cast<int16_t *>(pending_rx_buf[channel]->buf),
                                RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].i16, 0.1f);
                        break;
                    case LIBRFNM_STREAM_FORMAT_CF32:
                        measQuadDcOffset(reinterpret_cast<float *>(pending_rx_buf[channel]->buf),
                                RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].f32, 0.1f);
                        break;
                    }
                }

                switch (dev.get_transport_status()->rx_stream_format) {
                case LIBRFNM_STREAM_FORMAT_CS8:
                    applyQuadDcOffset(reinterpret_cast<int8_t *>(pending_rx_buf[channel]->buf),
                            RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].i8);
                    break;
                case LIBRFNM_STREAM_FORMAT_CS16:
                    applyQuadDcOffset(reinterpret_cast<int16_t *>(pending_rx_buf[channel]->buf),
                            RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].i16);
                    break;
                case LIBRFNM_STREAM_FORMAT_CF32:
                    applyQuadDcOffset(reinterpret_cast<float *>(pending_rx_buf[channel]->buf),
                            RFNM_USB_RX_PACKET_ELEM_CNT, dc_offsets[channel].f32);
                    break;
                }
            }

            if ((read_elems[channel] + RFNM_USB_RX_PACKET_ELEM_CNT) > elems_to_read) {

                overflowing_by_elems = (read_elems[channel] + RFNM_USB_RX_PACKET_ELEM_CNT) - elems_to_read;
                can_copy_bytes = outbufsize - (overflowing_by_elems * bytes_per_ele);
            }

            std::memcpy(((uint8_t*)buffs[buf_idx]) + (bytes_per_ele * read_elems[channel]),
                    pending_rx_buf[channel]->buf, can_copy_bytes);

            if (overflowing_by_elems) {
                std::memcpy(partial_rx_buf[channel].buf, (pending_rx_buf[channel]->buf + can_copy_bytes),
                        outbufsize - can_copy_bytes);
                partial_rx_buf[channel].left = outbufsize - can_copy_bytes;
                partial_rx_buf[channel].offset = 0;
            }

            read_elems[channel] += RFNM_USB_RX_PACKET_ELEM_CNT - overflowing_by_elems;
            sample_counter[channel] += RFNM_USB_RX_PACKET_ELEM_CNT - overflowing_by_elems;

            if (read_elems[channel] < elems_to_read) {
                need_more_data = true;
            }

            buf_idx++;
        }

        rx_qbuf_multi();
    }

    if (time_set) {
        timestamp_ns = (uint64_t)(first_sample * ns_per_sample[first_chan]);
        elems_read = read_elems[first_chan];
    } else {
        if (first_chan != SIZE_MAX) {
            timestamp_ns = (uint64_t)(sample_counter[first_chan] * ns_per_sample[first_chan]);
        } else {
            timestamp_ns = 0;
        }
        elems_read = 0;
    }

    return ret;
}

// only return data once a buffer has been dequeued from every active channel
// TODO: handle dropped/skipped buffers that could result in channel desync
rfnm_api_failcode rx_stream::rx_dqbuf_multi(uint32_t timeout_us) {
    rfnm_api_failcode ret = RFNM_API_OK;
    auto timeout = std::chrono::system_clock::now() + std::chrono::microseconds(timeout_us);

    for (uint32_t channel : channels) {
        if (pending_rx_buf[channel]) continue;

        uint32_t wait_ms = 0;
        auto time_remaining = timeout - std::chrono::system_clock::now();
        if (time_remaining > std::chrono::duration<int64_t>::zero()) {
            wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_remaining).count();
        }

        ret = dev.rx_dqbuf(&pending_rx_buf[channel], channel_flags[channel], wait_ms);
        if (ret) break;
    }

    return ret;
}

void rx_stream::rx_qbuf_multi() {
    for (uint32_t channel : channels) {
        if (pending_rx_buf[channel]) {
            dev.rx_qbuf(pending_rx_buf[channel]);
            pending_rx_buf[channel] = nullptr;
        }
    }
}

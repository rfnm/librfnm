#include <librfnm/rx_stream.h>
#include <librfnm/rfnm_fw_api.h>
#include <spdlog/spdlog.h>

using namespace rfnm;

MSDLL rx_stream::rx_stream(device &rfnm, uint8_t ch_ids) : dev(rfnm) {
    // iterate the full channel space: get_rx_channel_count() counts available channels,
    // which is not an index bound when availability is sparse
    for (uint32_t channel = 0; channel < MAX_RX_CHANNELS; channel++) {
        if (!(ch_ids & channel_flags[channel])) continue;
        channels.push_back(channel);
    }

    outbufsize = RFNM_USB_RX_PACKET_ELEM_CNT * dev.get_transport_status()->rx_stream_format;

    // legacy clock only used if the device timing anchor is unavailable
    ns_fallback = 1e9 / dev.get_hwinfo()->clock.samp_rate;
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

// samples -> ticks on this stream's exact R = r_num/r_den (128-bit intermediate)
uint64_t rx_stream::ticks_of_samples(uint64_t samples) {
    if (!timing_valid) {
        return samples;
    }
    return (uint64_t)(((unsigned __int128)samples * timing.r_num) / timing.r_den);
}

// extended tick of the next unconsumed sample in this channel's pending buffer
uint64_t rx_stream::pending_head_stamp_ext(uint32_t channel) {
    struct rx_buf *b = pending_rx_buf[channel];
    uint64_t buf_ext = dev.rx_tick_extend(b->phytimer, b->adc_id);
    return buf_ext + ticks_of_samples(b->elem_cnt - samples_left[channel]);
}

// Bring every channel to one common timeline point: the LATEST next-deliverable stamp
// across channels wins, channels behind it consume-and-drop their leading samples.
// Used for the stream start (initial=true, seeds t0) and for every gap resync (the
// timeline position jumps to the truth instead of pretending continuity).
rfnm_api_failcode rx_stream::align_channels(bool initial) {
    rfnm_api_failcode ret = RFNM_API_OK;

    if (!timing_valid) {
        gap_pending = false;
        return ret;
    }

    for (int pass = 0; pass < 16; pass++) {
        // every channel needs data on hand to compare stamps
        for (uint32_t channel : channels) {
            if (!pending_rx_buf[channel] || !samples_left[channel]) {
                ret = rx_dqbuf_multi(100000);
                if (ret) {
                    return ret;
                }
                break;
            }
        }

        uint64_t anchor = 0;
        for (uint32_t channel : channels) {
            uint64_t st = pending_head_stamp_ext(channel);
            if (st > anchor) {
                anchor = st;
            }
        }

        bool aligned = true;
        for (uint32_t channel : channels) {
            uint64_t st = pending_head_stamp_ext(channel);
            uint64_t behind_ticks = anchor - st;
            if (!behind_ticks) {
                continue;
            }
            uint64_t behind_samples = (uint64_t)(((unsigned __int128)behind_ticks * timing.r_den) / timing.r_num);
            if (!behind_samples) {
                continue;
            }
            if (behind_samples >= samples_left[channel]) {
                // whole buffer is pre-anchor: drop it and refetch on the next pass
                dev.rx_qbuf(pending_rx_buf[channel]);
                pending_rx_buf[channel] = nullptr;
                samples_left[channel] = 0;
                aligned = false;
            } else {
                samples_left[channel] -= (uint32_t)behind_samples;
            }
        }

        if (aligned) {
            if (initial) {
                t0_ext = anchor;
                pos_samples = 0;
            } else if (anchor >= t0_ext) {
                pos_samples = (uint64_t)(((unsigned __int128)(anchor - t0_ext) * timing.r_den) / timing.r_num);
            } else {
                // stamp regression across the resync (epoch flip re-based the extension):
                // keep the timeline monotonic by re-basing t0 so pos continues in place
                t0_ext = anchor - ticks_of_samples(pos_samples);
            }
            // re-seed the contiguity chains from the aligned point
            for (uint32_t channel : channels) {
                next_stamp_ext[channel] = pending_head_stamp_ext(channel) + ticks_of_samples(samples_left[channel]);
            }
            gap_pending = false;
            return RFNM_API_OK;
        }
    }

    // could not converge (pathological stamp churn) - resync again on the next read
    return RFNM_API_DQBUF_NO_DATA;
}

MSDLL rfnm_api_failcode rx_stream::align_to_tick(uint64_t tick, uint32_t timeout_us) {
    if (!stream_active || !timing_valid) {
        return RFNM_API_DQBUF_NO_DATA;
    }

    for (int pass = 0; pass < 4096; pass++) {
        bool need_data = false;
        for (uint32_t channel : channels) {
            if (!pending_rx_buf[channel] || !samples_left[channel]) {
                need_data = true;
            }
        }
        if (need_data) {
            rfnm_api_failcode ret = rx_dqbuf_multi(timeout_us);
            if (ret) {
                return ret;
            }
            continue;
        }

        bool aligned = true;
        for (uint32_t channel : channels) {
            uint64_t st = pending_head_stamp_ext(channel);
            if (st == tick) {
                continue;
            }
            if (st > tick) {
                return RFNM_API_SCHED_LATE;     // target already consumed: cannot rewind
            }
            uint64_t behind_ticks = tick - st;
            uint64_t behind_samples = (uint64_t)(((unsigned __int128)behind_ticks * timing.r_den) / timing.r_num);
            if (!behind_samples) {
                return RFNM_API_SCHED_MISALIGNED;   // tick falls between two samples
            }
            aligned = false;
            if (behind_samples >= samples_left[channel]) {
                dev.rx_qbuf(pending_rx_buf[channel]);
                pending_rx_buf[channel] = nullptr;
                samples_left[channel] = 0;
            } else {
                samples_left[channel] -= (uint32_t)behind_samples;
            }
        }

        if (aligned) {
            // jump the delivery timeline truthfully to the requested point (same
            // monotonicity rules as the gap resync above)
            if (tick >= t0_ext) {
                pos_samples = (uint64_t)(((unsigned __int128)(tick - t0_ext) * timing.r_den) / timing.r_num);
            } else {
                t0_ext = tick - ticks_of_samples(pos_samples);
            }
            for (uint32_t channel : channels) {
                next_stamp_ext[channel] = pending_head_stamp_ext(channel) + ticks_of_samples(samples_left[channel]);
            }
            gap_pending = false;
            return RFNM_API_OK;
        }
    }

    return RFNM_API_DQBUF_NO_DATA;
}

MSDLL rfnm_api_failcode rx_stream::start() {
    rfnm_api_failcode ret = RFNM_API_OK;

    // starting rx worker without any channels streaming will cause errors
    if (channels.size() == 0) return ret;

    // idempotent: this object already streams (also keeps rx_stream_count balanced)
    if (stream_active) return ret;

    dev.rx_work_start();
    stream_active = true;

    uint16_t apply_mask = 0;
    uint8_t chan_mask = 0;
    for (uint32_t channel : channels) {
        // a channel left RF_ON (a crashed client, or plain device state from a previous
        // session) is recoverable, not a conflict: re-own it. Refusing here made every
        // consumer learn the RF_OFF-then-apply dance by folklore.
        if (dev.get_rx_channel(channel)->enable != RFNM_CH_RF_OFF) {
            spdlog::warn("rx channel {} was already RF_ON at stream start - re-owning it", channel);
        }

        dev.set_rx_channel_status(channel, RFNM_CH_RF_ON, RFNM_CH_STREAM_AUTO, false);
        apply_mask |= rx_channel_apply_flags[channel];
        chan_mask |= channel_flags[channel];
    }

    // flush old junk before streaming new data
    ret = dev.rx_flush(20000, chan_mask);
    if (ret) goto error;

    // start ADCs
    ret = dev.apply(apply_mask);
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

    // the stream ack is live after the apply: R, tick rate and epoch for the exact
    // stamp-derived timeline (timestamps + gap handling). Without it (pathological),
    // fall back to the legacy counted clock.
    timing_valid = (dev.get_timing(DIR_RX, &timing) == RFNM_API_OK && timing.anchored &&
            timing.r_num > 0 && timing.r_den > 0 && timing.tick_hz > 0);
    if (!timing_valid) {
        spdlog::warn("rx timing anchor unavailable - timestamps fall back to a counted clock");
    }

    ret = align_channels(true);
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
        dev.set_rx_channel_status(channel, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO, false);
        apply_mask |= rx_channel_apply_flags[channel];
        chan_mask |= channel_flags[channel];
    }
    ret = dev.apply(apply_mask);

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

    // a gap was found on the previous read: realign every channel on the stamps and
    // jump the timeline before touching this read's data - its timestamp must be true
    if (gap_pending) {
        ret = align_channels(false);
        if (ret) {
            elems_read = 0;
            timestamp_ns = timing_valid ? dev.rx_tick_to_ns(ticks_of_samples(pos_samples))
                    : (uint64_t)(pos_samples * ns_fallback);
            return ret;
        }
    }

    timestamp_ns = timing_valid ? dev.rx_tick_to_ns(ticks_of_samples(pos_samples))
            : (uint64_t)(pos_samples * ns_fallback);

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
        if (samples_to_copy) {
            size_t buf_idx = 0;
            for (uint32_t channel : channels) {
                uint8_t *dst = reinterpret_cast<uint8_t *>(buffs[buf_idx++]) + read_elems * bytes_per_ele;
                size_t pkt_elems = pending_rx_buf[channel]->elem_cnt;
                uint8_t *src = pending_rx_buf[channel]->buf +
                    (pkt_elems - samples_left[channel]) * bytes_per_ele;
                std::memcpy(dst, src, samples_to_copy * bytes_per_ele);
                samples_left[channel] -= samples_to_copy;
            }
            read_elems += samples_to_copy;
            pos_samples += samples_to_copy;
        }

        if (need_more_data) {
            uint32_t wait_us = 0;
            auto time_remaining = timeout - std::chrono::system_clock::now();
            if (time_remaining > std::chrono::duration<int64_t>::zero()) {
                wait_us = std::chrono::duration_cast<std::chrono::microseconds>(time_remaining).count();
            }

            ret = rx_dqbuf_multi(wait_us);
            if (ret) break;

            // stamp chain broke: deliver what precedes the gap, resync on the next read.
            // Consumers see a short read with an exact timestamp on both sides - never
            // silently concatenated samples across missing data.
            if (gap_pending) {
                break;
            }
        }
    } while (need_more_data);

    elems_read = read_elems;

    // a short read at a gap is not an error - the samples delivered are good
    if (gap_pending && read_elems) {
        return RFNM_API_OK;
    }

    return ret;
}

rfnm_api_failcode rx_stream::rx_dqbuf_multi(uint32_t timeout_us, bool first) {
    rfnm_api_failcode ret = RFNM_API_OK;
    auto timeout = std::chrono::system_clock::now() + std::chrono::microseconds(timeout_us);

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

        // variable-size packets: the buffer declares how many samples it carries
        samples_left[channel] = pending_rx_buf[channel]->elem_cnt;

        // stamp-chain contiguity: any mismatch (device seam, transport hole, epoch flip)
        // is a timeline gap. The buffer is kept - it is the resync point; read() stops
        // in front of it and the next read realigns on it.
        if (!first && timing_valid) {
            uint64_t st = dev.rx_tick_extend(pending_rx_buf[channel]->phytimer, pending_rx_buf[channel]->adc_id);
            if (st != next_stamp_ext[channel]) {
                gap_pending = true;
            }
            next_stamp_ext[channel] = st + ticks_of_samples(pending_rx_buf[channel]->elem_cnt);
        }

        if (dc_correction[channel]) {
            size_t dc_elems = pending_rx_buf[channel]->elem_cnt;

            // periodically recalibrate DC offset to account for drift
            if ((pending_rx_buf[channel]->usb_cc & 0xF) == 0 || first) {
                float filter_factor = first ? 1.0f : 0.1f;

                switch (dev.get_transport_status()->rx_stream_format) {
                case STREAM_FORMAT_CS8:
                    measQuadDcOffset(reinterpret_cast<int8_t*>(pending_rx_buf[channel]->buf),
                        dc_elems * 2, dc_offsets[channel].i8, filter_factor);
                    break;
                case STREAM_FORMAT_CS16:
                    measQuadDcOffset(reinterpret_cast<int16_t*>(pending_rx_buf[channel]->buf),
                        dc_elems * 2, dc_offsets[channel].i16, filter_factor);
                    break;
                case STREAM_FORMAT_CF32:
                    measQuadDcOffset(reinterpret_cast<float*>(pending_rx_buf[channel]->buf),
                        dc_elems * 2, dc_offsets[channel].f32, filter_factor);
                    break;
                }
            }

            switch (dev.get_transport_status()->rx_stream_format) {
            case STREAM_FORMAT_CS8:
                applyQuadDcOffset(reinterpret_cast<int8_t*>(pending_rx_buf[channel]->buf),
                    dc_elems * 2, dc_offsets[channel].i8);
                break;
            case STREAM_FORMAT_CS16:
                applyQuadDcOffset(reinterpret_cast<int16_t*>(pending_rx_buf[channel]->buf),
                    dc_elems * 2, dc_offsets[channel].i16);
                break;
            case STREAM_FORMAT_CF32:
                applyQuadDcOffset(reinterpret_cast<float*>(pending_rx_buf[channel]->buf),
                    dc_elems * 2, dc_offsets[channel].f32);
                break;
            }
        }
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

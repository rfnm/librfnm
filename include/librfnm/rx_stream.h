#pragma once

#include "device.h"

namespace rfnm {
    union quad_dc_offset {
        int8_t i8[8];
        int16_t i16[8];
        float f32[8];
    };

    class rx_stream {
    public:
        MSDLL explicit rx_stream(device &rfnm, uint8_t ch_ids);
        MSDLL ~rx_stream();

        MSDLL rfnm_api_failcode start();
        MSDLL rfnm_api_failcode stop();

        MSDLL void set_auto_dc_offset(bool enabled, uint8_t ch_ids = 0xFF);

        MSDLL rfnm_api_failcode read(void * const * buffs, size_t elems_to_read,
            size_t &elems_read, uint64_t &timestamp_ns, uint32_t timeout_us = 20000);

        // "request x, get x" for stream consumers: consume-and-drop until the next
        // sample read() delivers is stamped exactly `tick` (extended ticks, same
        // domain as get_phytimer). Any offset, mid-session, sample-exact via the
        // stamps - packet boundaries never quantize delivery. SCHED_LATE = the tick
        // is already behind a channel's head (samples cannot be unread);
        // SCHED_MISALIGNED = the tick falls between two samples of this rate plan.
        MSDLL rfnm_api_failcode align_to_tick(uint64_t tick, uint32_t timeout_us = 100000);

    private:
        rfnm_api_failcode rx_dqbuf_multi(uint32_t timeout_us, bool first = false);
        void rx_qbuf_multi();
        rfnm_api_failcode align_channels(bool initial);
        uint64_t ticks_of_samples(uint64_t samples);
        uint64_t pending_head_stamp_ext(uint32_t channel);

        device &dev;

        std::vector<uint32_t> channels;
        size_t outbufsize = 0;
        bool stream_active = false;

        struct rx_buf * pending_rx_buf[MAX_RX_CHANNELS] = {};
        uint32_t samples_left[MAX_RX_CHANNELS] = {};

        // stamp-derived stream timeline (exact; replaces the old fabricated
        // sample-counter clock): t0_ext anchors sample 0, pos_samples is the next
        // sample to deliver, next_stamp_ext chains per-channel contiguity. A chain
        // mismatch marks gap_pending; read() finishes short and the next read()
        // realigns every channel on the stamps and jumps the timeline truthfully.
        struct rx_timing timing = {};
        bool timing_valid = false;
        uint64_t t0_ext = 0;
        uint64_t pos_samples = 0;
        uint64_t next_stamp_ext[MAX_RX_CHANNELS] = {};
        bool gap_pending = false;
        double ns_fallback = 0;

        bool dc_correction[MAX_RX_CHANNELS] = {false};
        union quad_dc_offset dc_offsets[MAX_RX_CHANNELS] = {};
    };
}

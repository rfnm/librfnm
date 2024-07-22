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

    private:
        rfnm_api_failcode rx_dqbuf_multi(uint32_t timeout_us, bool first = false);
        void rx_qbuf_multi();

        device &dev;

        std::vector<uint32_t> channels;
        size_t outbufsize = 0;
        bool stream_active = false;

        struct rx_buf * pending_rx_buf[MAX_RX_CHANNELS] = {};
        uint32_t samples_left[MAX_RX_CHANNELS] = {};

        int64_t sample_counter = 0;
        uint32_t last_phytimer[MAX_RX_CHANNELS] = {};
        double ns_per_sample;
        uint32_t phytimer_ticks_per_sample;

        bool dc_correction[MAX_RX_CHANNELS] = {false};
        union quad_dc_offset dc_offsets[MAX_RX_CHANNELS] = {};
    };
}

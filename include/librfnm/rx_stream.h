#pragma once

#include "device.h"

namespace rfnm {
    const size_t MAX_RX_CHAN_COUNT = 4;

    struct partial_buf {
        uint8_t* buf;
        uint32_t left;
        uint32_t offset;
    };

    union quad_dc_offset {
        int8_t i8[8];
        int16_t i16[8];
        float f32[8];
    };

    class rx_stream {
    public:
        MSDLL explicit rx_stream(device &rfnm, uint8_t ch_ids);
        MSDLL ~rx_stream();

        MSDLL rfnm_api_failcode activate();
        MSDLL rfnm_api_failcode deactivate();

        MSDLL void set_auto_dc_offset(bool enabled, uint8_t ch_ids = 0xFF);

        MSDLL rfnm_api_failcode read(void * const * buffs, size_t elems_to_read,
            size_t &elems_read, uint64_t &timestamp_ns, uint32_t timeout_us=20000);

    private:
        rfnm_api_failcode rx_dqbuf_multi(uint32_t timeout_us);
        void rx_qbuf_multi();

        device &dev;

        std::vector<unsigned int> channels;
        size_t outbufsize = 0;
        bool stream_active = false;

        struct rx_buf * pending_rx_buf[MAX_RX_CHAN_COUNT] = {};

        struct partial_buf partial_rx_buf[MAX_RX_CHAN_COUNT] = {};

        uint64_t sample_counter[MAX_RX_CHAN_COUNT] = {};
        double ns_per_sample[MAX_RX_CHAN_COUNT] = {};
        uint32_t last_phytimer[MAX_RX_CHAN_COUNT] = {};
        uint32_t phytimer_ticks_per_sample[MAX_RX_CHAN_COUNT] = {};

        bool dc_correction[MAX_RX_CHAN_COUNT] = {false};
        union quad_dc_offset dc_offsets[MAX_RX_CHAN_COUNT] = {};
    };
}

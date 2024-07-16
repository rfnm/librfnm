#pragma once

#include "librfnm.h"

#define MAX_RX_CHAN_COUNT 4

struct rfnm_partial_buf {
    uint8_t* buf;
    uint32_t left;
    uint32_t offset;
};

union rfnm_quad_dc_offset {
    int8_t i8[8];
    int16_t i16[8];
    float f32[8];
};

class rfnm_rx_stream {
public:
    MSDLL explicit rfnm_rx_stream(librfnm &rfnm, uint8_t ch_ids);
    MSDLL ~rfnm_rx_stream();

    MSDLL rfnm_api_failcode activate();
    MSDLL rfnm_api_failcode deactivate();

    MSDLL void set_auto_dc_offset(bool enabled, uint8_t ch_ids = 0xFF);

    MSDLL rfnm_api_failcode read(void * const * buffs, size_t elems_to_read,
        size_t &elems_read, uint64_t &timestamp_ns, uint32_t wait_for_ms=20);

private:
    rfnm_api_failcode rx_dqbuf_multi(uint32_t wait_for_ms);
    void rx_qbuf_multi();

    librfnm &lrfnm;

    std::vector<unsigned int> channels;
    size_t outbufsize = 0;

    struct librfnm_rx_buf * pending_rx_buf[MAX_RX_CHAN_COUNT] = {};

    struct rfnm_partial_buf partial_rx_buf[MAX_RX_CHAN_COUNT] = {};

    uint64_t sample_counter[MAX_RX_CHAN_COUNT] = {};
    double ns_per_sample[MAX_RX_CHAN_COUNT] = {};
    uint32_t last_phytimer[MAX_RX_CHAN_COUNT] = {};
    uint32_t phytimer_ticks_per_sample[MAX_RX_CHAN_COUNT] = {};

    bool dc_correction[MAX_RX_CHAN_COUNT] = {false};
    union rfnm_quad_dc_offset dc_offsets[MAX_RX_CHAN_COUNT] = {};
};

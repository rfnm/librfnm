#pragma once

#include <cstdint>

namespace rfnm {
    const uint16_t RFNM_USB_VID = 0x15A2;
    const uint16_t RFNM_USB_PID = 0x008C;
    const uint16_t RFNM_USB_PID_BOOST = 0x008D;

    const uint8_t RFNM_B_REQUEST = 100;

    const size_t THREAD_COUNT = 16;

    const size_t MIN_RX_BUFCNT = 1000;
    const size_t RX_RECOMB_BUF_LEN = 100;

    const uint32_t MAX_RX_CHANNELS = 8;
    const uint32_t MAX_TX_CHANNELS = 1;

    enum channel {
        CH0 = (0x1 << 0),
        CH1 = (0x1 << 1),
        CH2 = (0x1 << 2),
        CH3 = (0x1 << 3),
        CH4 = (0x1 << 4),
        CH5 = (0x1 << 5),
        CH6 = (0x1 << 6),
        CH7 = (0x1 << 7)
    };

    enum channel_apply {
        APPLY_CH0_TX = (0x1 << 0),
        APPLY_CH1_TX = (0x1 << 1),
        APPLY_CH2_TX = (0x1 << 2),
        APPLY_CH3_TX = (0x1 << 3),
        APPLY_CH4_TX = (0x1 << 4),
        APPLY_CH5_TX = (0x1 << 5),
        APPLY_CH6_TX = (0x1 << 6),
        APPLY_CH7_TX = (0x1 << 7),
        APPLY_CH0_RX = ((0x1 << 0) << 8),
        APPLY_CH1_RX = ((0x1 << 1) << 8),
        APPLY_CH2_RX = ((0x1 << 2) << 8),
        APPLY_CH3_RX = ((0x1 << 3) << 8),
        APPLY_CH4_RX = ((0x1 << 4) << 8),
        APPLY_CH5_RX = ((0x1 << 5) << 8),
        APPLY_CH6_RX = ((0x1 << 6) << 8),
        APPLY_CH7_RX = ((0x1 << 7) << 8)
    };

    static const uint8_t channel_flags[MAX_RX_CHANNELS] = {
        CH0,
        CH1,
        CH2,
        CH3,
        CH4,
        CH5,
        CH6,
        CH7
    };

    static const uint16_t rx_channel_apply_flags[MAX_RX_CHANNELS] = {
        APPLY_CH0_RX,
        APPLY_CH1_RX,
        APPLY_CH2_RX,
        APPLY_CH3_RX,
        APPLY_CH4_RX,
        APPLY_CH5_RX,
        APPLY_CH6_RX,
        APPLY_CH7_RX
    };

    static const uint16_t tx_channel_apply_flags[MAX_TX_CHANNELS] = {
        APPLY_CH0_TX
    };


    enum transport {
        TRANSPORT_LOCAL,
        TRANSPORT_USB,
        TRANSPORT_ETH,
        TRANSPORT_FIND
    };

    enum req_type {
        REQ_ALL = 0xff,
        REQ_TX = (0x1 << 1),
        REQ_RX = (0x1 << 2),
        REQ_HWINFO = (0x1 << 3),
        REQ_DEV_STATUS = (0x1 << 4),
    };

    enum debug_level {
        DEBUG_NONE = 0,
        DEBUG_ERROR = (0x1 << 1),
        DEBUG_INFO = (0x1 << 2),
        DEBUG_VERBOSE = (0x1 << 3),
    };

    enum stream_format {
        STREAM_FORMAT_CS8 = 2,
        STREAM_FORMAT_CS16 = 4,
        STREAM_FORMAT_CF32 = 8,
    };

    enum tx_latency_policy {
        // average number of tx buffers in the pipeline times two
        TX_LATENCY_POLICY_DEFAULT,
        // times 1.5
        TX_LATENCY_POLICY_AGGRESSIVE,
        // times 4
        TX_LATENCY_POLICY_RELAXED,
    };
}

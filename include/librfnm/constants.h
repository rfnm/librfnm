#pragma once

#include <cstdint>

namespace rfnm {
    const uint16_t RFNM_USB_VID = 0x15A2;
    const uint16_t RFNM_USB_PID = 0x008C;
    const uint16_t RFNM_USB_PID_BOOST = 0x008D;

    const uint8_t RFNM_B_REQUEST = 100;

    const size_t LIBRFNM_THREAD_COUNT = 16;

    const size_t LIBRFNM_MIN_RX_BUFCNT = 1000;
    const size_t LIBRFNM_RX_RECOMB_BUF_LEN = 100;

    const uint32_t MAX_RX_CHANNELS = 8;
    const uint32_t MAX_TX_CHANNELS = 1;

    enum channel {
        LIBRFNM_CH0 = (0x1 << 0),
        LIBRFNM_CH1 = (0x1 << 1),
        LIBRFNM_CH2 = (0x1 << 2),
        LIBRFNM_CH3 = (0x1 << 3),
        LIBRFNM_CH4 = (0x1 << 4),
        LIBRFNM_CH5 = (0x1 << 5),
        LIBRFNM_CH6 = (0x1 << 6),
        LIBRFNM_CH7 = (0x1 << 7)
    };

    enum channel_apply {
        LIBRFNM_APPLY_CH0_TX = (0x1 << 0),
        LIBRFNM_APPLY_CH1_TX = (0x1 << 1),
        LIBRFNM_APPLY_CH2_TX = (0x1 << 2),
        LIBRFNM_APPLY_CH3_TX = (0x1 << 3),
        LIBRFNM_APPLY_CH4_TX = (0x1 << 4),
        LIBRFNM_APPLY_CH5_TX = (0x1 << 5),
        LIBRFNM_APPLY_CH6_TX = (0x1 << 6),
        LIBRFNM_APPLY_CH7_TX = (0x1 << 7),
        LIBRFNM_APPLY_CH0_RX = ((0x1 << 0) << 8),
        LIBRFNM_APPLY_CH1_RX = ((0x1 << 1) << 8),
        LIBRFNM_APPLY_CH2_RX = ((0x1 << 2) << 8),
        LIBRFNM_APPLY_CH3_RX = ((0x1 << 3) << 8),
        LIBRFNM_APPLY_CH4_RX = ((0x1 << 4) << 8),
        LIBRFNM_APPLY_CH5_RX = ((0x1 << 5) << 8),
        LIBRFNM_APPLY_CH6_RX = ((0x1 << 6) << 8),
        LIBRFNM_APPLY_CH7_RX = ((0x1 << 7) << 8)
    };

    static const uint8_t channel_flags[MAX_RX_CHANNELS] = {
        LIBRFNM_CH0,
        LIBRFNM_CH1,
        LIBRFNM_CH2,
        LIBRFNM_CH3,
        LIBRFNM_CH4,
        LIBRFNM_CH5,
        LIBRFNM_CH6,
        LIBRFNM_CH7
    };

    static const uint16_t rx_channel_apply_flags[MAX_RX_CHANNELS] = {
        LIBRFNM_APPLY_CH0_RX,
        LIBRFNM_APPLY_CH1_RX,
        LIBRFNM_APPLY_CH2_RX,
        LIBRFNM_APPLY_CH3_RX,
        LIBRFNM_APPLY_CH4_RX,
        LIBRFNM_APPLY_CH5_RX,
        LIBRFNM_APPLY_CH6_RX,
        LIBRFNM_APPLY_CH7_RX
    };

    static const uint16_t tx_channel_apply_flags[MAX_TX_CHANNELS] = {
        LIBRFNM_APPLY_CH0_TX
    };


    enum transport {
        LIBRFNM_TRANSPORT_LOCAL,
        LIBRFNM_TRANSPORT_USB,
        LIBRFNM_TRANSPORT_ETH,
        LIBRFNM_TRANSPORT_FIND
    };

    enum req_type {
        LIBRFNM_REQ_ALL = 0xff,
        LIBRFNM_REQ_TX = (0x1 << 1),
        LIBRFNM_REQ_RX = (0x1 << 2),
        LIBRFNM_REQ_HWINFO = (0x1 << 3),
        LIBRFNM_REQ_DEV_STATUS = (0x1 << 4),
    };

    enum debug_level {
        LIBRFNM_DEBUG_NONE = 0,
        LIBRFNM_DEBUG_ERROR = (0x1 << 1),
        LIBRFNM_DEBUG_INFO = (0x1 << 2),
        LIBRFNM_DEBUG_VERBOSE = (0x1 << 3),
    };

    enum stream_format {
        LIBRFNM_STREAM_FORMAT_CS8 = 2,
        LIBRFNM_STREAM_FORMAT_CS16 = 4,
        LIBRFNM_STREAM_FORMAT_CF32 = 8,
    };

    enum tx_latency_policy {
        // average number of tx buffers in the pipeline times two
        LIBRFNM_TX_LATENCY_POLICY_DEFAULT,
        // times 1.5
        LIBRFNM_TX_LATENCY_POLICY_AGGRESSIVE,
        // times 4
        LIBRFNM_TX_LATENCY_POLICY_RELAXED,
    };
}

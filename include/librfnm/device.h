#pragma once

#include <queue>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <array>
#include <vector>

#include "rfnm_fw_api.h"

#if defined(__GNUC__)
#define MSDLL
#elif defined(_MSC_VER)
#define MSDLL __declspec(dllexport)
#endif

#define RFNM_USB_VID (0x15A2)
#define RFNM_USB_PID (0x8C)
#define RFNM_USB_PID_BOOST (0x8D)

#define RFNM_B_REQUEST (100)

#define LIBRFNM_THREAD_COUNT 16

#define LIBRFNM_MIN_RX_BUFCNT 1000
#define LIBRFNM_RX_RECOMB_BUF_LEN (100)

#define RFNM_MHZ_TO_HZ(MHz) (MHz * 1000 * 1000ul)
#define RFNM_HZ_TO_MHZ(Hz) (Hz / (1000ul * 1000ul))
#define RFNM_HZ_TO_KHZ(Hz) (Hz / 1000ul)

namespace rfnm {
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

    struct transport_status {
        enum transport transport;
        int usb_boost_connected;
        int theoretical_mbps;
        enum stream_format rx_stream_format;
        enum stream_format tx_stream_format;
        int boost_pp_tx;
        int boost_pp_rx;
    };

    struct status {
        struct transport_status transport_status;

        struct rfnm_dev_hwinfo hwinfo;
        struct rfnm_dev_tx_ch_list tx;
        struct rfnm_dev_rx_ch_list rx;

        struct rfnm_dev_status dev_status;

        std::chrono::time_point<std::chrono::high_resolution_clock> last_dev_time;
    };

    struct rx_buf {
        uint8_t* buf;
        uint32_t phytimer;
        uint32_t adc_cc;
        uint64_t usb_cc;
        uint32_t adc_id;
    };

    struct tx_buf {
        uint8_t* buf;
        uint32_t phytimer;
        uint32_t dac_cc;
        uint64_t usb_cc;
        uint32_t dac_id;
    };

    class rx_buf_compare {
    public:
        bool operator()(struct rx_buf* lra, struct rx_buf* lrb) {
            return (lra->usb_cc) > (lrb->usb_cc);
        }
    };

    struct rx_buf_s {
        std::queue<struct rx_buf*> in;
        std::priority_queue<struct rx_buf*, std::vector<struct rx_buf*>, rx_buf_compare> out[4];
        std::mutex in_mutex;
        std::mutex out_mutex;
        std::condition_variable cv;
        uint64_t usb_cc[4];
        uint64_t qbuf_cnt;

        uint64_t usb_cc_benchmark[4];
        std::mutex benchmark_mutex;
        uint8_t last_benchmark_adc;
    };

    struct tx_buf_s {
        std::queue<struct tx_buf*> in;
        std::queue<struct tx_buf*> out;
        std::mutex in_mutex;
        std::mutex out_mutex;
        //std::mutex cc_mutex;
        std::condition_variable cv;
        uint64_t usb_cc;
        uint64_t qbuf_cnt;
    };

    struct thread_data_s {
        int ep_id;
        int tx_active;
        int rx_active;
        int shutdown_req;
        std::condition_variable cv;
        std::mutex cv_mutex;
    };

    struct _usb_handle;

    class rx_stream;

    class device {
    public:
        MSDLL explicit device(enum transport transport, std::string address = "", enum debug_level dbg = LIBRFNM_DEBUG_NONE);
        MSDLL ~device();

        MSDLL static std::vector<struct rfnm_dev_hwinfo> find(enum transport transport, std::string address = "", int bind = 0);

        MSDLL rfnm_api_failcode get(enum req_type type);

        MSDLL rfnm_api_failcode set(uint16_t applies, bool confirm_execution = true, uint32_t timeout_us = 1000000);

        // Getters
        MSDLL const struct rfnm_dev_hwinfo * get_hwinfo();
        MSDLL const struct rfnm_dev_status * get_dev_status();
        MSDLL const struct transport_status * get_transport_status();
        MSDLL const struct rfnm_api_rx_ch * get_rx_channel(uint32_t channel);
        MSDLL const struct rfnm_api_tx_ch * get_tx_channel(uint32_t channel);

        // General setters
        MSDLL rfnm_api_failcode set_stream_format(enum stream_format format, size_t *bufsize);

        // RX channel setters
        MSDLL rfnm_api_failcode set_rx_channel_active(uint32_t channel, enum rfnm_ch_enable enable, enum rfnm_ch_stream stream, bool apply = true);
        MSDLL rfnm_api_failcode set_rx_channel_samp_freq_div(uint32_t channel, int16_t m, int16_t n, bool apply = true);
        MSDLL rfnm_api_failcode set_rx_channel_freq(uint32_t channel, int64_t freq, bool apply = true);
        MSDLL rfnm_api_failcode set_rx_channel_rfic_lpf_bw(uint32_t channel, int16_t bw, bool apply = true);
        MSDLL rfnm_api_failcode set_rx_channel_gain(uint32_t channel, int8_t gain, bool apply = true);
        // not exposing setter for rfic_dc_i and rfic_dc_q because that functionality will need to change
        // use the stream class for ADC interleaving aware DC offset removal instead
        MSDLL rfnm_api_failcode set_rx_channel_agc(uint32_t channel, enum rfnm_agc_type agc, bool apply = true);
        MSDLL rfnm_api_failcode set_rx_channel_bias_tee(uint32_t channel, enum rfnm_bias_tee bias_tee, bool apply = true);
        MSDLL rfnm_api_failcode set_rx_channel_path(uint32_t channel, enum rfnm_rf_path path, bool apply = true);
        // not exposing setter for data_type because this library only handles complex samples for now

        // TX channel setters
        MSDLL rfnm_api_failcode set_tx_channel_active(uint32_t channel, enum rfnm_ch_enable enable, enum rfnm_ch_stream stream, bool apply = true);
        MSDLL rfnm_api_failcode set_tx_channel_samp_freq_div(uint32_t channel, int16_t m, int16_t n, bool apply = true);
        MSDLL rfnm_api_failcode set_tx_channel_freq(uint32_t channel, int64_t freq, bool apply = true);
        MSDLL rfnm_api_failcode set_tx_channel_rfic_lpf_bw(uint32_t channel, int16_t bw, bool apply = true);
        MSDLL rfnm_api_failcode set_tx_channel_power(uint32_t channel, int8_t power, bool apply = true);
        MSDLL rfnm_api_failcode set_tx_channel_bias_tee(uint32_t channel, enum rfnm_bias_tee bias_tee, bool apply = true);
        MSDLL rfnm_api_failcode set_tx_channel_path(uint32_t channel, enum rfnm_rf_path path, bool apply = true);
        // not exposing setter for data_type because this library only handles complex samples for now

        // High level stream API
        MSDLL rx_stream * rx_stream_create(uint8_t ch_ids);

        // Low level RX stream API
        MSDLL rfnm_api_failcode rx_work_start();
        MSDLL rfnm_api_failcode rx_work_stop();
        MSDLL rfnm_api_failcode rx_qbuf(struct rx_buf* buf, bool new_buffer = false);
        MSDLL rfnm_api_failcode rx_dqbuf(struct rx_buf** buf, uint8_t ch_ids = 0, uint32_t timeout_us = 20000);
        MSDLL rfnm_api_failcode rx_flush(uint32_t timeout_us = 20000, uint8_t ch_ids = 0xFF);

        // Low level TX stream API
        MSDLL rfnm_api_failcode tx_work_start(enum tx_latency_policy policy = LIBRFNM_TX_LATENCY_POLICY_DEFAULT);
        MSDLL rfnm_api_failcode tx_work_stop();
        MSDLL rfnm_api_failcode tx_qbuf(struct tx_buf* buf, uint32_t timeout_us = 20000);
        MSDLL rfnm_api_failcode tx_dqbuf(struct tx_buf** buf);

        // RF path (antenna) name conversion
        MSDLL static enum rfnm_rf_path string_to_rf_path(std::string path);
        MSDLL static std::string rf_path_to_string(enum rfnm_rf_path path);

    private:
        void threadfn(size_t thread_index);

        MSDLL bool unpack_12_to_cs16(uint8_t* dest, uint8_t* src, size_t sample_cnt);
        MSDLL bool unpack_12_to_cf32(uint8_t* dest, uint8_t* src, size_t sample_cnt);
        MSDLL bool unpack_12_to_cs8(uint8_t* dest, uint8_t* src, size_t sample_cnt);
        MSDLL void pack_cs16_to_12(uint8_t* dest, uint8_t* src8, int sample_cnt);

        MSDLL int single_ch_id_bitmap_to_adc_id(uint8_t ch_ids);
        MSDLL void dqbuf_overwrite_cc(uint8_t adc_id, int acquire_lock);
        MSDLL int dqbuf_is_cc_continuous(uint8_t adc_id, int acquire_lock);

        _usb_handle *usb_handle = nullptr;

        std::mutex s_dev_status_mutex;
        std::mutex s_transport_pp_mutex;

        struct status* s = nullptr;

        struct rx_buf_s rx_s = {};
        struct tx_buf_s tx_s = {};
        struct thread_data_s thread_data[LIBRFNM_THREAD_COUNT] = {};

        std::array<std::thread, LIBRFNM_THREAD_COUNT> thread_c{};

        uint32_t cc_tx = 0;
        uint32_t cc_rx = 0;
        int last_dqbuf_ch = 0;

        int rx_stream_count = 0;
        bool rx_buffers_allocated = false;
        bool stream_format_locked = false;
    };
}

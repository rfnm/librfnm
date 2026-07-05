#pragma once

#include <queue>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <array>
#include <vector>

#include <asio.hpp>

#include "constants.h"
#include "rfnm_fw_api.h"

#if defined(__GNUC__)
#define MSDLL
#elif defined(_MSC_VER)
#define MSDLL __declspec(dllexport)
#endif

#ifdef BUILD_RFNM_LOCAL_TRANSPORT
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#endif

#define RFNM_MHZ_TO_HZ(MHz) (MHz * 1000 * 1000ul)
#define RFNM_HZ_TO_MHZ(Hz) (Hz / (1000ul * 1000ul))
#define RFNM_HZ_TO_KHZ(Hz) (Hz / 1000ul)

namespace rfnm {
    struct transport_status {
        enum transport transport;
        int usb_boost_connected;
        int theoretical_mbps;
        enum stream_format rx_stream_format;
        enum stream_format tx_stream_format;
        int boost_pp_tx;
        int boost_pp_rx;
    };

    // one discovered device, as returned by device::find(). Pass transport and address
    // to the device constructor to open this exact unit. The address is the USB serial
    // number for TRANSPORT_USB, the IPv4 address for TRANSPORT_TCP, and empty for
    // TRANSPORT_LOCAL.
    struct dev_info {
        enum transport transport;
        std::string address;
        struct rfnm_dev_hwinfo hwinfo;
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
        // exact tick of this buffer's first sample (61.44 MHz phy timer, u32 wrapping;
        // ticks per sample R = 2^dev_status.rx_r_shift / 2, valid within rx_flags' epoch)
        uint32_t phytimer;
        uint32_t rx_flags;  // RFNM_RX_FLAG_DISCONT + epoch[15:8]; was the dead adc_cc passthrough
        uint64_t usb_cc;
        uint32_t adc_id;
        // valid samples in buf (variable-size packets: the device flushes partial packets after a
        // latency deadline, so this can be < RFNM_USB_RX_PACKET_ELEM_CNT; buf is always allocated at max)
        uint32_t elem_cnt;
    };

    struct tx_buf {
        uint8_t* buf;
        uint32_t phytimer;
        uint32_t tx_flags;  // RFNM_TX_FLAG_* + epoch[15:8] (phase 3); was the never-populated dac_cc
        uint64_t usb_cc;
        uint32_t dac_id;
        // samples to send from buf: 0 = full packet (RFNM_USB_TX_PACKET_ELEM_CNT); otherwise a
        // multiple of 256 up to the full size. Small packets pace finer (latency), full packets
        // amortize per-transfer costs (throughput) - pick per use case.
        uint32_t elem_cnt;
    };

    // phytimer phase 2: the RX timing anchor (ticks-first). All ticks are EXTENDED
    // 64-bit phy timer ticks (the 32-bit counter unwrapped against the received
    // stream), meaningful within one epoch only. R = r_num/r_den ticks per sample,
    // exact - there is no tolerance anywhere in this surface.
    struct rx_timing {
        uint64_t t0;        // extended tick of the current epoch's first sample
        uint64_t tick_hz;   // phy timer rate for this clock plan (61.44 MHz canonical)
        uint32_t r_num;     // ticks per output sample = r_num / r_den
        uint32_t r_den;
        uint8_t epoch;      // stream generation; stamps only compare within one
        uint32_t regates;   // fw lane parks (overrun heals) this epoch
    };

    // phytimer phase 3: the TX timing anchor. Same tick domain as rx_timing - when
    // both directions apply together t0 is the SAME minted tick, so "tick T" means
    // one instant across RX and TX (the timing-advance primitive). Ring slot n airs
    // at t0 + n*256*(r_num/r_den) ticks; schedule with tx_buf_schedule().
    struct tx_timing {
        uint64_t t0;        // tick the DAC gate opened for this epoch (raw 32-bit, zero-extended)
        uint64_t tick_hz;
        uint32_t r_num;     // ticks per TX ring sample = r_num / r_den (exact)
        uint32_t r_den;
        uint8_t epoch;      // TX stream generation
        uint32_t underruns;     // fw-side DAC starvation events (anchor-health)
        uint32_t timed_rejects; // kernel-side scheduled-packet rejects (late/misaligned/rewind)
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

        uint64_t usb_cc_dropped[4];
        uint64_t usb_cc_ok[4];

        // phytimer phase 1 stamp-chain validation at the ordered dqbuf point: expected =
        // previous packet's stamp + elem_cnt x R. A break without a DISCONT/epoch resync
        // is a protocol error (phytimer_break); a flagged one is a clean device self-heal
        // (phytimer_discont). Chains re-anchor after any event so one break = one count.
        uint32_t expected_phytimer[4];
        uint8_t phytimer_epoch[4];
        bool phytimer_valid[4];
        uint64_t phytimer_discont[4];
        uint64_t phytimer_break[4];

        // phytimer phase 2: 32->64 bit tick extension, tracked at the same ordered
        // point (packets are monotonic per adc there). ext_high accumulates wraps
        // since the epoch anchor; ext_last is the newest raw stamp seen.
        uint64_t ext_high[4];
        uint32_t ext_last[4];
        bool ext_valid[4];
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

    struct ch_helper {
        uint8_t id;
        uint8_t mask; 
        uint16_t apply;
        uint8_t is_rx;
        uint8_t is_tx;
    };


    struct _usb_handle;

    class rx_stream;

    class device {
    public:
        MSDLL explicit device(enum transport transport, std::string address = "", enum debug_level dbg = DEBUG_NONE);
        MSDLL ~device();

        MSDLL static std::vector<struct dev_info> find(enum transport transport, std::string address = "");

        MSDLL rfnm_api_failcode get(enum req_type type);

        // ---- phytimer phase 2: ticks-first timing surface ----
        // Snapshot of the stream ack + clock plan. Valid after an RX apply.
        MSDLL rfnm_api_failcode get_rx_timing(struct rx_timing *t);
        // Unwrap a raw 32-bit stamp (rx_buf.phytimer) into extended ticks. Valid for
        // stamps within ~35 s of the newest dequeued packet, current epoch only.
        MSDLL uint64_t rx_tick_extend(uint32_t stamp, uint32_t adc_id = 0);
        // Exact conversions (128-bit internally; ns rounds to nearest).
        MSDLL uint64_t rx_tick_to_ns(uint64_t ticks);
        MSDLL uint64_t rx_ns_to_tick(uint64_t ns);
        // samples -> ticks via R; exact for every plan with r_shift >= 1 (rounds
        // down half a tick for odd sample counts on the 122.88M full-rate plan)
        MSDLL uint64_t rx_samples_to_ticks(uint64_t samples);
        // TDD pattern: gates + frontend flips on one M4 tick grid, sample-exact
        // stamps, one DISCONT per window boundary. period/duty in TICKS, both must
        // be multiples of the chunk (768 ADC samples; ticks per chunk from the
        // clock plan), duty < period, period >= ~4 ms (v1 scheduler floor).
        // LOCAL transport only in v1.
        MSDLL rfnm_api_failcode rx_tdd_configure(uint64_t period_ticks, uint64_t duty_ticks);
        MSDLL rfnm_api_failcode rx_tdd_stop();
        // Stamp-chain health since open: clean flagged jumps (window boundaries,
        // self-heals) vs unflagged breaks (data loss nothing accounted for - any
        // nonzero break count is a bug somewhere).
        MSDLL void get_rx_timing_health(uint64_t *disconts, uint64_t *breaks);

        // ---- phytimer phase 3: timed TX ----
        // Snapshot of the TX anchor. Refresh dev_status first (get(REQ_DEV_STATUS))
        // if no RX stream is running to refresh it for you.
        MSDLL rfnm_api_failcode get_tx_timing(struct tx_timing *t);
        // Mark buf to air its first sample at exactly `tick` (extended ticks, same
        // domain as rx stamps). Validates slot alignment (256 samples x R); the
        // kernel enforces the scheduling window (min ~64 slots lead, max ~ring span
        // ~= 68 ms at 61.44M) and zero-fills any gap, so silence between scheduled
        // bursts is automatic. Rejects are counted in tx_timing.timed_rejects -
        // never silent, never mis-timed.
        MSDLL rfnm_api_failcode tx_buf_schedule(struct tx_buf *buf, uint64_t tick);

        MSDLL rfnm_api_failcode apply(uint16_t applies, bool confirm_execution = true, uint32_t timeout_us = 1000000);

        // Getters
        MSDLL const struct rfnm_dev_hwinfo * get_hwinfo();
        MSDLL const struct rfnm_dev_status * get_dev_status();
        MSDLL const struct transport_status * get_transport_status();
        // cumulative RX packet counters since the last flush; a rising dropped count
        // means the transport cannot sustain the configured rate
        MSDLL void get_rx_stream_stats(uint64_t &ok, uint64_t &dropped);
        MSDLL const struct rfnm_api_rx_ch * get_rx_channel(uint32_t channel);
        MSDLL const struct rfnm_api_tx_ch * get_tx_channel(uint32_t channel);
        MSDLL uint32_t get_rx_channel_count();
        MSDLL uint32_t get_tx_channel_count();
        MSDLL bool is_rx_channel_available(uint32_t channel);
        MSDLL bool is_tx_channel_available(uint32_t channel);
        MSDLL rfnm_api_failcode control_transfer(enum rfnm_control_ep type, uint32_t size, uint8_t * buf, uint32_t timeout_ms);

        // General setters
        MSDLL rfnm_api_failcode set_stream_format(enum stream_format format, size_t* bufsize = 0, uint8_t *bytes_per_ele = 0);
        MSDLL rfnm_api_failcode set_samp_rate(uint64_t freq, uint32_t timeout_us = 20000);

        // Suggested RX RFIC LPF bandwidth in MHz (the unit set_rx_channel_rfic_lpf_bw takes)
        // for a sample rate: filter to the wanted band, capped at the effective ADC rate.
        // Pass the clock state from hwinfo when available - it is used when it corresponds
        // to the requested rate, otherwise the rate ladder predicts the ADC rate.
        MSDLL static int16_t suggested_lpf_bw(uint64_t samp_rate, const struct rfnm_dev_hwinfo_clock* clock = nullptr);

        // RX channel setters
        //MSDLL rfnm_api_failcode set_rx_channel_samp_freq_div(uint32_t channel, int16_t m, int16_t n, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_freq(uint32_t channel, int64_t freq, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_rfic_lpf_bw(uint32_t channel, int16_t bw, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_gain(uint32_t channel, int8_t gain, bool apply = false);
        // not exposing setter for rfic_dc_i and rfic_dc_q because that functionality will need to change
        // use the stream class for ADC interleaving aware DC offset removal instead
        MSDLL rfnm_api_failcode set_rx_channel_agc(uint32_t channel, enum rfnm_agc_type agc, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_fm_notch(uint32_t channel, enum rfnm_fm_notch fm_notch, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_bias_tee(uint32_t channel, enum rfnm_bias_tee bias_tee, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_path(uint32_t channel, enum rfnm_rf_path path, bool apply = false);
        // not exposing setter for data_type because this library only handles complex samples for now

        // TX channel setters
        //MSDLL rfnm_api_failcode set_tx_channel_samp_freq_div(uint32_t channel, int16_t m, int16_t n, bool apply = false);
        MSDLL rfnm_api_failcode set_tx_channel_freq(uint32_t channel, int64_t freq, bool apply = false);
        MSDLL rfnm_api_failcode set_tx_channel_rfic_lpf_bw(uint32_t channel, int16_t bw, bool apply = false);
        MSDLL rfnm_api_failcode set_tx_channel_power(uint32_t channel, int8_t power, bool apply = false);
        MSDLL rfnm_api_failcode set_tx_channel_bias_tee(uint32_t channel, enum rfnm_bias_tee bias_tee, bool apply = false);
        MSDLL rfnm_api_failcode set_tx_channel_path(uint32_t channel, enum rfnm_rf_path path, bool apply = false);
        // not exposing setter for data_type because this library only handles complex samples for now

        // High level stream API
        MSDLL rx_stream * rx_stream_create(uint8_t ch_ids);

        // Low level RX stream API
        MSDLL rfnm_api_failcode rx_work_start();
        MSDLL rfnm_api_failcode rx_work_stop();
        MSDLL rfnm_api_failcode rx_qbuf(struct rx_buf* buf, bool new_buffer = false);
        MSDLL rfnm_api_failcode rx_dqbuf(struct rx_buf** buf, uint8_t ch_ids = 0, uint32_t timeout_us = 20000);
        MSDLL rfnm_api_failcode rx_flush(uint32_t timeout_us = 20000, uint8_t ch_ids = 0xFF);
        MSDLL rfnm_api_failcode set_rx_channel_status(uint32_t channel, enum rfnm_ch_enable enable, enum rfnm_ch_stream stream, bool apply = false);

        // Low level TX stream API
        MSDLL rfnm_api_failcode tx_work_start(enum tx_latency_policy policy = TX_LATENCY_POLICY_DEFAULT);
        MSDLL rfnm_api_failcode tx_work_stop();
        MSDLL rfnm_api_failcode tx_qbuf(struct tx_buf* buf, uint32_t timeout_us = 20000);
        MSDLL rfnm_api_failcode tx_dqbuf(struct tx_buf** buf);
        MSDLL rfnm_api_failcode set_tx_channel_status(uint32_t channel, enum rfnm_ch_enable enable, enum rfnm_ch_stream stream, bool apply = false);

        // RF path (antenna) name conversion
        MSDLL static enum rfnm_rf_path string_to_rf_path(std::string path);
        MSDLL static std::string rf_path_to_string(enum rfnm_rf_path path);

        MSDLL static const char* failcode_to_string(rfnm_api_failcode code);

        size_t THREAD_COUNT = 16;


        inline ch_helper rx_ch_helper(uint8_t rx_ch_id) {
            ch_helper helper;
            helper.id = rx_ch_id;
            helper.mask = channel_flags[rx_ch_id];
            helper.apply = rx_channel_apply_flags[rx_ch_id];
            helper.is_rx = 1;
            helper.is_tx = 0;
            return helper;
        }

        inline ch_helper tx_ch_helper(uint8_t tx_ch_id) {
            ch_helper helper;
            helper.id = tx_ch_id;
            helper.mask = channel_flags[tx_ch_id];
            helper.apply = tx_channel_apply_flags[tx_ch_id];
            helper.is_rx = 0;
            helper.is_tx = 1;
            return helper;
        }

    private:
        void threadfn(size_t thread_index);

        MSDLL bool unpack_12_to_cs16(uint8_t* dest, uint8_t* src, size_t sample_cnt);
        MSDLL bool unpack_12_to_cf32(uint8_t* dest, uint8_t* src, size_t sample_cnt);
        MSDLL bool unpack_12_to_cs8(uint8_t* dest, uint8_t* src, size_t sample_cnt);
        MSDLL void pack_cs16_to_12(uint8_t* dest, uint8_t* src8, int sample_cnt);

        // local-transport cs16 payloads: format conversion without the 12-bit stage
        MSDLL bool convert_cs16_to_cs16(uint8_t* dest, uint8_t* src, size_t sample_cnt);
        MSDLL bool convert_cs16_to_cf32(uint8_t* dest, uint8_t* src, size_t sample_cnt);
        MSDLL bool convert_cs16_to_cs8(uint8_t* dest, uint8_t* src, size_t sample_cnt);

        MSDLL int single_ch_id_bitmap_to_adc_id(uint8_t ch_ids);
        MSDLL void dqbuf_overwrite_cc(uint8_t adc_id, int acquire_lock);
        MSDLL int dqbuf_is_cc_continuous(uint8_t adc_id, int acquire_lock);
        MSDLL void reorder_tx_queue_nolock(tx_buf_s &tx_s);

        MSDLL std::vector<uint64_t> get_retransmission_list(uint8_t adc_id);

        MSDLL void reset_device_state();

        

        _usb_handle *usb_handle = nullptr;
        int rfnm_ctrl_ep_ioctl;

        std::string rfnm_eth_transport_ip_addr;

        asio::ip::udp::endpoint rfnm_ctrl_ep_udp;
        std::unique_ptr<asio::io_context> rfnm_ctrl_ioctx_tcp;
        std::unique_ptr<asio::ip::udp::socket> rfnm_ctrl_socket_udp;


        std::unique_ptr<asio::ip::tcp::socket> rfnm_ctrl_socket_tcp;
        // Serializes a full control-socket transaction (SET write, or GET write+read).
        // Without it, the worker status poll and main-thread get/apply/set race on the
        // single TCP control socket and consume each other's responses.
        std::mutex rfnm_ctrl_socket_tcp_mutex;

        std::shared_ptr<asio::io_context> tcp_data_io_context;
        std::shared_ptr<asio::ip::tcp::socket> tcp_data_socket;
        std::mutex tcp_data_tx_mutex;
        std::mutex tcp_data_rx_mutex;
        std::atomic<bool> tcp_data_connected{ false };

        std::atomic<bool> usb_shutdown{ false };


        


        

        std::mutex s_dev_status_mutex;
        std::mutex s_transport_pp_mutex;

        struct status* s = nullptr;

        struct rx_buf_s rx_s = {};
        struct tx_buf_s tx_s = {};
        struct thread_data_s thread_data[MAX_THREAD_COUNT] = {};

        std::array<std::thread, MAX_THREAD_COUNT> thread_c{};

        uint32_t cc_tx = 0;
        uint32_t cc_rx = 0;
        uint32_t cc_samp_rate = 0;
        int last_dqbuf_ch = 0;

        int rx_stream_count = 0;
        bool rx_buffers_allocated = false;
        bool stream_format_locked = false;
    };

    std::string compute_broadcast_address(const std::string& ip_str, const std::string& mask_str);
    static std::vector<std::string> get_broadcast_addresses();
}

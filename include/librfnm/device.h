#pragma once

#include <queue>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <array>
#include <vector>
#include <atomic>
#include <chrono>
#include <cstdint>

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
        // USB RX dead-pipe watchdog telemetry. rx_dead_pipe_resync_cnt counts SET_INTERFACE
        // resyncs actually fired since open - every one rebuilt the endpoints and may have
        // destroyed in-flight packets, so any nonzero count during a session that should be
        // healthy is a real transport fault. rx_gap_suppressed_cnt counts silence episodes
        // on gated/scheduled sessions that the watchdog judged as honest device idle (the
        // device shipped nothing) and deliberately did NOT resync - benign, diagnostic only.
        uint32_t rx_dead_pipe_resync_cnt;
        uint32_t rx_gap_suppressed_cnt;
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

        // get_phytimer() unwrap state for stream-less clients (see the getter's contract)
        uint64_t ptmr_ext = 0;
        bool ptmr_ext_valid = false;

        // v4: last apply/samp-rate result (ecodes + reject fields) for get_apply_error()
        struct rfnm_dev_get_set_result_ext last_set_res = {};
        // v4: device-time extrapolation cache, stamped at every dev_status refresh
        // (under s_dev_status_mutex like the status itself)
        uint32_t ptmr_at_status = 0;
        // TX positional anchor the CURRENT feed session is pinned to - latched at
        // tx_feed_seek_to (or first stamped packet) under in_mutex+s_dev_status_mutex,
        // stamped from EXCLUSIVELY (never the live dev_status cache: the background
        // poller mutates it, and a re-mint mid-feed forked seek from stamps by
        // (t0_new - t0_old) mod 2^32 = the +880 ms wild-stamp bug). Lives in the
        // lib-allocated status block, NOT in device/tx_buf_s: old client binaries
        // allocate device with THEIR sizeof (ABI rule).
        uint32_t tx_anchor_t0 = 0;
        uint8_t tx_anchor_epoch = 0;
        uint8_t tx_anchor_r_shift = 0;
        bool tx_anchor_valid = false;
        std::chrono::time_point<std::chrono::steady_clock> host_at_status;
        bool ptmr_at_status_valid = false;
        // v4: TX headroom meter (sampled scans; plain ints - telemetry, single writer)
        int32_t tx_peak_abs = 0;
        uint64_t tx_headroom_scanned = 0;
        uint32_t tx_headroom_ctr = 0;
        bool tx_headroom_warned_low = false;
        bool tx_headroom_warned_sat = false;
        // wave 3: control-channel dead latch (TCP) - fail fast instead of a fresh
        // receive timeout per call; session-terminal, reopen recovers
        std::atomic<bool> tcp_ctrl_dead{ false };
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

        // freshest cc received per adc (maintained at the worker push). The consumer-lag
        // clamp jumps to this - the newest packet we HOLD - never to the device's producer
        // counter, which on a lossless transport runs ahead of data still in the pipe.
        uint64_t usb_cc_max_seen[4];

        // cc of the last DELIVERED packet per adc: a delivered-cc seam marks a transport
        // hole already accounted in usb_cc_dropped, so its stamp jump is not a "break"
        uint64_t delivered_cc[4];
        bool delivered_cc_valid[4];

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
        // positional free-run stamping (defect #20): cumulative samples fed since the
        // last TX-enabling apply(). tx_qbuf stamps every non-TIME_VALID packet with
        // tx_t0 + feed_pos*R (POS_VALID), so the device places it at its exact ring
        // slot - "feed position 0 airs at tx_t0" is a contract, not an accident.
        uint64_t feed_pos;
        // last successfully applied sample rate: the RX dead-pipe watchdog judges
        // "trickle-dead" (defect #18 residual) against it - 0 = unknown, watchdog
        // falls back to pure-silence semantics
        uint64_t samp_rate_hz;
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
        // clock plan), duty < period, period >= ~4 ms (v1 scheduler floor; a 1.25 ms
        // dark arm is proven clean). Reachable on EVERY transport (USB ep0 / TCP /
        // LOCAL). The pattern's PHASE is the stream anchor's: tile boundaries land at
        // anchor + k*period, so anchor_at() is how a pattern aligns to an external
        // timeline (e.g. NR DL slots). FE flips need RFNM_CH_RF_ON_TDD enables, RX
        // apply BEFORE TX apply (the RX half-profile parks the PA - that is the TDD
        // TX-desense fix).
        MSDLL rfnm_api_failcode rx_tdd_configure(uint64_t period_ticks, uint64_t duty_ticks);
        MSDLL rfnm_api_failcode rx_tdd_stop();
        // ---- exactness plan: the client-requested stream anchor ----
        // Request the stream anchor (rx_t0 AND tx_t0 - the hardware mints ONE shared
        // timeline) at `tick`. The gates open at the earliest achievable tick
        // CONGRUENT to the request: the tick itself when the lead suffices (~4 ms
        // guard + command transit), else advanced by whole TDD periods when a pattern
        // is armed / whole chunks otherwise - and every mid-session self-heal re-mint
        // preserves that congruence, so a requested phase holds for the session.
        // Sticky until the session's SM reset. Takes effect via an immediate re-gate
        // when a stream is running, else at the next apply. USB/TCP are
        // fire-and-forget: verify with get_rx_timing/get_tx_timing (t0 == request, or
        // congruent to it). Any tick is accepted - phase is the contract, not the
        // absolute first window.
        MSDLL rfnm_api_failcode anchor_at(uint64_t tick);
        inline rfnm_api_failcode rx_anchor_at(uint64_t tick) { return anchor_at(tick); }
        inline rfnm_api_failcode tx_anchor_at(uint64_t tick) { return anchor_at(tick); }
        // The feed contract, published by the kernel (never hardcode these): the TX
        // pump's enforced feed lead in phytimer ticks (rate-aware; OAI sl_ahead sanity)
        // and the partial-RX-packet flush deadline in us (bounds worst-case RX
        // delivery latency). Fetches a fresh dev_status.
        MSDLL rfnm_api_failcode get_feed_contract(uint32_t *tx_feed_lead_ticks, uint32_t *rx_flush_deadline_us);
        // v3 bundle: the HONEST usable write-lead minimum (placement guard + publish
        // staleness allowance) - budget from this, not the conservative advisory above.
        MSDLL rfnm_api_failcode get_feed_min_lead(uint32_t *tx_feed_min_lead_ticks);
        // v3 bundle: POS placement outcomes, device-side cumulative (snapshot at
        // session start, diff at the incident). Nonzero late/stale/misaligned deltas
        // during your session are YOUR dropped feed - the "client counters read clean
        // while the kernel dropped 92%" hole, closed.
        MSDLL rfnm_api_failcode get_tx_pos_stats(uint32_t *placed, uint32_t *late,
                uint32_t *stale, uint32_t *misaligned);
        // v3 bundle: the anchor congruence step (ticks) for the applied stream word -
        // consume this instead of re-deriving 384<<dcs from cached hwinfo.
        MSDLL rfnm_api_failcode get_anchor_step(uint32_t *step_ticks);

        // ---- v4 accessors ----
        // human-actionable message for the last apply()/set_samp_rate() rejection on a
        // channel: field + the requested value + the advertised range. Empty-ish text
        // when the channel carried no error.
        MSDLL std::string get_apply_error(uint8_t channel, bool tx);
        // "device time now" extrapolated from the last dev_status refresh (phytimer
        // domain, u32 wrap; no control round trip). False until a status was fetched.
        MSDLL bool get_device_time_approx(uint32_t *ticks);
        // tx_t0 - rx_t0 of the current anchors (one timeline when minted together);
        // NOT_SUPPORTED until both directions are anchored. Constant per apply today;
        // a true device constant arrives with the phytimer-parity fw.
        MSDLL rfnm_api_failcode get_tx_rx_anchor_offset(int32_t *offset_ticks);
        // cc of the most recent late/stale POS placement (as of last status refresh)
        MSDLL rfnm_api_failcode get_tx_last_late_cc(uint64_t *cc);
        // TX headroom meter: peak |sample| seen at the wire boundary + samples scanned
        // (sampled 1-in-16 packets). The 12-bit wire drops the low 4 bits - a peak far
        // below full scale means few effective bits on air.
        MSDLL void get_tx_headroom(int32_t *peak_abs, uint64_t *samples_scanned);

        // #19 pump-event telemetry: cumulative device-side self-heal counters (since
        // module load - snapshot at session start, diff at the incident). tx_pace_rolls
        // = mispaced-start regate requests from the kernel TX pace check; tx_arm_repairs
        // = self-heal re-applies that carried TX (client-invisible tx_t0/tx_epoch
        // re-mints + fw GO-detect repair opportunities). Refreshes dev_status like the
        // other timing getters, so the read never races a stale worker poll.
        MSDLL rfnm_api_failcode get_tx_pump_stats(uint32_t *tx_pace_rolls, uint32_t *tx_arm_repairs);
        // ---- v3: absolute-time scheduling (transport-general) ----
        // Requests are stamped with the ABSOLUTE phytimer tick at which they execute
        // (same domain as get_phytimer / the RX stamps), monotonic by tick, >= 150 us
        // ahead. Late or malformed requests fail loudly; the device never executes
        // late. Repetition is REFILL: keep pushing occurrences (patterns unroll
        // host-side). schedule_reset() drops everything queued. TX is scheduled by
        // writing TIME_VALID data packets (tx_qbuf with tx_flags/phytimer set) - the
        // packet IS the schedule; the device's request ring is internal.
        MSDLL rfnm_api_failcode schedule_reset();
        MSDLL rfnm_api_failcode schedule_rx(uint64_t tick, uint32_t len_samples, uint16_t tag = 0);
        // low-level escape hatch (bench tools, future FE flips): kind 1 = RX window,
        // 2 = TX slot window (internal payload binding), 3 = FE profile flip
        MSDLL rfnm_api_failcode schedule_ctl(uint64_t tick, uint8_t kind, uint16_t type,
                uint32_t len_samples, uint8_t flags = 0, uint32_t bind = 0);
        // Stamp-chain health since open: clean flagged jumps (window boundaries,
        // self-heals) vs unflagged breaks (data loss nothing accounted for - any
        // nonzero break count is a bug somewhere).
        MSDLL void get_rx_timing_health(uint64_t *disconts, uint64_t *breaks);

        // ---- phytimer phase 3: timed TX ----
        // Snapshot of the TX anchor. Refresh dev_status first (get(REQ_DEV_STATUS))
        // if no RX stream is running to refresh it for you.
        MSDLL rfnm_api_failcode get_tx_timing(struct tx_timing *t);
        // Current board time in extended ticks, NO stream required: the device
        // captures the phy timer on every dev-status read and this call fetches a
        // fresh status. Freshness = one status round trip (~ms class; the value is
        // a true board-side capture, the transit only adds age, never error). Same
        // tick domain as the RX stamps whenever an RX stream is live; otherwise
        // self-extended from this handle's first call (call at least once per ~35 s
        // to keep the unwrap exact). This is the "now" for tx_buf_schedule() -
        // TX-only clients must NOT open an RX stream just to read the clock (an RX
        // apply on a TX board steals the shared FE port).
        MSDLL rfnm_api_failcode get_phytimer(uint64_t *tick);
        // Mark buf to air its CONTENT first sample at exactly `tick` (extended ticks,
        // same domain as rx stamps) - any tick, no slot alignment required: a
        // sub-slot tick opens the gate at the slot boundary below (<= 255 samples
        // early, airing the zero pad) and the content lands at `tick` exactly
        // (pad-to-exact). Needs the buffer to have slack for the pad: a full packet
        // with a misaligned tick is rejected SCHED_MISALIGNED. The kernel enforces
        // the scheduling window (min ~64 slots lead, max ~ring span ~= 68 ms at
        // 61.44M) and zero-fills any gap, so silence between scheduled bursts is
        // automatic. Rejects are counted in tx_timing.timed_rejects - never silent,
        // never mis-timed.
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
        MSDLL rfnm_api_failcode set_rx_channel_freq(uint32_t channel, int64_t freq, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_rfic_lpf_bw(uint32_t channel, int16_t bw, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_gain(uint32_t channel, int8_t gain, bool apply = false);
        // not exposing setter for rfic_dc_i and rfic_dc_q because that functionality will need to change
        // use the stream class for ADC interleaving aware DC offset removal instead
        MSDLL rfnm_api_failcode set_rx_channel_agc(uint32_t channel, enum rfnm_agc_type agc, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_fm_notch(uint32_t channel, enum rfnm_fm_notch fm_notch, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_fe_mode(uint32_t channel, enum rfnm_rx_fe_mode fe_mode, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_bias_tee(uint32_t channel, enum rfnm_bias_tee bias_tee, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_path(uint32_t channel, enum rfnm_rf_path path, bool apply = false);
        // not exposing setter for data_type because this library only handles complex samples for now

        // TX channel setters
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
        // Force off every RX channel the device reports enabled (stale channels from a
        // killed client block stream creation; the mask names only enabled channels so
        // device-side apply validation passes). Call after open, before configuring your
        // own channels. Not automatic in rx_stream::start(): it would tear down channels
        // a concurrent stream on this device object is using.
        MSDLL rfnm_api_failcode rx_disable_stale_channels(uint32_t timeout_us = 20000000);

        // Low level TX stream API
        MSDLL rfnm_api_failcode tx_work_start(enum tx_latency_policy policy = TX_LATENCY_POLICY_DEFAULT);
        MSDLL rfnm_api_failcode tx_work_stop();
        MSDLL rfnm_api_failcode tx_qbuf(struct tx_buf* buf, uint32_t timeout_us = 20000);
        MSDLL rfnm_api_failcode tx_dqbuf(struct tx_buf** buf);
        // Advance the positional (POS_VALID) feed position by `samples` WITHOUT data:
        // the skipped span is scrubbed device-side (r4 gap scrub), the next tx_qbuf
        // stamps at the post-seek position. For sparse/late-starting feeders - a gap
        // wider than the transport lead can never be closed by feeding (the drain
        // consumes at line rate); it must be seeked over. Multiple of 256 only.
        MSDLL rfnm_api_failcode tx_feed_seek(uint64_t samples);
        // Absolute forward-only variant: the next tx_qbuf airs its first sample at
        // tx_t0 + abs_samples*R. Backward = RFNM_API_SCHED_ORDER; off-grid (not a
        // multiple of 256) = RFNM_API_NOT_SUPPORTED.
        MSDLL rfnm_api_failcode tx_feed_seek_to(uint64_t abs_samples);
        // Current feed position in samples (cumulative qbufs + seeks this session).
        MSDLL uint64_t tx_feed_pos();
        MSDLL rfnm_api_failcode set_tx_channel_status(uint32_t channel, enum rfnm_ch_enable enable, enum rfnm_ch_stream stream, bool apply = false);

        // RF path (antenna) name conversion
        MSDLL static enum rfnm_rf_path string_to_rf_path(std::string path);
        MSDLL static std::string rf_path_to_string(enum rfnm_rf_path path);

        MSDLL static const char* failcode_to_string(rfnm_api_failcode code);


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
        void tx_headroom_scan(const uint8_t* buf, size_t elems);  // v4 meter (worker thread)

        // local-transport cs16 payloads: format conversion without the 12-bit stage
        MSDLL bool convert_cs16_to_cs16(uint8_t* dest, uint8_t* src, size_t sample_cnt);
        MSDLL bool convert_cs16_to_cf32(uint8_t* dest, uint8_t* src, size_t sample_cnt);
        MSDLL bool convert_cs16_to_cs8(uint8_t* dest, uint8_t* src, size_t sample_cnt);

        MSDLL int single_ch_id_bitmap_to_adc_id(uint8_t ch_ids);
        void dqbuf_overwrite_cc_locked(uint8_t adc_id);
        MSDLL int dqbuf_is_cc_continuous(uint8_t adc_id, int acquire_lock);
        MSDLL void reorder_tx_queue_nolock(tx_buf_s &tx_s);

        MSDLL void reset_device_state();

        

        _usb_handle *usb_handle = nullptr;
        int rfnm_ctrl_ep_ioctl;

        std::string rfnm_eth_transport_ip_addr;

        // Native TCP socket handle (int fd on POSIX, SOCKET on Windows) widened to a plain
        // integer so this public header pulls in neither asio nor platform socket headers.
        // All socket operations live in the private layer src/net_socket.*; UINT64_MAX = not open.
        struct net_sock {
            uint64_t h = UINT64_MAX;
        };

        net_sock rfnm_ctrl_socket_tcp;
        // Serializes a full control-socket transaction (SET write, or GET write+read).
        // Without it, the worker status poll and main-thread get/apply/set race on the
        // single TCP control socket and consume each other's responses.
        std::mutex rfnm_ctrl_socket_tcp_mutex;

        net_sock tcp_data_socket;
        std::mutex tcp_data_tx_mutex;
        std::mutex tcp_data_rx_mutex;
        std::atomic<bool> tcp_data_connected{ false };

        std::atomic<bool> usb_shutdown{ false };

        // The client armed a gated/scheduled RX session (rx_tdd_configure or a schedule_rx
        // push): bulk-IN silence between windows is legitimate, so the USB RX dead-pipe
        // watchdog must judge the pipe by the device's published ship counters instead of
        // a flat silence timeout (see the watchdog in threadfn). Sticky for the device
        // lifetime by design: clearing it (e.g. on rx_tdd_stop) while ring windows are
        // still pending would re-open the spurious-resync window this exists to close,
        // and a scheduled-capable client keeps real-wedge recovery through the counter
        // check either way.
        std::atomic<bool> rx_scheduled_session{ false };
        // bumped by rx_work_start when the workers (re)activate: tells the USB RX engine
        // to re-arm its dead-pipe watchdog baselines for the new session
        std::atomic<uint32_t> rx_work_generation{ 0 };
        // defect #18: latched by the USB RX watchdog when SET_INTERFACE resyncs stop
        // helping (2 consecutive resyncs with zero progress after them). A resync can
        // only re-arm the endpoints BELOW a stopped board-side producer - hammering it
        // ESHUTDOWN-storms the gadget back into the same frozen state forever, so past
        // the budget the watchdog stops resyncing and fails LOUDLY instead: rx_dqbuf
        // returns RFNM_API_RX_PIPE_DEAD while this is set. Recovery is the client's
        // (reopen the device = SM reset re-rolls the board-side lottery); cleared by
        // rx_work_start so a torn-down-and-restarted session gets a fresh verdict.
        std::atomic<bool> rx_pipe_dead_latch{ false };


        


        

        std::mutex s_dev_status_mutex;
        std::mutex s_transport_pp_mutex;

        struct status* s = nullptr;

        struct rx_buf_s rx_s = {};
        struct tx_buf_s tx_s = {};
        // live worker count for this transport (<= MAX_THREAD_COUNT), set once in the ctor
        size_t THREAD_COUNT = 16;
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
}

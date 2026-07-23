#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "constants.h"
#include "rfnm_fw_api.h"
#include "version.h"

// API export decoration. The library is built with -fvisibility=hidden on GCC/Clang,
// so every public entry point must carry the default-visibility attribute. On MSVC the
// build relies on CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS (function-only API, no exported
// data), so the macro stays empty there — a consumer-side __declspec(dllexport) here
// would mis-declare the import direction.
#if defined(__GNUC__) || defined(__clang__)
#define MSDLL __attribute__((visibility("default")))
#else
#define MSDLL
#endif

#define RFNM_MHZ_TO_HZ(MHz) ((MHz) * 1000 * 1000ul)	// parens: MHZ_TO_HZ(f0 + 10) must expand to (f0 + 10) MHz
#define RFNM_HZ_TO_MHZ(Hz) ((Hz) / (1000ul * 1000ul))
#define RFNM_HZ_TO_KHZ(Hz) ((Hz) / 1000ul)

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

    struct rx_buf {
        uint8_t* buf;
        // exact tick of this buffer's first sample (phy timer, u32 wrapping; tick rate
        // = dcs_clk/2 from hwinfo - plan-dependent, NEVER a constant;
        // ticks per sample R = 2^dev_status.rx_r_shift / 2, valid within rx_flags' epoch)
        uint32_t phytimer;
        uint32_t rx_flags;  // RFNM_RX_FLAG_DISCONT + epoch[15:8]
        uint64_t usb_cc;
        uint32_t adc_id;
        // valid samples in buf (variable-size packets: the device flushes partial packets after a
        // latency deadline, so this can be < RFNM_USB_RX_PACKET_ELEM_CNT; buf is always allocated at max)
        uint32_t elem_cnt;
    };

    struct tx_buf {
        uint8_t* buf;
        uint32_t phytimer;
        uint32_t tx_flags;  // RFNM_TX_FLAG_* + epoch[15:8]
        uint64_t usb_cc;
        uint32_t dac_id;
        // samples to send from buf: 0 = full packet (RFNM_USB_TX_PACKET_ELEM_CNT); otherwise a
        // multiple of 256 up to the full size. Small packets pace finer (latency), full packets
        // amortize per-transfer costs (throughput) - pick per use case.
        uint32_t elem_cnt;
    };

    struct ch_helper {
        uint8_t id;
        uint8_t mask;
        uint16_t apply;
        uint8_t is_rx;
        uint8_t is_tx;
    };

    // ---- the v2 timing/health/contract surface (~/r/docs/librfnm-v2-p2-surface-2026-07-14.md) ----

    enum stream_dir {
        DIR_RX,
        DIR_TX,
    };

    // open() behavior flags (device constructor). OBSERVER attaches WITHOUT the
    // session-ownership SM reset - for diagnostic attach to a session another
    // process owns. Default open takes ownership (SM reset clears stale channel
    // enables, TDD patterns and anchors from a dead predecessor).
    enum open_flags : uint32_t {
        OPEN_DEFAULT  = 0,
        OPEN_OBSERVER = (1u << 0),
    };

    // ONE timing snapshot for either direction. All ticks are EXTENDED 64-bit phy
    // timer ticks, meaningful within one epoch; R = r_num/r_den ticks per sample,
    // exact - no tolerance anywhere in this surface. When both directions apply
    // together t0 is the SAME minted tick ("tick T" = one instant across RX and TX -
    // the timing-advance primitive; TX ring slot n airs at t0 + n*256*R, schedule
    // with tx_buf_schedule()). t0_ext extension rule, stated: while an RX stream is
    // (or was) live, BOTH directions extend into the RX stamps' 64-bit domain,
    // usable directly against rx stamps and clock_now(); with no RX extension yet,
    // t0_ext is the raw 32-bit anchor zero-extended.
    struct timing {
        uint64_t tick_hz;   // dcs_clk/2 for this clock plan (hwinfo-published); 0 = plan unknown
        uint32_t r_num;     // ticks per sample = r_num / r_den, exact, from the APPLIED stream word
        uint32_t r_den;
        uint64_t t0_ext;    // extended tick of this direction's epoch anchor
        uint32_t epoch;     // stream generation (u8 on the v5 wire; widens end-to-end at v6)
        bool anchored;      // epoch != 0: the anchor and R are live
    };

    // ONE health snapshot: every device-side counter in one status round trip, plus
    // the host-local stream counters (no extra RTT). Cumulative since module load
    // (device side) / since open (host side) - snapshot at session start, diff at the
    // incident.
    struct health {
        // rx device-side
        uint32_t rx_regates;            // fw lane parks (overrun heals) this epoch
        // tx device-side
        uint32_t tx_underruns;          // fw DAC starvation events
        uint32_t tx_timed_rejects;      // kernel scheduled-packet rejects
        uint32_t tx_pos_placed;         // POS placement outcomes (late/stale/misaligned = YOUR dropped feed)
        uint32_t tx_pos_late;
        uint32_t tx_pos_stale;
        uint32_t tx_pos_misaligned;
        uint32_t tx_pace_rolls;         // mispaced-start regate requests
        uint32_t tx_arm_repairs;        // self-heal re-applies that carried TX (client-invisible re-mints)
        uint64_t tx_last_late_cc;       // cc of the most recent late/stale placement
        uint32_t tx_ring_read_ptr;      // fw read position / kernel write head (ring slots):
        uint32_t tx_ring_head;          // a positional client's REAL margin is stamp-slot minus read_ptr
        // host-local
        uint64_t rx_stamp_disconts;     // clean flagged seams (window boundaries, self-heals)
        uint64_t rx_stamp_breaks;       // unflagged chain breaks - ANY nonzero count is a bug somewhere
        uint64_t rx_pkts_ok;
        uint64_t rx_pkts_dropped;       // rising = the transport cannot sustain the rate
        int32_t tx_headroom_peak_abs;   // peak |sample| at the wire (12-bit wire keeps bits [15:4])
        uint64_t tx_headroom_scanned;
    };

    // The device-published contract constants, one fetch (never hardcode these).
    struct feed_contract {
        uint32_t tx_feed_lead_ticks;    // the HONEST usable minimum TX write lead - budget from this
        uint32_t rx_flush_deadline_us;  // partial-RX-packet flush deadline (bounds delivery latency)
        uint32_t anchor_step_ticks;     // the anchor congruence step for the applied stream word
    };

    // Error classes over the v5 failcode values (the numbers ride the wire in apply
    // ecodes, so they stay pinned until the v6 break; the names and classes are
    // host-side). WOULD_BLOCK/NOT_SATISFIED below are the correct names for two
    // misnamed codes - the old names die at the P2 break wave.
    enum rfnm_error_class {
        ERR_OK,
        ERR_VALIDATION,             // the request itself is invalid (range, enum, plan)
        ERR_TRANSPORT_TERMINAL,     // the session is over; reopen (or upgrade) to recover
        ERR_FLOW,                   // backpressure / not-yet - retry is the contract
        ERR_SCHEDULE,               // scheduling contract violated (late, misaligned, order...)
        ERR_STATE,                  // session-state precondition missing (anchor, buffers)
    };

    const rfnm_api_failcode RFNM_API_MIN_QBUF_CNT_NOT_SATISFIED = RFNM_API_MIN_QBUF_CNT_NOT_SATIFIED;
    const rfnm_api_failcode RFNM_API_WOULD_BLOCK = RFNM_API_MIN_QBUF_QUEUE_FULL;

    inline enum rfnm_error_class failcode_class(rfnm_api_failcode c) {
        switch (c) {
        case RFNM_API_OK:
            return ERR_OK;
        case RFNM_API_PROBE_FAIL:
        case RFNM_API_TUNE_FAIL:
        case RFNM_API_GAIN_FAIL:
        case RFNM_API_NOT_SUPPORTED:
            return ERR_VALIDATION;
        case RFNM_API_USB_FAIL:
        case RFNM_API_RX_PIPE_DEAD:
        case RFNM_API_SW_UPGRADE_REQUIRED:
            return ERR_TRANSPORT_TERMINAL;
        case RFNM_API_TIMEOUT:
        case RFNM_API_DQBUF_OVERFLOW:
        case RFNM_API_DQBUF_NO_DATA:
        case RFNM_API_MIN_QBUF_QUEUE_FULL:      // = WOULD_BLOCK
            return ERR_FLOW;
        case RFNM_API_SCHED_NO_ANCHOR:
        case RFNM_API_SCHED_MISALIGNED:
        case RFNM_API_SCHED_LATE:
        case RFNM_API_SCHED_FULL:
        case RFNM_API_SCHED_ORDER:
        case RFNM_API_SCHED_NOT_RESET:
            return ERR_SCHEDULE;
        case RFNM_API_MIN_QBUF_CNT_NOT_SATIFIED:
        case RFNM_API_TX_NOT_ANCHORED:
        default:
            return ERR_STATE;
        }
    }

    class rx_stream;

    class device {
    public:
        // flags: enum open_flags (OPEN_OBSERVER = attach without the SM reset)
        MSDLL explicit device(enum transport transport, std::string address = "", uint32_t flags = OPEN_DEFAULT);
        MSDLL ~device();

        device(const device&) = delete;
        device& operator=(const device&) = delete;

        MSDLL static std::vector<struct dev_info> find(enum transport transport, std::string address = "");

        MSDLL rfnm_api_failcode get(enum req_type type);

        // ---- the ticks-first timing surface ----
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
        // clock plan), duty < period; TDD v2 floor: BOTH spans (duty and
        // period-duty) >= 600 us - sub-floor patterns are refused here
        // (synchronously, in this plan's tick rate) and in fw telemetry.
        // Reachable on EVERY transport (USB ep0 / TCP / LOCAL).
        // MODE WARNING: a pattern WITHOUT an armed schedule ring paces the TX drain
        // to the duty and re-mints the TX epoch every window - positional TX is
        // impossible there and the device REFUSES positional writes in that mode
        // (loud, counted). Port-TDD positional TX = ring + pattern. The pattern's
        // PHASE is the stream anchor's: tile boundaries land at anchor + k*period,
        // so anchor_at() is how a pattern aligns to an external timeline (e.g. NR DL
        // slots). FE flips need RFNM_CH_RF_ON_TDD enables, RX apply BEFORE TX apply
        // (the RX half-profile parks the PA - that is the TDD TX-desense fix).
        MSDLL rfnm_api_failcode rx_tdd_configure(uint64_t period_ticks, uint64_t duty_ticks);
        MSDLL rfnm_api_failcode rx_tdd_stop();
        // Gated-delivery lead projection, under the pattern THIS session armed (params
        // cached at rx_tdd_configure). The delivered clock (get_rx_delivered) advances
        // only during open spans - duty/period of wall time - so content meant to AIR
        // delivered_lead samples ahead of it must be written further ahead on the
        // line-rate feed axis: feed_lead = delivered_lead * period/duty (exact, floor;
        // 256-grid alignment stays the caller's). A stamp computed WITHOUT this
        // projection lands short by (period-duty)/duty x lead at air time.
        // AXIS RULE (the r12p2 overshoot receipt): delivered_lead must difference two
        // DELIVERED-clock values - folding in a line-rate counter (a ring produce
        // count, a wall clock) adds the session's whole accumulated deficit to the
        // "lead" and the projection overshoots, growing with session age.
        // No pattern known to this session -> SCHED_NO_ANCHOR, never a silent identity.
        MSDLL rfnm_api_failcode tdd_project_lead(uint64_t delivered_lead_samples, uint64_t *feed_lead_samples);
        // ---- the client-requested stream anchor ----
        // Request the stream anchor (rx_t0 AND tx_t0 - the hardware mints ONE shared
        // timeline) at `tick`. The gates open at the earliest achievable tick
        // CONGRUENT to the request: the tick itself when the lead suffices (~4 ms
        // guard + command transit), else advanced by whole TDD periods when a pattern
        // is armed / whole chunks otherwise - and every mid-session self-heal re-mint
        // preserves that congruence, so a requested phase holds for the session.
        // Sticky until the session's SM reset. Takes effect via an immediate re-gate
        // when a stream is running, else at the next apply. USB/TCP are
        // fire-and-forget: verify with get_timing (t0_ext == request, or congruent
        // to it). Any tick is accepted - phase is the contract, not the absolute
        // first window.
        MSDLL rfnm_api_failcode anchor_at(uint64_t tick);

        // human-actionable message for the last apply()/set_samp_rate() rejection on a
        // channel: field + the requested value + the advertised range. Empty-ish text
        // when the channel carried no error.
        MSDLL std::string get_apply_error(uint8_t channel, bool tx);

        // ---- the apply timing handle ----
        // (~/r/docs/phytimer-unified-contract-amendments-2026-07-13.md par.2/par.3)
        // Which timelines the last apply()/rate reconfig actually broke (bit0 rx,
        // bit1 tx, bit2 dcs_freq reclock - re-read tick_hz/r/anchor step after that
        // one) + the at-apply wire-epoch snapshots the settle test compares against.
        // A frequency- or gain-only apply reports timing_break == 0: the stream word
        // deduped, no timeline moved, timing stays trustworthy THROUGH the apply.
        MSDLL rfnm_api_failcode get_apply_timing(uint8_t *timing_break, uint32_t *rx_epoch_at_apply, uint32_t *tx_epoch_at_apply);
        // ONE status refresh + the advance-past test: true once every broken
        // direction's wire epoch moved PAST its snapshot (fresh anchor live). This
        // replaces every wait-for-epoch-change / warm-two-sessions ritual on both
        // families (Blue: usually true at first poll; geul: flips when the ~800 ms
        // async mint lands). Errors ride the status path's loud dead-latches - a dead
        // board fails this call, it never wedges the poll loop.
        MSDLL rfnm_api_failcode apply_timing_settled(bool *settled);

        // ---- the merged timing/health/contract getters (struct block above) ----
        // ONE timing snapshot per direction (one status refresh - the anchor-freshness
        // contract; never serves a stale cache). Returns OK with anchored=false while
        // the direction has no epoch yet - test t->anchored, not the failcode.
        MSDLL rfnm_api_failcode get_timing(enum stream_dir dir, struct timing *t);
        // Board time in extended ticks, no stream required. Freshness = one status
        // round trip (~ms class: a true board-side capture; transit adds age, never
        // error). Same domain as the RX stamps whenever an RX stream is live, else
        // self-extended from this handle's first call (call at least once per ~35 s).
        // This is the "now" for tx_buf_schedule()/schedule_rx() - TX-only clients
        // must NOT open an RX stream just to read the clock (an RX apply on a TX
        // board steals the shared FE port).
        MSDLL rfnm_api_failcode clock_now(uint64_t *tick_ext);
        // ---- time unification, one absolute anchor (~/r/docs/time-unification-plan-2026-07-17.md) ----
        // The LA9310 phytimer made absolute: (wrap_count << 32) | hw_low32, kernel-owned
        // extension, FRESH AT REQUEST TIME (one status round trip - never a cached copy).
        // Ticks compare only within one phy_gen; a gen move means every outstanding time
        // promise is void (TIME_RESET) - reopen and re-read (gen, now64).
        MSDLL rfnm_api_failcode get_phytimer64(uint64_t *tick64);
        MSDLL rfnm_api_failcode get_phy_gen(uint32_t *gen);
        // ONE health snapshot (one status refresh + the host-local counters).
        MSDLL rfnm_api_failcode get_health(struct health *h);
        // The device-published contract constants (one status refresh).
        MSDLL rfnm_api_failcode get_contract(struct feed_contract *c);

        // ---- absolute-time scheduling (transport-general) ----
        // Requests are stamped with the ABSOLUTE phytimer tick at which they execute
        // (same domain as clock_now / the RX stamps), monotonic by tick, >= 150 us
        // ahead. Late or malformed requests fail loudly; the device never executes
        // late. Repetition is REFILL: keep pushing occurrences (patterns unroll
        // host-side). schedule_reset() drops everything queued. TX is scheduled by
        // writing TIME_VALID data packets (tx_qbuf with tx_flags/phytimer set) - the
        // packet IS the schedule; the device's request ring is internal.
        MSDLL rfnm_api_failcode schedule_reset();
        MSDLL rfnm_api_failcode schedule_rx(uint64_t tick, uint32_t len_samples, uint16_t tag = 0);
        // Scheduled TX from RING-RESIDENT payload: air len_samples starting at DAC
        // ring slot dac_slot, gate-open at `tick` (same contract as schedule_rx:
        // absolute, monotonic, loud rejects). THE composable shape for schedules
        // that interleave RX windows and TX bursts in time: the request ring is
        // push-order monotonic ACROSS kinds, and TIME_VALID data-path packets land
        // with unbounded lag vs control-path pushes - submitting a time-interleaved
        // schedule over both paths WILL reject EINVAL. Fill the payload first
        // (free-run write = slot 0, or a positional feed), then push the whole mixed
        // schedule through this one ordered path.
        MSDLL rfnm_api_failcode schedule_tx_slot(uint64_t tick, uint32_t len_samples, uint32_t dac_slot);
        // The live delivered-sample <-> tick splice. Returns the cumulative samples
        // DELIVERED on this adc (in-order dequeues since open) and the extended tick
        // just past the last delivered sample - maintained at the ordered dequeue
        // point from the device's own stamps, so it tracks every gap and re-gate the
        // moment it is delivered. Convert any delivered index:
        //   tick(idx) = tick_at_delivered - rx_samples_to_ticks(delivered - idx)
        // NEVER derive ticks by counting samples against an init-time anchor (the
        // frozen t0 + count*R construction): under any gated delivery that map
        // inherits every closed span forever. Local state, no control round trip.
        // DQBUF_NO_DATA until a valid-epoch packet landed.
        // Under an armed TDD pattern this clock advances ONLY during open spans:
        // leads measured against it must be projected onto the feed axis before
        // writing ahead - see tdd_project_lead.
        MSDLL rfnm_api_failcode get_rx_delivered(uint64_t *delivered_samples,
                uint64_t *tick_at_delivered, uint32_t adc_id = 0);

        // ---- timed TX ----
        // Mark buf to air its CONTENT first sample at exactly `tick` (extended ticks,
        // same domain as rx stamps) - any tick, no slot alignment required: a
        // sub-slot tick opens the gate at the slot boundary below (<= 255 samples
        // early, airing the zero pad) and the content lands at `tick` exactly
        // (pad-to-exact). Needs the buffer to have slack for the pad: a full packet
        // with a misaligned tick is rejected SCHED_MISALIGNED. The kernel enforces
        // the scheduling window (min ~64 slots lead, max ~ring span) and zero-fills
        // any gap, so silence between scheduled bursts is automatic. Rejects are
        // counted in health.tx_timed_rejects - never silent, never mis-timed.
        MSDLL rfnm_api_failcode tx_buf_schedule(struct tx_buf *buf, uint64_t tick);

        MSDLL rfnm_api_failcode apply(uint16_t applies, bool confirm_execution = true, uint32_t timeout_us = 1000000);

        // Getters
        MSDLL const struct rfnm_dev_hwinfo * get_hwinfo();
        MSDLL const struct rfnm_dev_status * get_dev_status();
        MSDLL const struct transport_status * get_transport_status();
        MSDLL const struct rfnm_api_rx_ch * get_rx_channel(uint32_t channel);
        MSDLL const struct rfnm_api_tx_ch * get_tx_channel(uint32_t channel);
        MSDLL uint32_t get_rx_channel_count();
        MSDLL uint32_t get_tx_channel_count();
        MSDLL bool is_rx_channel_available(uint32_t channel);
        MSDLL bool is_tx_channel_available(uint32_t channel);

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
        // rfic_dc_i/rfic_dc_q have no setter: use the stream class for ADC
        // interleaving aware DC offset removal instead
        MSDLL rfnm_api_failcode set_rx_channel_agc(uint32_t channel, enum rfnm_agc_type agc, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_fm_notch(uint32_t channel, enum rfnm_fm_notch fm_notch, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_fe_mode(uint32_t channel, enum rfnm_rx_fe_mode fe_mode, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_bias_tee(uint32_t channel, enum rfnm_bias_tee bias_tee, bool apply = false);
        MSDLL rfnm_api_failcode set_rx_channel_path(uint32_t channel, enum rfnm_rf_path path, bool apply = false);

        // TX channel setters
        MSDLL rfnm_api_failcode set_tx_channel_freq(uint32_t channel, int64_t freq, bool apply = false);
        MSDLL rfnm_api_failcode set_tx_channel_rfic_lpf_bw(uint32_t channel, int16_t bw, bool apply = false);
        MSDLL rfnm_api_failcode set_tx_channel_power(uint32_t channel, int8_t power, bool apply = false);
        MSDLL rfnm_api_failcode set_tx_channel_bias_tee(uint32_t channel, enum rfnm_bias_tee bias_tee, bool apply = false);
        MSDLL rfnm_api_failcode set_tx_channel_path(uint32_t channel, enum rfnm_rf_path path, bool apply = false);

        // High level stream API
        MSDLL rx_stream * rx_stream_create(uint8_t ch_ids);

        // Low level RX stream API
        MSDLL rfnm_api_failcode rx_work_start();
        MSDLL rfnm_api_failcode rx_work_stop();
        MSDLL rfnm_api_failcode rx_qbuf(struct rx_buf* buf, bool new_buffer = false);
        MSDLL rfnm_api_failcode rx_dqbuf(struct rx_buf** buf, uint8_t ch_ids = 0, uint32_t timeout_us = 20000);
        MSDLL rfnm_api_failcode rx_flush(uint32_t timeout_us = 20000, uint8_t ch_ids = 0xFF);
        MSDLL rfnm_api_failcode set_rx_channel_status(uint32_t channel, enum rfnm_ch_enable enable, enum rfnm_ch_stream stream, bool apply = false);
        // DEPRECATED: the kernel clears stale channel enables at the session ownership
        // boundary (SM reset at open), so this ritual is unnecessary. Kept only until
        // the fleet-app rebuild wave deletes the last callers; new code must not call it.
        MSDLL rfnm_api_failcode rx_disable_stale_channels(uint32_t timeout_us = 20000000);

        // Low level TX stream API
        MSDLL rfnm_api_failcode tx_work_start(enum tx_latency_policy policy = TX_LATENCY_POLICY_DEFAULT);
        MSDLL rfnm_api_failcode tx_work_stop();
        MSDLL rfnm_api_failcode tx_qbuf(struct tx_buf* buf, uint32_t timeout_us = 20000);
        MSDLL rfnm_api_failcode tx_dqbuf(struct tx_buf** buf);
        // Advance the positional (POS_VALID) feed position by `samples` WITHOUT data:
        // the skipped span is scrubbed device-side, the next tx_qbuf stamps at the
        // post-seek position. For sparse/late-starting feeders - a gap wider than the
        // transport lead can never be closed by feeding (the drain consumes at line
        // rate); it must be seeked over. Multiple of 256 only.
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
        // ALL state and machinery live behind this one pointer (src/core/impl.h).
        // sizeof(device) is one pointer, permanently: client binaries built against
        // any header revision allocate compatibly, and internal state can never leak
        // back into the ABI.
        struct impl;
        impl *pimpl;
    };

    std::string compute_broadcast_address(const std::string& ip_str, const std::string& mask_str);
}

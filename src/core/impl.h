#pragma once

// The private half of rfnm::device (v2 P1). The public header carries ONE opaque
// pointer; every member and every piece of machinery lives here, grouped by role:
//
//  - session_state: everything with SESSION lifetime, created/reset at exactly one
//    point - device open, which is also the SM reset (impl::reset_session). Partial
//    mid-session invalidations (flush, apply-with-TX-break) are named session methods,
//    never field pokes scattered across call sites.
//  - rx_queue_s / tx_queue_s: the RX reorder/delivery queues and the TX feed queue.
//  - impl: cached device-published state, transport handles, the two workers.
//
// File split by role: core/{usb,tcp,local}.cpp own their transport (open/find/ctrl
// transact/worker passes), core/pump.cpp owns the shared worker loop + status poll,
// core/rx_queue.cpp the ordered RX delivery machinery, core/tx_queue.cpp the TX feed,
// api/*.cpp the public surface (thin bodies over impl).

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <librfnm/device.h>

#include "net_socket.h"
#include "timebase.h"

// TUP 3b (~/r/docs/tup-deletion-map-2026-07-17.md items 61/72): lib-internal marker for
// schedule-positional bufs - the stamp is already final: the tx worker must skip the
// anchor auto-stamp and feed_pos accounting (scheduled bursts are out-of-band vs the
// sequential feed). STRIPPED at the wire copy; never leaves the process. Bit 30 sits
// far above wire use (TIME_VALID/EOB/POS_VALID = bits 0-2, epoch byte = [15:8]).
#define RFNM_TX_FLAG_LIB_PRESTAMPED	(1u << 30)
// default OFF: today's TIME_VALID wire path stays byte-identical. Flip via env
// RFNM_TXSCHED_POSITIONAL=1 (read once per process) - the kernel timed lane retires
// only after the TA-sweep parity receipt on this lane.
static inline bool rfnm_txsched_positional(void) {
    static const bool en = [] {
        const char *e = getenv("RFNM_TXSCHED_POSITIONAL");
        return e && e[0] == '1';
    }();
    return en;
}


struct libusb_transfer;

namespace rfnm {

    // The real worker shape on EVERY transport: worker 0 = TX pump + status-poll
    // cadence owner, worker 1 = RX pump + status backstop.
    const size_t RFNM_WORKER_COUNT = 2;

    // USB async URB engines: pipelined transfers on ordered endpoints (one OUT endpoint
    // for TX, the four IN endpoints for RX). USB guarantees per-endpoint FIFO order, so
    // K outstanding transfers give line rate without reordering. Single owner thread per
    // engine; the state slots are atomic because BOTH workers pump the shared default
    // libusb context, so a completion can be delivered inside the other worker's
    // libusb_handle_events call - the callback's store needs a happens-before edge to
    // the owning worker's reads.
    const int RFNM_TX_ASYNC_URBS = 12;
    const int RFNM_RX_ASYNC_EPS = 4;
    const int RFNM_RX_ASYNC_URBS_PER_EP = 6;

    struct usb_tx_async_state {
        libusb_transfer* xfer[RFNM_TX_ASYNC_URBS];
        uint8_t* xbuf[RFNM_TX_ASYNC_URBS];
        struct tx_buf* app[RFNM_TX_ASYNC_URBS];
        // 0 = free, 1 = in flight, 2 = done-ok, 3 = done-error
        std::atomic<int> state[RFNM_TX_ASYNC_URBS];
        int initialized;
    };

    struct usb_rx_async_state {
        libusb_transfer* xfer[RFNM_RX_ASYNC_EPS][RFNM_RX_ASYNC_URBS_PER_EP];
        uint8_t* xbuf[RFNM_RX_ASYNC_EPS][RFNM_RX_ASYNC_URBS_PER_EP];
        // 0 unsubmitted, 1 in flight, 2 done, 3 failed
        std::atomic<int> state[RFNM_RX_ASYNC_EPS][RFNM_RX_ASYNC_URBS_PER_EP];
        // per-endpoint reaping is sequential so each endpoint's stream stays in
        // submission order; cross-endpoint skew is bounded by the pipeline depth and
        // handled by the cc reorder queue
        int next_reap[RFNM_RX_ASYNC_EPS];
        int initialized;
    };

    class rx_buf_compare {
    public:
        bool operator()(struct rx_buf* lra, struct rx_buf* lrb) {
            return (lra->usb_cc) > (lrb->usb_cc);
        }
    };

    // RX ordered-delivery state: the reorder queues and the per-adc cc/stamp-chain
    // bookkeeping maintained at the ordered dequeue point (core/rx_queue.cpp).
    struct rx_queue_s {
        std::queue<struct rx_buf*> in;
        std::priority_queue<struct rx_buf*, std::vector<struct rx_buf*>, rx_buf_compare> out[4];
        std::mutex in_mutex;
        std::mutex out_mutex;
        // pairs EXCLUSIVELY with out_mutex (a condition_variable waited on under two
        // different mutexes is UB - the worker-side buffer-starvation nap uses the
        // statically paired nap cv in core/pump.cpp instead)
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

        // stamp-chain validation at the ordered dqbuf point: expected = previous packet's
        // stamp + elem_cnt x R. A break without a DISCONT/epoch resync is a protocol error
        // (phytimer_break); a flagged one is a clean device self-heal (phytimer_discont).
        // Chains re-anchor after any event so one break = one count.
        uint32_t expected_phytimer[4];
        uint8_t phytimer_epoch[4];
        bool phytimer_valid[4];
        uint64_t phytimer_discont[4];
        uint64_t phytimer_break[4];
    };

    // TX feed queue: buffers in flight to the transport + recycled back to the client.
    struct tx_queue_s {
        std::queue<struct tx_buf*> in;
        std::queue<struct tx_buf*> out;
        std::mutex in_mutex;
        std::mutex out_mutex;
        std::condition_variable cv;
        uint64_t usb_cc;
        uint64_t qbuf_cnt;
    };

    struct worker_slot {
        int ep_id;
        int tx_active;
        int rx_active;
        int shutdown_req;
        std::condition_variable cv;
        std::mutex cv_mutex;
    };

    // Everything with SESSION lifetime. Created zeroed with the device and reset at
    // exactly one point: impl::reset_session(), called at open right after the SM
    // reset (the only SM-reset point in a device's life).
    struct session_state {
        // ---- TX feed axis (positional free-run) ----
        // cumulative samples fed since the last TX-timeline-breaking apply(). tx_qbuf
        // stamps every non-TIME_VALID packet with tx_t0 + feed_pos*R (POS_VALID), so
        // the device places it at its exact ring slot - "feed position 0 airs at
        // tx_t0" is a contract, not an accident.
        uint64_t feed_pos = 0;
        // the session declared positional intent (any tx_feed_seek/seek_to): from then
        // on a write that cannot be stamped POS_VALID is REFUSED
        // (RFNM_API_TX_NOT_ANCHORED), never silently demoted to free-run
        bool pos_intent = false;

        // ---- TX positional anchor latch ----
        // The anchor the CURRENT feed session is pinned to - latched ONCE (at
        // tx_feed_seek_to, or at the first stamped packet) under
        // in_mutex+s_dev_status_mutex, and NEVER re-read from the live dev_status
        // cache afterwards: the background poller mutates that cache, and a device
        // re-mint landing mid-feed would fork the seek basis from the stamp basis by
        // (t0_new - t0_old) mod 2^32. A re-mint instead makes these stamps honestly
        // stale-epoch (loud device drops, tx_pos_stale); the latch is invalidated by
        // an apply whose result says the TX timeline actually broke (v5 bit1).
        uint32_t tx_anchor_t0 = 0;
        uint8_t tx_anchor_epoch = 0;
        uint8_t tx_anchor_r_shift = 0;
        bool tx_anchor_valid = false;

        // ---- tick extension: the raw32<->extended owners (core/timebase.h) ----
        // per-adc stream extenders, advanced at the ordered dequeue point
        tick_extender rx_ext[4];
        // the stream-less get_phytimer unwrap (self-extending polled counter)
        tick_self_extender ptmr_ext;

        // ---- delivered<->tick splice (per adc) ----
        // delivered_samples counts every in-order dequeue (all epochs - the consumer's
        // delivered axis is cumulative); the end tick tracks the extension validity.
        // Written at the ordered dequeue point under rx out_mutex (readers get a
        // tear-free pair via get_rx_delivered).
        uint64_t delivered_samples[4] = {};
        uint64_t delivered_end_tick[4] = {};
        bool delivered_map_valid[4] = {};

        // ---- scheduled-session flag + watchdog session bookkeeping ----
        // The client armed a gated/scheduled RX session (rx_tdd_configure or a
        // schedule_rx push): bulk-IN silence between windows is legitimate, so the USB
        // RX dead-pipe watchdog judges the pipe by the device's published ship counters
        // instead of a flat silence timeout. Sticky for the device lifetime BY DESIGN:
        // clearing it (e.g. on rx_tdd_stop) while ring windows are still pending would
        // re-open the spurious-resync window this closes, and a scheduled-capable
        // client keeps real-wedge recovery through the counter check either way.
        std::atomic<bool> rx_scheduled_session{ false };
        // The pattern THIS session armed (rx_tdd_configure params, ticks) - the basis
        // for tdd_project_lead's period/duty ratio. Zero = no pattern known here
        // (a STANDING pattern armed by a previous session is deliberately not
        // guessed at - the projector refuses instead).
        uint64_t tdd_period_ticks = 0;
        uint64_t tdd_duty_ticks = 0;
        // bumped by rx_work_start when the workers (re)activate: tells the USB RX
        // engine to re-arm its dead-pipe watchdog baselines for the new session
        std::atomic<uint32_t> rx_work_generation{ 0 };
        // latched by the USB RX watchdog when SET_INTERFACE resyncs stop helping
        // (2 consecutive resyncs with zero progress after them): a resync can only
        // re-arm the endpoints BELOW a stopped board-side producer, so past the budget
        // the watchdog fails LOUDLY instead - rx_dqbuf returns RFNM_API_RX_PIPE_DEAD
        // while this is set. Recovery is the client's (reopen = SM reset re-rolls the
        // board-side lottery); cleared by rx_work_start so a torn-down-and-restarted
        // session gets a fresh verdict.
        std::atomic<bool> rx_pipe_dead_latch{ false };

        // ---- apply bookkeeping ----
        // last apply/samp-rate result (ecodes + reject fields + the v5 timing handle)
        // for get_apply_error/get_apply_timing/apply_timing_settled
        struct rfnm_dev_get_set_result_ext last_set_res = {};
        uint32_t cc_tx = 0;
        uint32_t cc_rx = 0;
        uint32_t cc_samp_rate = 0;
        // last successfully applied sample rate: the RX dead-pipe watchdog judges
        // "trickle-dead" against it - 0 = unknown, watchdog falls back to
        // pure-silence semantics
        uint64_t samp_rate_hz = 0;

        // full reset back to the freshly-constructed state (impl::reset_session is
        // the only caller - the one create/reset point)
        void reset() {
            feed_pos = 0;
            pos_intent = false;
            tx_anchor_t0 = 0;
            tx_anchor_epoch = 0;
            tx_anchor_r_shift = 0;
            tx_anchor_valid = false;
            for (int a = 0; a < 4; a++) {
                rx_ext[a].reset();
                delivered_samples[a] = 0;
                delivered_end_tick[a] = 0;
                delivered_map_valid[a] = false;
            }
            ptmr_ext.reset();
            rx_scheduled_session = false;
            tdd_period_ticks = 0;
            tdd_duty_ticks = 0;
            rx_work_generation = 0;
            rx_pipe_dead_latch = false;
            last_set_res = {};
            cc_tx = 0;
            cc_rx = 0;
            cc_samp_rate = 0;
            samp_rate_hz = 0;
        }
    };

    // per-worker pass context: staging buffers plus the USB engine + watchdog scratch
    // that belongs to exactly one worker (single writer: the owning worker thread)
    struct worker_ctx {
        size_t index = 0;
        uint8_t* lrxbuf_heap = nullptr;
        uint8_t* ltxbuf_heap = nullptr;
        struct rfnm_rx_usb_buf* lrxbuf = nullptr;   // local transport repoints this into the mmap'd pool per packet
        struct rfnm_tx_usb_buf* ltxbuf = nullptr;
        int transferred = 0;
        bool prev_tx_active = false;
        int status_fail_streak = 0;

        // USB engines (worker 0 owns az/TX, worker 1 owns arz/RX)
        usb_tx_async_state az = {};
        usb_rx_async_state arz = {};

        // USB RX dead-pipe watchdog scratch (worker 1). The judge inputs live in
        // session_state (rx_scheduled_session, rx_work_generation, samp_rate_hz);
        // this is the per-episode bookkeeping.
        int rx_reap_rr = 0;
        std::chrono::high_resolution_clock::time_point rx_last_progress;
        uint64_t wd_qbuf_acked[4] = {};
        uint32_t wd_generation = 0;
        bool wd_dead_pending = false;
        bool wd_gap_counted = false;
        std::chrono::high_resolution_clock::time_point wd_dead_since;
        uint32_t wd_resyncs_no_progress = 0;
        uint64_t wd_win_samples = 0;
        bool wd_rate_starved = false;
        std::chrono::high_resolution_clock::time_point wd_win_start;
        bool wd_win_started = false;

#ifdef BUILD_RFNM_LOCAL_TRANSPORT
        // zero-copy pool indices this pass holds (RX between GET and PUT, TX between
        // ACQ and SUB) + this worker's data-ep fd
        int data_ep_fp = -1;
        uint32_t local_rx_idx = 0;
        int local_rx_have = 0;
        uint32_t local_tx_idx = 0;
#endif
    };

    // worker pass outcomes: deliver = packet staged in ctx (run the shared deliver
    // tail); skip = nothing this pass (requeue the app buffer, move on); dead = the
    // transport declared this worker's stream over (exit the worker loop)
    enum class rx_pass_result {
        deliver,
        skip,
        dead,
    };

    // USB TX submit outcomes: again = retry the TX stage immediately (pipeline was
    // full, or slots remain and more input is queued); done = fall through to the
    // status poll; dead = the transport is gone (exit the worker loop)
    enum class tx_pass_result {
        again,
        done,
        dead,
    };

    struct _usb_handle;

    struct device::impl {
        // ---- lifecycle (api/device.cpp orchestrates; transports implement) ----
        // each open method fully cleans up its own partial state on failure and
        // returns false; the FIND transport falls through to the next one
        bool usb_open(const std::string &address);
        void usb_close();
        bool tcp_open(const std::string &address);
        void tcp_close();
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
        bool local_open();
#endif
        void start_workers();
        // signal + join with a bounded wait; true = a worker had to be detached and
        // the session state must be LEAKED deliberately (freeing it under a live
        // detached worker is a use-after-free)
        bool shutdown_workers();
        // THE one session create/reset point: called at open, right after the SM reset
        void reset_session();

        // ---- control plane ----
        rfnm_api_failcode ctrl_transfer(enum rfnm_control_ep type, uint32_t size, uint8_t *buf, uint32_t timeout_ms);
        rfnm_api_failcode usb_ctrl_transfer(enum rfnm_control_ep type, uint32_t size, uint8_t *buf, uint32_t timeout_ms);
        rfnm_api_failcode tcp_ctrl_transfer(enum rfnm_control_ep type, uint32_t size, uint8_t *buf, uint32_t timeout_ms);
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
        rfnm_api_failcode local_ctrl_transfer(enum rfnm_control_ep type, uint32_t size, uint8_t *buf, uint32_t timeout_ms);
#endif
        rfnm_api_failcode get(enum req_type type);
        // the scheduled-request verb (kind 1 = RX window, 2 = TX slot window, 3 = FE
        // flip, 4 = anchor intent); the device's request ring is an internal scheduler,
        // not a client concept
        rfnm_api_failcode schedule_ctl(uint64_t tick, uint8_t kind, uint16_t type,
                uint32_t len_samples, uint8_t flags, uint32_t bind);

        // ---- workers (core/pump.cpp) ----
        void worker(size_t index);
        void status_poll_pass(worker_ctx &c);
        bool rx_deliver(worker_ctx &c, struct rx_buf *buf);
        void reorder_tx_queue_nolock(tx_queue_s &q);
        void tx_headroom_scan(const uint8_t *buf, size_t elems);

        // ---- per-transport worker passes ----
        rx_pass_result usb_rx_pass(worker_ctx &c);
        tx_pass_result usb_tx_pass(worker_ctx &c, struct tx_buf *buf, uint32_t tx_wire_len);
        void usb_tx_drain_on_stop(worker_ctx &c);
        void usb_engine_teardown(worker_ctx &c);
        rx_pass_result tcp_rx_pass(worker_ctx &c);
        bool tcp_tx_send(worker_ctx &c, uint32_t tx_wire_len);
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
        rx_pass_result local_rx_fetch(worker_ctx &c);
        void local_rx_put(worker_ctx &c);
        bool local_tx_prepare(worker_ctx &c, struct tx_buf *buf, uint32_t tx_elems, uint32_t &tx_wire_len);
        void local_tx_send(worker_ctx &c);
#endif

        // ---- RX ordered delivery (core/rx_queue.cpp) ----
        rfnm_api_failcode rx_qbuf(struct rx_buf *buf, bool new_buffer);
        rfnm_api_failcode rx_dqbuf(struct rx_buf **buf, uint8_t ch_ids, uint32_t timeout_us);
        rfnm_api_failcode rx_flush(uint32_t timeout_us, uint8_t ch_ids);
        int dqbuf_is_cc_continuous(uint8_t adc_id, int acquire_lock);
        void dqbuf_overwrite_cc_locked(uint8_t adc_id);
        int single_ch_id_bitmap_to_adc_id(uint8_t ch_ids);

        // ---- TX feed (core/tx_queue.cpp) ----
        rfnm_api_failcode tx_qbuf(struct tx_buf *buf, uint32_t timeout_us);
        rfnm_api_failcode tx_dqbuf(struct tx_buf **buf);
        rfnm_api_failcode tx_feed_seek(uint64_t samples);
        rfnm_api_failcode tx_feed_seek_to(uint64_t abs_samples);
        uint64_t tx_feed_pos();

        // ================= state =================

        session_state ses;

        // cached device-published state (the worker status poll and get() refresh it;
        // the memcpy into dev_status and the freshness stamps are guarded by
        // s_dev_status_mutex - tx_qbuf stamps packets from this cache under the same
        // lock, so a torn tx_t0/tx_epoch pair at a self-heal re-mint is impossible)
        struct transport_status transport_status = {};

        // open() behavior (enum open_flags): set once in the constructor before any
        // transport opens; OPEN_OBSERVER suppresses the session-ownership SM reset
        uint32_t open_flags = 0;
        struct rfnm_dev_hwinfo hwinfo = {};
        struct rfnm_dev_tx_ch_list tx = {};
        struct rfnm_dev_rx_ch_list rx = {};
        struct rfnm_dev_status dev_status = {};
        std::chrono::time_point<std::chrono::high_resolution_clock> last_dev_time;
        // device-time extrapolation cache, stamped at every dev_status refresh
        uint32_t ptmr_at_status = 0;
        std::chrono::time_point<std::chrono::steady_clock> host_at_status;
        bool ptmr_at_status_valid = false;
        std::mutex s_dev_status_mutex;

        // TX headroom meter (sampled scans; plain ints - telemetry, single writer)
        int32_t tx_peak_abs = 0;
        uint64_t tx_headroom_scanned = 0;
        uint32_t tx_headroom_ctr = 0;
        bool tx_headroom_warned_low = false;
        bool tx_headroom_warned_sat = false;

        // queues
        rx_queue_s rx_s = {};
        tx_queue_s tx_s = {};

        // ---- transports ----
        _usb_handle *usb_handle = nullptr;
        int rfnm_ctrl_ep_ioctl = -1;
        std::string rfnm_eth_transport_ip_addr;
        uint64_t rfnm_ctrl_socket_tcp = net::NET_SOCK_INVALID;
        // Serializes a full control-socket transaction (SET write, or GET write+read).
        // Without it, the worker status poll and main-thread get/apply/set race on the
        // single TCP control socket and consume each other's responses.
        std::mutex rfnm_ctrl_socket_tcp_mutex;
        uint64_t tcp_data_socket = net::NET_SOCK_INVALID;
        std::mutex tcp_data_tx_mutex;
        std::mutex tcp_data_rx_mutex;
        std::atomic<bool> tcp_data_connected{ false };
        // control-channel dead latch (TCP): fail fast instead of a fresh receive
        // timeout per call; session-terminal, reopen recovers
        std::atomic<bool> tcp_ctrl_dead{ false };

        // ---- workers ----
        worker_slot thread_data[RFNM_WORKER_COUNT] = {};
        std::array<std::thread, RFNM_WORKER_COUNT> thread_c{};

        // ---- misc ----
        int last_dqbuf_ch = 0;
        int rx_stream_count = 0;
        bool rx_buffers_allocated = false;
        bool stream_format_locked = false;
    };

    // The RX worker's buffer-starvation nap. Statically paired cv (rx_s.cv keeps
    // exactly one mutex, out_mutex). Shared across device instances by design:
    // notifiers use notify_all and every waiter re-checks its own queue after the
    // bounded wait, so a cross-device wake is a benign early re-check.
    extern std::mutex rfnm_rx_in_nap_mutex;
    extern std::condition_variable rfnm_rx_in_nap_cv;

    // one device may enumerate at most once per transport (device.cpp; used by every
    // per-transport finder)
    bool sn_already_found(const std::vector<struct dev_info>& found, enum transport transport, const uint8_t* sn);

    // per-transport discovery (each appends to `found`, deduping via sn_already_found)
    void usb_find_into(std::vector<struct dev_info> &found, const std::string &address);
    void tcp_find_into(std::vector<struct dev_info> &found, const std::string &address);
#ifdef BUILD_RFNM_LOCAL_TRANSPORT
    void local_find_into(std::vector<struct dev_info> &found);
#endif

    // RFNM_DEBUG=stall,rx,cclog:<path> - THE debug knob (comma tokens; cclog carries
    // its output path after the colon). Parsed once at first use (pump.cpp), so hot
    // paths read a bool instead of walking environ per packet.
    struct debug_knobs {
        bool stall;
        bool rx;
        const char *cclog_path;     // nullptr = off
    };
    const debug_knobs &debug_knob();
}

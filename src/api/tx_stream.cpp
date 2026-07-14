// The library-owned TX feed pump (v2 P2). Composes PUBLIC device primitives only -
// this is the receipted consumer composition (OAI shim / vsgd / bootstrap_gate),
// owned once. Mechanics and constants mirror the shim's receipted pump: opportunistic
// dqbuf reclaim, bounded backpressure retries (loud drops past the budget), 256-grid
// staging with EOB short-flush, relative seeks for burst gaps, seek_to only at the
// session-start latch points.

#include <chrono>
#include <cstring>
#include <thread>

#include <spdlog/spdlog.h>

#include <librfnm/tx_stream.h>

#include "../core/impl.h"       // RFNM_TX_ASYNC_URBS: the transport's structural buffer hold
#include "../core/timebase.h"

using namespace rfnm;

namespace {
    // the shim's receipted backpressure budget: ~500 x (20 ms qbuf timeout + 200 us nap)
    const int QBUF_TRIES = 500;
    const int POOL_NAP_US = 200;
    const int PRIME_PACKETS = 10;   // virgin-pump prime (kernel ingest gate opens > 8 consumed)
}

struct tx_stream::impl {
    device &dev;
    enum tx_latency_policy policy;

    // buffer pool (stream-owned; in-flight bufs come back through tx_dqbuf)
    std::vector<struct tx_buf*> pool;
    std::vector<struct tx_buf*> free_q;
    size_t pool_total = 0;

    // staging (one packet)
    int16_t *stage = nullptr;
    uint32_t stage_cnt = 0;                 // samples staged
    uint32_t pkt_samples = RFNM_USB_TX_PACKET_ELEM_CNT;

    bool started = false;
    bool broken = false;                    // feed-axis rebase detected: loud until restarted
    uint64_t expected_dev_pos = 0;          // the lib's feed position after our last op

    // session timing (cached at start; a TX-breaking apply invalidates the session anyway)
    uint64_t tick_hz = 0;
    uint32_t r_num = 1, r_den = 2;

    // pacer (vsgd shape; default off)
    uint32_t pace_lead_us = 0;
    uint64_t pace_clock_ext = 0;            // cached clock_now + host stamp for cheap extrapolation
    std::chrono::steady_clock::time_point pace_clock_host;
    bool pace_clock_valid = false;

    tx_stream::stats st = {};

    explicit impl(device &d, size_t pool_packets, enum tx_latency_policy pol) : dev(d), policy(pol) {
        pool_total = pool_packets ? pool_packets : 1;
        for (size_t i = 0; i < pool_total; i++) {
            struct tx_buf *b = new tx_buf();
            memset(b, 0, sizeof(*b));
            b->buf = new uint8_t[(size_t)pkt_samples * 4];
            pool.push_back(b);
            free_q.push_back(b);
        }
        stage = new int16_t[(size_t)pkt_samples * 2];
    }

    void reclaim() {
        struct tx_buf *done;
        while (dev.tx_dqbuf(&done) == RFNM_API_OK) {
            free_q.push_back(done);
        }
    }

    // acquire a pool buffer, blocking on completions (stall episodes counted)
    struct tx_buf *acquire() {
        reclaim();
        bool stalled = false;
        while (free_q.empty()) {
            struct tx_buf *done;
            if (dev.tx_dqbuf(&done) == RFNM_API_OK) {
                free_q.push_back(done);
            } else {
                if (!stalled) {
                    stalled = true;
                    st.stalls++;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(POOL_NAP_US));
            }
        }
        struct tx_buf *b = free_q.back();
        free_q.pop_back();
        return b;
    }

    // qbuf with the receipted retry budget; on success track the lib axis and pace
    rfnm_api_failcode ship(struct tx_buf *b, uint32_t samples) {
        int tries = QBUF_TRIES;
        rfnm_api_failcode r;
        while ((r = dev.tx_qbuf(b, 20000)) != RFNM_API_OK && --tries) {
            if (r != RFNM_API_MIN_QBUF_QUEUE_FULL) {
                break;      // a non-flow failure never heals by retrying
            }
            st.stalls++;
            std::this_thread::sleep_for(std::chrono::microseconds(POOL_NAP_US));
        }
        if (r != RFNM_API_OK) {
            st.drops++;
            spdlog::error("tx_stream: qbuf stuck (code {}), packet dropped", (int)r);
            free_q.push_back(b);
            return r;
        }
        // adopt the lib's axis (the only truth): an UNSTAMPED packet (no anchor yet)
        // does not advance it, a stamped one advances it by `samples` - the lib knows,
        // we don't guess (the r9 shadow-fork class)
        (void)samples;
        expected_dev_pos = dev.tx_feed_pos();
        pace(b);
        return RFNM_API_OK;
    }

    // hold the feed lead near the budget (vsgd pacer, contract-derived): sleep the
    // excess of (queued stamp - clock now). Clock reads are throttled - between
    // refreshes "now" extrapolates from the host clock through the tick rate.
    void pace(struct tx_buf *b) {
        if (!pace_lead_us || !tick_hz || !(b->tx_flags & RFNM_TX_FLAG_POS_VALID)) {
            return;
        }
        timebase tb{ tick_hz };
        uint64_t budget = tb.us_to_ticks_floor(pace_lead_us);
        auto now_host = std::chrono::steady_clock::now();
        if (!pace_clock_valid ||
                std::chrono::duration_cast<std::chrono::milliseconds>(now_host - pace_clock_host).count() > 20) {
            uint64_t t = 0;
            if (dev.get_phytimer(&t)) {
                return;     // clock unreadable: never block the feed on telemetry
            }
            pace_clock_ext = t;
            pace_clock_host = std::chrono::steady_clock::now();
            pace_clock_valid = true;
            now_host = pace_clock_host;
        }
        uint64_t el_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now_host - pace_clock_host).count();
        uint64_t now_est = pace_clock_ext + tb.ns_to_ticks(el_ns);
        int32_t lead = (int32_t)(b->phytimer - (uint32_t)now_est);  // signed mod-2^32 distance
        if (lead > 0 && (uint64_t)lead > budget) {
            uint64_t excess_ns = tb.ticks_to_ns((uint64_t)lead - budget);
            std::this_thread::sleep_for(std::chrono::nanoseconds(excess_ns));
        }
    }

    // the ONLY truth is the lib's feed axis: it only ever moves FORWARD under a live
    // session (stamped ships, seeks) - backward motion means an apply() rebased it
    // (v5 TX break) and the session is over, loudly
    bool axis_ok() {
        if (broken) {
            return false;
        }
        if (dev.tx_feed_pos() < expected_dev_pos) {
            broken = true;
            st.rebases++;
            spdlog::error("tx_stream: feed axis rebased under the session (apply broke the TX timeline) - restart the stream");
            return false;
        }
        return true;
    }

    rfnm_api_failcode ship_full_from_stage() {
        struct tx_buf *b = acquire();
        memcpy(b->buf, stage, (size_t)pkt_samples * 4);
        b->phytimer = 0;
        b->tx_flags = 0;
        b->dac_id = 0;
        b->usb_cc = 0;
        b->elem_cnt = 0;    // full packet
        rfnm_api_failcode r = ship(b, pkt_samples);
        if (r == RFNM_API_OK) {
            st.packets++;
            stage_cnt = 0;
        }
        return r;
    }

    rfnm_api_failcode flush_short(bool eob) {
        if (!stage_cnt) {
            return RFNM_API_OK;
        }
        uint32_t padded = (stage_cnt + 255u) & ~255u;
        memset(stage + (size_t)stage_cnt * 2, 0, (size_t)(padded - stage_cnt) * 4);
        struct tx_buf *b = acquire();
        memcpy(b->buf, stage, (size_t)padded * 4);
        b->phytimer = 0;
        // EOB on a burst tail: the kernel scrub-ahead blanks past it immediately -
        // without it, low-lead inter-burst gaps air lap-old ring content (self-jam)
        b->tx_flags = eob ? RFNM_TX_FLAG_EOB : 0;
        b->dac_id = 0;
        b->usb_cc = 0;
        b->elem_cnt = padded;   // short packet
        rfnm_api_failcode r = ship(b, padded);
        if (r == RFNM_API_OK) {
            st.short_flushes++;
            stage_cnt = 0;
        }
        return r;
    }

    // append samples (zeros when iq == NULL) through the stage
    rfnm_api_failcode feed(const int16_t *iq, uint64_t nsamps, bool zeros) {
        while (nsamps) {
            uint32_t room = pkt_samples - stage_cnt;
            uint32_t n = (nsamps < room) ? (uint32_t)nsamps : room;
            if (zeros) {
                memset(stage + (size_t)stage_cnt * 2, 0, (size_t)n * 4);
            } else {
                memcpy(stage + (size_t)stage_cnt * 2, iq, (size_t)n * 4);
                iq += (size_t)n * 2;
            }
            stage_cnt += n;
            nsamps -= n;
            if (stage_cnt == pkt_samples) {
                rfnm_api_failcode r = ship_full_from_stage();
                if (r != RFNM_API_OK) {
                    return r;
                }
            }
        }
        return RFNM_API_OK;
    }

    // virgin-pump prime: fed content is what starts a cold fw pump (the kernel OUT
    // ingest opens on consumed packets). A PUBLISHED anchor proves the pump already
    // runs - then priming is both unnecessary and harmful (stamped at stale feed
    // positions = loud late drops), so it is strictly conditional.
    void prime_if_virgin() {
        struct tx_timing tt = {};
        if (dev.get_tx_timing(&tt) == RFNM_API_OK) {
            return;     // anchored = pump live
        }
        for (int i = 0; i < PRIME_PACKETS; i++) {
            struct tx_buf *b = acquire();
            memset(b->buf, 0, (size_t)pkt_samples * 4);
            b->phytimer = 0;
            b->tx_flags = 0;
            b->dac_id = 0;
            b->usb_cc = 0;
            b->elem_cnt = 0;
            if (ship(b, pkt_samples) == RFNM_API_OK) {
                st.primes++;
            }
        }
    }

    void adopt_axis() {
        expected_dev_pos = dev.tx_feed_pos();
        broken = false;
    }

    bool cache_timing() {
        struct tx_timing tt = {};
        if (dev.get_tx_timing(&tt) != RFNM_API_OK) {
            return false;
        }
        tick_hz = tt.tick_hz;
        r_num = tt.r_num;
        r_den = tt.r_den;
        return true;
    }
};

MSDLL tx_stream::tx_stream(device &dev, size_t pool_packets, enum tx_latency_policy policy) {
    pimpl = new impl(dev, pool_packets, policy);
}

MSDLL tx_stream::~tx_stream() {
    impl &m = *pimpl;
    // reclaim WHILE the workers still drain (queued packets go out at line rate and
    // their buffers come home) - a packet parked in a stopped TX queue can never be
    // reclaimed. The USB engine structurally holds up to its URB pipeline depth
    // until the NEXT session's wake (its drain-on-stop fires then), so the loop
    // exits once only that residue remains. Only then stop the workers.
    auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (m.free_q.size() + RFNM_TX_ASYNC_URBS < m.pool_total && std::chrono::steady_clock::now() < until) {
        m.reclaim();
        if (m.free_q.size() < m.pool_total) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    m.dev.tx_work_stop();
    m.reclaim();
    if (m.free_q.size() < m.pool_total) {
        // free only what came back; the rest stays alive for the transport (freeing
        // under a live completion is a use-after-free). Up to the URB pipeline depth
        // is the structural cost of a one-shot stream; more means a stuck queue.
        size_t missing = m.pool_total - m.free_q.size();
        if (missing > (size_t)RFNM_TX_ASYNC_URBS) {
            spdlog::warn("tx_stream: {} pool buffers still in flight at teardown (queue stuck?) - leaking them deliberately", missing);
        } else {
            spdlog::debug("tx_stream: {} buffers stay with the transport until the next session wake (structural URB hold)", missing);
        }
        for (struct tx_buf *b : m.free_q) {
            delete[] b->buf;
            delete b;
        }
    } else {
        for (struct tx_buf *b : m.pool) {
            delete[] b->buf;
            delete b;
        }
    }
    delete[] m.stage;
    delete pimpl;
}

MSDLL rfnm_api_failcode tx_stream::start_free_run() {
    impl &m = *pimpl;
    rfnm_api_failcode r = m.dev.tx_work_start(m.policy);
    if (r != RFNM_API_OK) {
        return r;
    }
    m.cache_timing();       // may be unanchored yet (free-run needs no anchor)
    m.adopt_axis();
    m.started = true;
    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode tx_stream::start_at(uint64_t abs_samples) {
    impl &m = *pimpl;
    rfnm_api_failcode r = m.dev.tx_work_start(m.policy);
    if (r != RFNM_API_OK) {
        return r;
    }
    if (!m.cache_timing()) {
        return RFNM_API_SCHED_NO_ANCHOR;    // positional start needs a published anchor
    }
    r = m.dev.tx_feed_seek_to(abs_samples); // latches the session anchor + declares intent
    if (r != RFNM_API_OK) {
        return r;
    }
    m.adopt_axis();
    m.started = true;
    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode tx_stream::start_earliest(uint64_t *pos_out, uint32_t margin_us, uint32_t anchor_timeout_ms) {
    impl &m = *pimpl;
    rfnm_api_failcode r = m.dev.tx_work_start(m.policy);
    if (r != RFNM_API_OK) {
        return r;
    }

    m.adopt_axis();
    m.prime_if_virgin();

    // wait for the anchor (VSPA pipeline start is ~250 ms after a cold arm)
    struct tx_timing tt = {};
    auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(anchor_timeout_ms);
    while ((r = m.dev.get_tx_timing(&tt)) != RFNM_API_OK) {
        if (std::chrono::steady_clock::now() >= until) {
            return RFNM_API_SCHED_NO_ANCHOR;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    m.tick_hz = tt.tick_hz;
    m.r_num = tt.r_num;
    m.r_den = tt.r_den;
    if (!m.tick_hz) {
        return RFNM_API_SCHED_NO_ANCHOR;    // no clock plan published: nothing to compute against
    }

    // earliest placeable position: now + max(margin, the device's published lead),
    // converted through the DERIVED ratio, 256-floor (the receipted composition)
    uint64_t now_ext = 0;
    if ((r = m.dev.get_phytimer(&now_ext))) {
        return r;
    }
    uint32_t dt = (uint32_t)now_ext - (uint32_t)tt.t0;      // raw-32, wrap-exact
    timebase tb{ m.tick_hz };
    uint64_t margin_ticks = tb.us_to_ticks_floor(margin_us ? margin_us : 2000);
    uint32_t lead = 0, flushdl = 0;
    if (!m.dev.get_feed_contract(&lead, &flushdl) && lead > margin_ticks) {
        margin_ticks = lead;
    }
    uint64_t airpos = ((((uint64_t)dt + margin_ticks) * tt.r_den) / tt.r_num) & ~255ull;

    uint64_t cur = m.dev.tx_feed_pos();
    if (airpos > cur) {
        r = m.dev.tx_feed_seek_to(airpos);  // latch + intent
    } else {
        // already past the earliest point (prime span, or a fresh anchor): the cursor
        // stays - declare positional intent without moving (a zero-length seek)
        airpos = cur;
        r = m.dev.tx_feed_seek(0);
    }
    if (r != RFNM_API_OK) {
        return r;
    }
    m.adopt_axis();
    m.started = true;
    if (pos_out) {
        *pos_out = airpos;
    }
    return RFNM_API_OK;
}

MSDLL void tx_stream::stop() {
    impl &m = *pimpl;
    m.dev.tx_work_stop();
    m.started = false;
}

MSDLL rfnm_api_failcode tx_stream::write(const int16_t *iq, size_t samples) {
    impl &m = *pimpl;
    if (!m.started || !m.axis_ok()) {
        return RFNM_API_TX_NOT_ANCHORED;
    }
    m.st.writes++;
    return m.feed(iq, samples, iq == nullptr);
}

MSDLL rfnm_api_failcode tx_stream::write_at(uint64_t abs_samples, const int16_t *iq, size_t samples) {
    impl &m = *pimpl;
    if (!m.started || !m.axis_ok()) {
        return RFNM_API_TX_NOT_ANCHORED;
    }
    m.st.writes++;
    uint64_t cur = m.dev.tx_feed_pos() + m.stage_cnt;   // live axis + staged: the only cursor
    if (abs_samples < cur) {
        return RFNM_API_SCHED_ORDER;        // samples cannot be unwritten
    }
    if (abs_samples > cur) {
        // burst gap: ship the previous tail (EOB) first, then seek the 256-grid part
        // and zero-fill the remainder - sample-exact positioning on a 256-grid seek
        rfnm_api_failcode r = m.flush_short(true);
        if (r != RFNM_API_OK) {
            return r;
        }
        cur = m.dev.tx_feed_pos();          // the tail pad advanced the axis (up to 255 samples)
        if (abs_samples < cur) {
            return RFNM_API_SCHED_ORDER;    // the target fell inside the pad span
        }
        uint64_t gap = abs_samples - cur;
        uint64_t seek = gap & ~255ull;
        if (seek) {
            r = m.dev.tx_feed_seek(seek);   // relative: the session anchor stays latched
            if (r != RFNM_API_OK) {
                return r;
            }
            m.expected_dev_pos = m.dev.tx_feed_pos();
            m.st.seeks++;
        }
        uint64_t zfill = gap - seek;
        if (zfill) {
            r = m.feed(nullptr, zfill, true);
            if (r != RFNM_API_OK) {
                return r;
            }
            m.st.zero_fill_samples += zfill;
        }
    }
    return m.feed(iq, samples, iq == nullptr);
}

MSDLL rfnm_api_failcode tx_stream::flush(bool end_of_burst) {
    impl &m = *pimpl;
    if (!m.started || !m.axis_ok()) {
        return RFNM_API_TX_NOT_ANCHORED;
    }
    return m.flush_short(end_of_burst);
}

MSDLL uint64_t tx_stream::pos() {
    impl &m = *pimpl;
    return m.dev.tx_feed_pos() + m.stage_cnt;
}

MSDLL void tx_stream::set_pace_lead_us(uint32_t lead_us) {
    pimpl->pace_lead_us = lead_us;
}

MSDLL void tx_stream::get_stats(stats *out) {
    if (out) {
        *out = pimpl->st;
    }
}

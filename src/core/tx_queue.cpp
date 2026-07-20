// TX feed: client writes with positional (POS_VALID) stamping against the session's
// latched anchor, feed-axis seeks, and buffer recycling.

#include <spdlog/spdlog.h>

#include "impl.h"

using namespace rfnm;

rfnm_api_failcode device::impl::tx_qbuf(struct tx_buf* buf, uint32_t timeout_us) {
    // tx_qbuf never blocks: the signed-window judge below either admits the packet or
    // returns MIN_QBUF_QUEUE_FULL for the caller to retry. The timeout parameter stays
    // for API stability (v4 surface) and as the reserved slot for a blocking variant.
    (void)timeout_us;
    // time unification (v5-twin parity): the first-stamp anchor latch below pins
    // dev_status.tx_t0 - refresh before locking so a seek-less session latches a
    // fresh observation, not the poller's cache (a stale now64 would anchor the
    // whole continuous stream behind the device's now64 judge). Unlocked peek =
    // heuristic (a miss falls back to the cache); fires at most once per session.
    if (!ses.tx_anchor_valid && !(buf->tx_flags & (RFNM_TX_FLAG_TIME_VALID | RFNM_TX_FLAG_LIB_PRESTAMPED))) {
        get(REQ_DEV_STATUS);
    }
    // elem_cnt selects the packet size: 0 = full, else a multiple of 256 samples (the
    // base-buf granularity the device consumes)
    if (buf->elem_cnt && (buf->elem_cnt % (LA_TX_BASE_BUFSIZE_12 / 3) || buf->elem_cnt > RFNM_USB_TX_PACKET_ELEM_CNT)) {
        return RFNM_API_NOT_SUPPORTED;
    }

    std::lock_guard<std::mutex> lockGuard1(s_dev_status_mutex);

    // SIGNED window: after an unclean client death the device still publishes the
    // dead session's consumed-cc (e.g. ~45k) while a fresh client counts from 1; an
    // unsigned subtraction wraps huge, reads as "window full", and gates TX shut
    // permanently (RX alive, tool shedding 100%). A stale/ahead device counter now
    // reads negative = window open; the kernel's cc-mismatch resync then adopts the
    // new session's numbering and it self-heals.
    if ((int64_t)(tx_s.usb_cc - dev_status.usb_dac_last_dqbuf[0]) > 2000) {
        return RFNM_API_MIN_QBUF_QUEUE_FULL;
    }

    std::lock_guard<std::mutex> lockGuard2(tx_s.in_mutex);

    // bound the in-queue depth independently of the cc window: the cc window (2000)
    // is wider than a typical app buffer pool (1500), so a TX path slower than the
    // RX path could swallow the app's entire pool into this queue before the window
    // closed - RX then starves for buffers and a full-duplex relay deadlocks
    // (wire-limited TX with every app buffer parked here). 256 packets is ~85 ms
    // of pipeline at full rate - never the throughput limiter, always pool-safe.
    if (tx_s.in.size() >= 256) {
        return RFNM_API_MIN_QBUF_QUEUE_FULL;
    }

    tx_s.qbuf_cnt++;
    tx_s.usb_cc++;

    // the wire field is 32-bit but the window check above is 64-bit - keep the full
    // value here (truncating would desync the two at cc 2^32, i.e. QUEUE_FULL forever
    // after ~8.3 days of full-rate TX); the wire struct field carries the low 32 bits
    // and the device side resyncs on cc as designed.
    buf->usb_cc = tx_s.usb_cc;

    // positional free-run stamping: every non-TIME_VALID packet carries the exact tick
    // of its first sample, computed from THE SESSION'S LATCHED anchor and the
    // cumulative feed position - the device places it at that ring slot, so "feed
    // position 0 airs at tx_t0" holds exactly instead of re-rolling per session with
    // an align-to-live-tail race. R = 2^tx_r_shift / 2 ticks per sample (exact;
    // packets are 256-sample multiples, so the conversion never truncates). The TX
    // epoch rides flags[15:8]: the device drops stamps minted against a torn-down
    // anchor. The anchor is latched ONCE (at seek, or at the first stamped packet)
    // and never re-read from the live dev_status cache here - see session_state.
    if (!(buf->tx_flags & (RFNM_TX_FLAG_TIME_VALID | RFNM_TX_FLAG_LIB_PRESTAMPED)) && ses.pos_intent &&
            !ses.tx_anchor_valid && !dev_status.tx_epoch) {
        // the session declared positional intent (it seeked) and this write cannot
        // be stamped - refuse it, never silently demote to free-run (a demoted burst
        // vanishes with zero counters)
        tx_s.qbuf_cnt--;
        tx_s.usb_cc--;
        return RFNM_API_TX_NOT_ANCHORED;
    }
    if (!(buf->tx_flags & (RFNM_TX_FLAG_TIME_VALID | RFNM_TX_FLAG_LIB_PRESTAMPED)) &&
            (ses.tx_anchor_valid || dev_status.tx_epoch)) {
        uint32_t samples = buf->elem_cnt ? buf->elem_cnt : RFNM_USB_TX_PACKET_ELEM_CNT;
        if (!ses.tx_anchor_valid) {
            ses.tx_anchor_t0 = dev_status.tx_t0;
            ses.tx_anchor_epoch = (uint8_t)(dev_status.tx_epoch & 0xFF);
            ses.tx_anchor_r_shift = (uint8_t)dev_status.tx_r_shift;
            ses.tx_anchor_valid = true;
            // TUP virgin auto-seek (see the v5 twin + the plan doc): a seek-less
            // continuous client's feed_pos 0 = the session-start tick, already PAST
            // under the one-lane now64 judge - every packet would late-drop and the
            // client airs silence. Anchor the virgin feed at fresh now + published
            // lead; continuous clients then stream exactly as before.
            if (!ses.pos_intent && ses.feed_pos == 0) {
                uint64_t lead = dev_status.tx_feed_min_lead_ticks ?
                        dev_status.tx_feed_min_lead_ticks : dev_status.tx_feed_lead_ticks;
                lead *= 2;	// margin (v5-twin parity, receipted u6): the status-read
                		// staleness + in-flight time eat into a bare min-lead anchor
                uint32_t ahead = (uint32_t)(dev_status.phytimer_now64 ?
                        dev_status.phytimer_now64 : dev_status.phytimer_now)
                        + (uint32_t)lead - ses.tx_anchor_t0;
                // samples = ticks*2 / 2^r_shift (R = 2^shift/2 ticks per sample)
                ses.feed_pos = ((((uint64_t)ahead << 1) >> ses.tx_anchor_r_shift) / 256 + 1) * 256;
            }
        }
        buf->phytimer = ses.tx_anchor_t0 +
                (uint32_t)tick_ratio{ ses.tx_anchor_r_shift }.samples_to_ticks(ses.feed_pos);
        buf->tx_flags = RFNM_TX_FLAG_POS_VALID | ((uint32_t)ses.tx_anchor_epoch << 8) |
                (buf->tx_flags & RFNM_TX_FLAG_EOB);
        ses.feed_pos += samples;
    }

    tx_s.in.push(buf);

    tx_s.cv.notify_one();

    return RFNM_API_OK;
}

rfnm_api_failcode device::impl::tx_dqbuf(struct tx_buf** buf) {
    std::lock_guard<std::mutex> lockGuard(tx_s.out_mutex);

    if (tx_s.out.size()) {
        *buf = tx_s.out.front();
        tx_s.out.pop();
        return RFNM_API_OK;
    }
    else {
        return RFNM_API_DQBUF_NO_DATA;
    }
}

rfnm_api_failcode device::impl::tx_feed_seek(uint64_t samples) {
    ses.pos_intent = true;	// from here on, unstampable writes are refused
    // Advance the positional free-run feed position WITHOUT sending data: the next
    // tx_qbuf stamps at the post-seek position and the device's gap scrub blanks
    // the skipped ring span. For sparse/late-starting feeders (an NR UE writes its
    // first content seconds into the session): the drain runs at line rate, so a
    // multi-second feed gap can never be closed by feeding - only by seeking.
    // Sample-grid granularity: must be a multiple of 256 (the ring slot quantum)
    // so subsequent stamps stay on the packet grid.
    if (samples % 256) {
        return RFNM_API_NOT_SUPPORTED;
    }
    // same lock tx_qbuf stamps under: a seek racing a concurrent qbuf must never
    // tear feed_pos or stamp a packet astride the seek
    std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
    ses.feed_pos += samples;
    return RFNM_API_OK;
}

rfnm_api_failcode device::impl::tx_feed_seek_to(uint64_t abs_samples) {
    ses.pos_intent = true;
    // Absolute variant: place the NEXT tx_qbuf at feed position abs_samples, i.e.
    // air its first sample at tx_t0 + abs_samples*R. Forward-only - a backward seek
    // would restamp over already-fed span (the device would drop it late anyway);
    // callers that missed their slot skip ahead instead. Same 256-sample grid rule.
    // The seek LATCHES the session anchor (see session_state): the caller computed
    // abs_samples against the timing it just read, so pinning the same observation
    // here makes seek and stamps coherent by construction.
    if (abs_samples % 256) {
        return RFNM_API_NOT_SUPPORTED;
    }
    // time unification (v5-twin parity): the latch below pins dev_status.tx_t0 -
    // refresh FIRST so the pin is a fresh observation, not the poller's cache
    // (receipted 07-17 as a constant 32 ms stamp skew). Strictly before the locks:
    // get() takes s_dev_status_mutex internally. Best-effort - a failed refresh
    // falls back to the cache as before.
    get(REQ_DEV_STATUS);
    std::lock_guard<std::mutex> lockGuard0(s_dev_status_mutex);
    std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
    if (abs_samples < ses.feed_pos) {
        return RFNM_API_SCHED_ORDER;
    }
    ses.feed_pos = abs_samples;
    if (dev_status.tx_epoch) {
        ses.tx_anchor_t0 = dev_status.tx_t0;
        ses.tx_anchor_epoch = (uint8_t)(dev_status.tx_epoch & 0xFF);
        ses.tx_anchor_r_shift = (uint8_t)dev_status.tx_r_shift;
        ses.tx_anchor_valid = true;
    }
    return RFNM_API_OK;
}

uint64_t device::impl::tx_feed_pos() {
    std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
    return ses.feed_pos;
}

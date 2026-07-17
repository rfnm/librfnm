// The ticks-first timing surface: anchors, conversions, extension, the delivered
// splice, health counters and the apply-timing handle. Every getter that reads a
// device anchor refreshes dev_status first - an anchor read must never serve a stale
// cache (the defect-#13 class).

#include "../core/impl.h"
#include "../core/timebase.h"

using namespace rfnm;

MSDLL uint64_t device::rx_tick_extend(uint32_t stamp, uint32_t adc_id) {
    impl& m = *pimpl;
    if (adc_id > 3) {
        return stamp;
    }
    return m.ses.rx_ext[adc_id].extend_near(stamp);
}

MSDLL uint64_t device::rx_tick_to_ns(uint64_t ticks) {
    // clock plan unknown -> honest zero, never a constant guess (timebase contract)
    return timebase::from_hwinfo(pimpl->hwinfo).ticks_to_ns(ticks);
}

MSDLL uint64_t device::rx_ns_to_tick(uint64_t ns) {
    return timebase::from_hwinfo(pimpl->hwinfo).ns_to_ticks(ns);
}

MSDLL uint64_t device::rx_samples_to_ticks(uint64_t samples) {
    return tick_ratio{ pimpl->dev_status.rx_r_shift }.samples_to_ticks(samples);
}

MSDLL rfnm_api_failcode device::get_rx_delivered(uint64_t *delivered_samples,
        uint64_t *tick_at_delivered, uint32_t adc_id) {
    impl& m = *pimpl;
    // local splice state only - no control round trip, callable per-write.
    // Guarded by the out_mutex like the other ordered-point state.
    std::lock_guard<std::mutex> lockGuard(m.rx_s.out_mutex);
    if (adc_id >= 4 || !m.ses.delivered_map_valid[adc_id]) {
        return RFNM_API_DQBUF_NO_DATA;
    }
    if (delivered_samples) {
        *delivered_samples = m.ses.delivered_samples[adc_id];
    }
    if (tick_at_delivered) {
        *tick_at_delivered = m.ses.delivered_end_tick[adc_id];
    }
    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::get_apply_timing(uint8_t *timing_break, uint32_t *rx_epoch_at_apply, uint32_t *tx_epoch_at_apply) {
    impl& m = *pimpl;
    // reads the stored v5 result tail (same idiom as get_apply_error: apply/set are
    // client-serialized, last_set_res is written at the cc-matched result fetch)
    if (timing_break) {
        *timing_break = m.ses.last_set_res.timing_break;
    }
    if (rx_epoch_at_apply) {
        *rx_epoch_at_apply = m.ses.last_set_res.rx_epoch_at_apply;
    }
    if (tx_epoch_at_apply) {
        *tx_epoch_at_apply = m.ses.last_set_res.tx_epoch_at_apply;
    }
    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::apply_timing_settled(bool *settled) {
    impl& m = *pimpl;
    // one status refresh per poll, OUTSIDE the status mutex (get() takes its own
    // locks); liveness rides the status path's loud dead-latches, so a dead board
    // errors this call instead of wedging the caller's poll loop
    rfnm_api_failcode r = m.get(REQ_DEV_STATUS);
    if (r != RFNM_API_OK) {
        return r;
    }
    std::lock_guard<std::mutex> lockGuard(m.s_dev_status_mutex);
    uint8_t brk = m.ses.last_set_res.timing_break;
    bool ok = true;
    if (brk & 0x1) {
        ok = ok && ((int32_t)(m.dev_status.rx_epoch - m.ses.last_set_res.rx_epoch_at_apply) > 0);
    }
    if (brk & 0x2) {
        ok = ok && ((int32_t)(m.dev_status.tx_epoch - m.ses.last_set_res.tx_epoch_at_apply) > 0);
    }
    if (settled) {
        *settled = ok;
    }
    return RFNM_API_OK;
}

// ---- the merged timing/health/contract surface ----

MSDLL rfnm_api_failcode device::get_timing(enum stream_dir dir, struct timing *t) {
    impl& m = *pimpl;
    rfnm_api_failcode r = m.get(REQ_DEV_STATUS);
    if (r != RFNM_API_OK) {
        return r;
    }
    uint32_t raw_t0 = (dir == DIR_RX) ? m.dev_status.rx_t0 : m.dev_status.tx_t0;
    tick_ratio ratio{ (dir == DIR_RX) ? m.dev_status.rx_r_shift : m.dev_status.tx_r_shift };
    t->tick_hz = timebase::from_hwinfo(m.hwinfo).tick_hz;
    t->r_num = ratio.r_num();
    t->r_den = ratio.r_den();
    // the ONE extension rule (header contract): both directions extend into the RX
    // stamps' domain while it is live, so "tick T" is one instant across RX and TX
    t->t0_ext = m.ses.rx_ext[0].valid ? m.ses.rx_ext[0].extend_near(raw_t0) : (uint64_t)raw_t0;
    t->epoch = (dir == DIR_RX) ? m.dev_status.rx_epoch : m.dev_status.tx_epoch;
    t->anchored = (t->epoch != 0);
    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::clock_now(uint64_t *tick_ext) {
    impl& m = *pimpl;
    if (m.get(REQ_DEV_STATUS)) {
        return RFNM_API_USB_FAIL;
    }
    uint32_t raw = m.dev_status.phytimer_now;
    if (m.ses.rx_ext[0].valid) {
        // an RX stream is (or was) live: stay in the stamps' extended domain
        *tick_ext = m.ses.rx_ext[0].extend_near(raw);
        return RFNM_API_OK;
    }
    // stream-less: unwrap against this handle's previous read (exact within +-35 s)
    *tick_ext = m.ses.ptmr_ext.step(raw);
    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::get_phytimer64(uint64_t *tick64) {
    // time unification: the kernel samples its 64-bit extension AT REQUEST TIME and
    // publishes it in dev_status - one refresh here, never a cached copy
    impl& m = *pimpl;
    rfnm_api_failcode r = m.get(REQ_DEV_STATUS);
    if (r != RFNM_API_OK) {
        return r;
    }
    std::lock_guard<std::mutex> lockGuard(m.s_dev_status_mutex);
    if (tick64) {
        *tick64 = m.dev_status.phytimer_now64;
    }
    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::get_phy_gen(uint32_t *gen) {
    impl& m = *pimpl;
    rfnm_api_failcode r = m.get(REQ_DEV_STATUS);
    if (r != RFNM_API_OK) {
        return r;
    }
    std::lock_guard<std::mutex> lockGuard(m.s_dev_status_mutex);
    if (gen) {
        *gen = m.dev_status.phy_gen;
    }
    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::get_health(struct health *h) {
    impl& m = *pimpl;
    rfnm_api_failcode r = m.get(REQ_DEV_STATUS);
    if (r != RFNM_API_OK) {
        return r;
    }
    {
        std::lock_guard<std::mutex> lockGuard(m.s_dev_status_mutex);
        h->rx_regates = m.dev_status.rx_regate_cnt;
        h->tx_underruns = m.dev_status.tx_underrun_cnt;
        h->tx_timed_rejects = m.dev_status.tx_timed_reject_cnt;
        h->tx_pos_placed = m.dev_status.tx_pos_placed;
        h->tx_pos_late = m.dev_status.tx_pos_late;
        h->tx_pos_stale = m.dev_status.tx_pos_stale;
        h->tx_pos_misaligned = m.dev_status.tx_pos_misaligned;
        h->tx_pace_rolls = m.dev_status.tx_pace_rolls;
        h->tx_arm_repairs = m.dev_status.tx_arm_repairs;
        h->tx_last_late_cc = m.dev_status.tx_last_late_usb_cc;
        h->tx_ring_read_ptr = m.dev_status.tx_ring_read_ptr;
        h->tx_ring_head = m.dev_status.tx_ring_head;
    }
    // host-local counters: monotonic aligned loads, no lock needed
    uint64_t d = 0, b = 0, ok = 0, drop = 0;
    for (int a = 0; a < 4; a++) {
        d += m.rx_s.phytimer_discont[a];
        b += m.rx_s.phytimer_break[a];
        ok += m.rx_s.usb_cc_ok[a];
        drop += m.rx_s.usb_cc_dropped[a];
    }
    h->rx_stamp_disconts = d;
    h->rx_stamp_breaks = b;
    h->rx_pkts_ok = ok;
    h->rx_pkts_dropped = drop;
    h->tx_headroom_peak_abs = m.tx_peak_abs;
    h->tx_headroom_scanned = m.tx_headroom_scanned;
    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::get_contract(struct feed_contract *c) {
    impl& m = *pimpl;
    rfnm_api_failcode r = m.get(REQ_DEV_STATUS);
    if (r != RFNM_API_OK) {
        return r;
    }
    c->tx_feed_lead_ticks = m.dev_status.tx_feed_min_lead_ticks ?
            m.dev_status.tx_feed_min_lead_ticks : m.dev_status.tx_feed_lead_ticks;
    c->rx_flush_deadline_us = m.dev_status.rx_flush_deadline_us;
    c->anchor_step_ticks = m.dev_status.sched_anchor_step_ticks;
    return RFNM_API_OK;
}

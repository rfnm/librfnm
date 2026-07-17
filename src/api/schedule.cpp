// Absolute-time scheduling: the request-ring verbs (RX windows, TX slot windows, the
// sticky anchor intent), TDD pattern configuration, and pad-to-exact timed TX stamping.

#include <cstring>

#include "../core/impl.h"
#include "../core/timebase.h"

using namespace rfnm;

MSDLL rfnm_api_failcode device::schedule_reset() {
    struct rfnm_dev_txn t = {0};
    // the schedule request rides the control channel on every transport
    // (USB ep0 / TCP ctrl socket / LOCAL ioctl) - no transport gate
    t.op = 0;
    return pimpl->ctrl_transfer(RFNM_SET_TXN, sizeof(struct rfnm_dev_txn), (unsigned char *)&t, 50);
}

rfnm_api_failcode device::impl::schedule_ctl(uint64_t tick, uint8_t kind, uint16_t type,
        uint32_t len_samples, uint8_t flags, uint32_t bind) {
    // the scheduled-request verb (bench tools reach it via the public wrappers). The
    // device's request ring is an internal scheduler, not a client concept: apps
    // schedule RX with schedule_rx() and TX via schedule_tx_slot() / TIME_VALID
    // data packets.
    struct rfnm_dev_txn t = {0};
    t.op = 1;
    t.tick = (uint32_t)tick;    // device works mod 2^32; extended ticks truncate exactly
    t.kind = kind;
    t.flags = flags;
    t.type = type;
    t.len = len_samples;
    t.bind = bind;
    rfnm_api_failcode ret = ctrl_transfer(RFNM_SET_TXN, sizeof(struct rfnm_dev_txn), (unsigned char *)&t, 50);
    if (ret == RFNM_API_OK && kind == 1) {
        // an RX window is scheduled: from here on bulk-IN silence between windows is
        // legitimate and the RX dead-pipe watchdog must judge by the device's ship
        // counters (sticky - see session_state)
        ses.rx_scheduled_session = true;
    }
    return ret;
}

MSDLL rfnm_api_failcode device::schedule_tx_slot(uint64_t tick, uint32_t len_samples, uint32_t dac_slot) {
    // the public ordered-path TX schedule (kind 2, ring-resident payload) - see the
    // header contract; the raw ctl verb stays private
    return pimpl->schedule_ctl(tick, 2, 0, len_samples, 0, dac_slot);
}

MSDLL rfnm_api_failcode device::schedule_rx(uint64_t tick, uint32_t len_samples, uint16_t tag) {
    // capture len_samples starting at the absolute phytimer tick (sample resolution).
    // Fire-and-forget on remote transports: schedule health is read back via the
    // dev_status counters, arrivals via the stamped RX packets themselves.
    // The window LENGTH quantizes UP to the VSPA chunk (768 samples): the open tick
    // stays exact, extra tail samples arrive stamped - the client discards past its
    // request instead of losing coverage to a truncating division device-side.
    uint32_t len = (len_samples + 767u) / 768u * 768u;
    return pimpl->schedule_ctl(tick, 1 /* RX window */, tag, len, 0, 0);
}

MSDLL rfnm_api_failcode device::anchor_at(uint64_t tick) {
    // sticky session anchor (kind 4 on the schedule verb): phase congruence and
    // re-mint stickiness live kernel/M4-side - see the header contract. USB/TCP are
    // fire-and-forget; verify via get_timing after the next apply.
    return pimpl->schedule_ctl(tick, 4 /* ANCHOR */, 0, 0, 1 /* flags bit0: tick valid */, 0);
}

MSDLL rfnm_api_failcode device::rx_tdd_configure(uint64_t period_ticks, uint64_t duty_ticks) {
    impl& m = *pimpl;
    struct rfnm_dev_tdd r_tdd;
    uint64_t chunk_ticks;

    // refresh the clock plan: the chunk length in ticks depends on the ADC divider
    if (m.get(REQ_HWINFO)) {
        return RFNM_API_USB_FAIL;
    }
    chunk_ticks = rx_chunk_ticks(m.hwinfo);

    // TDD v2 fw floor: BOTH spans (duty and period-duty) >= 600 us, enforced here in
    // THIS plan's tick rate so the refusal is synchronous (USB/TCP SET_TDD is
    // fire-and-forget; the fw refuses sub-floor patterns in telemetry only).
    timebase tb = timebase::from_hwinfo(m.hwinfo);
    uint64_t span_floor_ticks = tb.us_to_ticks_floor(600);
    if (!tb.valid() || !period_ticks || !duty_ticks ||
            period_ticks % chunk_ticks || duty_ticks % chunk_ticks ||
            duty_ticks >= period_ticks ||
            period_ticks / chunk_ticks > 0xFFFF ||
            duty_ticks < span_floor_ticks ||
            (period_ticks - duty_ticks) < span_floor_ticks) {
        return RFNM_API_NOT_SUPPORTED;
    }

    r_tdd.period_chunks = (uint32_t)(period_ticks / chunk_ticks);
    r_tdd.duty_chunks = (uint32_t)(duty_ticks / chunk_ticks);
    rfnm_api_failcode ret = m.ctrl_transfer(RFNM_SET_TDD, sizeof(struct rfnm_dev_tdd), (unsigned char *)&r_tdd, 50);
    if (ret == RFNM_API_OK) {
        // TDD gating armed: the pipe is legitimately silent outside the RX duty window
        // (sticky - see session_state)
        m.ses.rx_scheduled_session = true;
        // the projector's basis (tdd_project_lead): this session now knows the pattern
        m.ses.tdd_period_ticks = period_ticks;
        m.ses.tdd_duty_ticks = duty_ticks;
    }
    return ret;
}

MSDLL rfnm_api_failcode device::rx_tdd_stop() {
    impl& m = *pimpl;
    struct rfnm_dev_tdd r_tdd = {0, 0};
    rfnm_api_failcode ret = m.ctrl_transfer(RFNM_SET_TDD, sizeof(struct rfnm_dev_tdd), (unsigned char *)&r_tdd, 50);
    if (ret == RFNM_API_OK) {
        m.ses.tdd_period_ticks = 0;
        m.ses.tdd_duty_ticks = 0;
    }
    return ret;
}

MSDLL rfnm_api_failcode device::tdd_project_lead(uint64_t delivered_lead_samples, uint64_t *feed_lead_samples) {
    impl& m = *pimpl;
    uint64_t p = m.ses.tdd_period_ticks;
    uint64_t d = m.ses.tdd_duty_ticks;
    if (!p || !d) {
        // no pattern known to THIS session (none armed, or armed by a predecessor) -
        // refuse rather than return a silent identity a gated consumer would mis-stamp by
        return RFNM_API_SCHED_NO_ANCHOR;
    }
    if (feed_lead_samples) {
        *feed_lead_samples = gated_feed_lead(delivered_lead_samples, p, d);
    }
    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::tx_buf_schedule(struct tx_buf *buf, uint64_t tick) {
    impl& m = *pimpl;
    uint32_t caller_flags = buf->tx_flags;
    if (!m.dev_status.tx_epoch) {
        return RFNM_API_SCHED_NO_ANCHOR;
    }
    // pad-to-exact (plan math in core/timebase.h, test-pinned): the gate can only open
    // on the DAC slot grid, so a mid-slot tick opens one slot boundary below and leads
    // with zeros - the CONTENT airs at `tick` exactly.
    uint32_t cnt = buf->elem_cnt ? buf->elem_cnt : RFNM_USB_TX_PACKET_ELEM_CNT;
    tx_pad_plan plan = plan_tx_pad(tick, m.dev_status.tx_t0, tick_ratio{ m.dev_status.tx_r_shift },
            cnt, RFNM_USB_TX_PACKET_ELEM_CNT);
    if (!plan.ok) {
        return RFNM_API_SCHED_MISALIGNED;   // tick between samples, or no slack for the pad
    }
    if (plan.lead_samples) {
        uint32_t lead = plan.lead_samples;
        uint32_t padded = plan.padded_elem_cnt;
        memmove(buf->buf + (size_t)lead * 4, buf->buf, (size_t)cnt * 4);
        memset(buf->buf, 0, (size_t)lead * 4);
        if (padded > cnt + lead) {
            memset(buf->buf + (size_t)(cnt + lead) * 4, 0, (size_t)(padded - cnt - lead) * 4);
        }
        buf->elem_cnt = padded;
        tick = plan.gate_tick;
    }
    if (rfnm_txsched_positional()) {
        // TUP 3b stamped-positional lane (pad above is SHARED - parity is structural):
        // POS_VALID at the slot-aligned absolute tick; the kernel's pos gap-scrub gives
        // the inter-burst silence and its judge enforces the lead. Lib pre-check keeps
        // the failure loud and local; one fresh status per call (per-burst cadence).
        get(REQ_DEV_STATUS);
        uint64_t now64 = m.dev_status.phytimer_now64;
        uint32_t lead_ticks = m.dev_status.tx_feed_min_lead_ticks ?
                m.dev_status.tx_feed_min_lead_ticks : m.dev_status.tx_feed_lead_ticks;
        if (now64 && tick < now64 + lead_ticks) {
            return RFNM_API_SCHED_LATE;     // now64==0 = pre-step-1 kernel: its judge decides
        }
        buf->phytimer = (uint32_t)tick;
        buf->tx_flags = RFNM_TX_FLAG_POS_VALID | RFNM_TX_FLAG_LIB_PRESTAMPED |
                ((m.dev_status.tx_epoch & 0xFF) << 8) | (caller_flags & RFNM_TX_FLAG_EOB);
        return RFNM_API_OK;
    }
    buf->phytimer = (uint32_t)tick;
    buf->tx_flags = RFNM_TX_FLAG_TIME_VALID | ((m.dev_status.tx_epoch & 0xFF) << 8);
    return RFNM_API_OK;
}

// RX ordered delivery: the client-facing queue half. dqbuf_is_cc_continuous encodes
// the reorder/adopt rules (DISCONT-adopt, hole-step, consumer-lag clamp, epoch culls);
// rx_dqbuf's ordered pop maintains the stamp chains, the tick extension and the
// delivered<->tick splice.

#include <cstdlib>

#include <spdlog/spdlog.h>

#include "impl.h"

using namespace rfnm;

rfnm_api_failcode device::impl::rx_qbuf(struct rx_buf* buf, bool new_buffer) {
    {
        std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
        if (new_buffer) rx_s.qbuf_cnt++;
        rx_s.in.push(buf);
    }
    rfnm_rx_in_nap_cv.notify_all();
    return RFNM_API_OK;
}

int device::impl::single_ch_id_bitmap_to_adc_id(uint8_t ch_ids) {
    size_t ch_id = 0;
    while (ch_id < MAX_RX_CHANNELS) {
        if ((ch_ids & 0x1) == 1) {
            return rx.ch[ch_id].adc_id;
        }
        ch_id++;
        ch_ids = ch_ids >> 1;
    }
    return -1;
}

// core of the cc adoption; caller MUST hold out_mutex. A single lock-held decision
// point: an unlock-then-relock flow would let the queue top change between the
// adoption decision and the adoption itself.
void device::impl::dqbuf_overwrite_cc_locked(uint8_t adc_id) {
    rx_s.in_mutex.lock();

    uint64_t old_cc = rx_s.usb_cc[adc_id];
    size_t queue_size = rx_s.out[adc_id].size();

    // use whatever buffer is at the top of the queue
    if (queue_size) {
        rx_s.usb_cc[adc_id] = rx_s.out[adc_id].top()->usb_cc;
    }
    else {
        rx_s.usb_cc[adc_id]++;
    }

    rx_s.in_mutex.unlock();

    if (rx_s.usb_cc[adc_id] > old_cc) {
        rx_s.usb_cc_dropped[adc_id] += (rx_s.usb_cc[adc_id] - old_cc);
    }

    // saturation produces thousands of drop events per second - rate-limit the log
    // (counters keep accumulating) and report drops as a share of all packets
    static std::atomic<int64_t> last_drop_log_ms{0};
    int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    int64_t prev_ms = last_drop_log_ms.load(std::memory_order_relaxed);
    if (now_ms - prev_ms >= 1000 && last_drop_log_ms.compare_exchange_strong(prev_ms, now_ms)) {
        uint64_t total_cc = rx_s.usb_cc_ok[adc_id] + rx_s.usb_cc_dropped[adc_id];
        spdlog::info("cc {} -> {} size {} adc {} remote {}; ok {} dropped {} ({:.4f}%)",
            old_cc, rx_s.usb_cc[adc_id], queue_size, adc_id, static_cast<uint64_t>(dev_status.usb_adc_last_qbuf[adc_id]),
            rx_s.usb_cc_ok[adc_id], rx_s.usb_cc_dropped[adc_id],
            total_cc > 0 ? (100.0 * rx_s.usb_cc_dropped[adc_id] / total_cc) : 0);
    }
}

int device::impl::dqbuf_is_cc_continuous(uint8_t adc_id, int acquire_lock) {
    struct rx_buf* buf;
    size_t queue_size;

    if (acquire_lock) {
        rx_s.out_mutex.lock();
    }

    queue_size = rx_s.out[adc_id].size();
    if (queue_size < 1) {
        if (acquire_lock) {
            rx_s.out_mutex.unlock();
        }
        return 0;
    }

    buf = rx_s.out[adc_id].top();

    // stamps and flags only mean something within the CURRENT stream generation; a stale
    // session's leftovers (the dwc3 gadget re-delivers them at USB stream head) carry the
    // previous epoch and must never anchor, adopt, or survive the reorder on trust
    uint8_t cur_epoch = (uint8_t)(dev_status.rx_epoch & 0xFF);

    // special case for first buffer of stream
    if (rx_s.usb_cc[adc_id] == UINT64_MAX) {
        int ret = 0;
        if (getenv("RFNM_DEBUG_RX")) {
            spdlog::info("CCINIT adc {} queue_size {}", adc_id, queue_size);
        }

        // never anchor on a stale generation: cull old-epoch leftovers first (they would
        // otherwise count toward - or win - the anchor and hijack the fresh session)
        while (queue_size > 0 && (uint8_t)RFNM_STREAM_FLAG_EPOCH(buf->rx_flags) != cur_epoch) {
            std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
            rx_s.out[adc_id].pop();
            rx_s.in.push(buf);
            rfnm_rx_in_nap_cv.notify_all();
            queue_size--;
            if (queue_size == 0) {
                break;
            }
            buf = rx_s.out[adc_id].top();
        }

        // anchor once 10 buffers arrived (out-of-order safety), or immediately on a
        // current-epoch DISCONT top: the flagged packet IS the first packet after a seam
        // (stream start, window start), so it is a true anchor by definition - sparse
        // scheduled captures and slow full-record streams never accumulate 10
        if (queue_size >= 10 || (queue_size > 0 && (buf->rx_flags & RFNM_RX_FLAG_DISCONT))) {
            rx_s.usb_cc[adc_id] = buf->usb_cc;
            ret = 1;
        }

        if (acquire_lock) {
            rx_s.out_mutex.unlock();
        }
        return ret;
    }

    int max_allowed_in_flight = RX_MAX_INFLIGHT_BUF_CNT;
    if (transport_status.transport == TRANSPORT_TCP) {
        max_allowed_in_flight = RX_MAX_INFLIGHT_BUF_CNT_ETH;
    }

    // the TCP/local stream carries the local ring's cc, which ticks independently of the usb
    // ring's cc (variable-size usb packets advance the usb counter faster) - compare like to like
    uint64_t dev_last_qbuf = dev_status.usb_adc_last_qbuf[adc_id];
    if (transport_status.transport == TRANSPORT_TCP || transport_status.transport == TRANSPORT_LOCAL) {
        dev_last_qbuf = dev_status.local_adc_last_qbuf[adc_id];
    }

    if (abs(((int64_t)dev_last_qbuf) - ((int64_t)rx_s.usb_cc[adc_id])) > max_allowed_in_flight) {
        if (dev_last_qbuf > rx_s.usb_cc[adc_id]) {
            // device counter far AHEAD: consumer lag. Jump to the freshest packet we
            // actually HOLD, not the device's producer counter: on a lossless transport
            // (TCP) the producer counter runs ahead of data still queued in the socket,
            // and jumping past it culls every arriving packet forever - a slow consumer
            // starved into a silent dead stream. Jumping to the held head keeps the
            // stream live at the consumer's pace and bounds staleness by the transport
            // pipeline depth. The skipped span is accounted - the freshness contract's
            // "you may lose samples, counted".
            uint64_t jump_to = rx_s.usb_cc_max_seen[adc_id];
            if (jump_to > dev_last_qbuf) {
                jump_to = dev_last_qbuf;    // never trust a held cc past the producer (stale-session garbage)
            }
            if (jump_to > rx_s.usb_cc[adc_id]) {
                spdlog::info("max allowed inflight exceeded, reset cc from {} to {} (dev head {})",
                    rx_s.usb_cc[adc_id], jump_to, static_cast<uint64_t>(dev_last_qbuf));
                rx_s.usb_cc_dropped[adc_id] += jump_to - rx_s.usb_cc[adc_id];
                rx_s.usb_cc[adc_id] = jump_to;
            }
        } else {
            // device counter far BEHIND: the device-side counter restarted (LA9310
            // hard reset during a reclock). Pinning expected to the stale head wedges
            // the stream forever - the new session counts from 1 and the hole-step
            // logic cannot cross the seam at qsize 1. Re-anchor on whatever arrives
            // next instead. Return now: with expected at UINT64_MAX the discard loop
            // below would drain the new session's queue.
            spdlog::info("device cc counter restarted ({} < expected {}), re-anchoring adc {}", static_cast<uint64_t>(dev_last_qbuf), rx_s.usb_cc[adc_id], adc_id);
            rx_s.usb_cc[adc_id] = UINT64_MAX;
            rx_s.usb_cc_max_seen[adc_id] = 0;
            if (acquire_lock) {
                rx_s.out_mutex.unlock();
            }
            return 0;
        }
    }

    // Local transport freshness bound (defect #42): the zero-copy pool is 991 packets -
    // BELOW the 1000-packet inflight threshold above, so the consumer-lag jump can never
    // fire and a slow-but-alive consumer holds a permanently full FIFO: delivery goes
    // seconds stale with every read succeeding (kernel DISCONT seams heal nothing
    // because they are adopted only once they surface behind ~990 older packets).
    // Bound the HELD queue depth directly - queue_size and usb_cc_max_seen are
    // in-process truth, no dev_status round-trip - with the same jump-and-account
    // semantics as above (the freshness contract: lose samples under lag, counted,
    // never silently deliver stale ones). Env-tunable while the default stays 128;
    // 0 disables the bound (production spectrumd path until #42 flips default-on).
    if (transport_status.transport == TRANSPORT_LOCAL) {
        static const int local_bound = []() {
            const char *e = getenv("RFNM_RX_LOCAL_MAX_QUEUE");
            return e ? atoi(e) : 128;
        }();
        if (local_bound > 0 && queue_size > (size_t)local_bound && rx_s.usb_cc_max_seen[adc_id] > rx_s.usb_cc[adc_id]) {
            uint64_t jump_to = rx_s.usb_cc_max_seen[adc_id];
            spdlog::info("local rx queue {} > bound {}, reset cc from {} to {}",
                queue_size, local_bound, rx_s.usb_cc[adc_id], jump_to);
            rx_s.usb_cc_dropped[adc_id] += jump_to - rx_s.usb_cc[adc_id];
            rx_s.usb_cc[adc_id] = jump_to;
        }
    }

    std::vector<uint64_t> discarded;
    while (queue_size > 0) {
        // discard anything OLDER than expected, INCLUDING expected-1: a duplicate of the
        // just-consumed cc (kernel ring wrap re-stamping a buffer held by a late in-flight
        // req) otherwise sits at the queue top forever - neither continuous nor discardable -
        // and wedges the stream behind it until the timeout mass-discard kills the session.
        // A stale-OLDER top is discardable even as the last queued buffer: it can never be
        // delivered, and a queue_size > 1 guard would leave it parked at the top blocking
        // dqbuf entirely (the inflight reset jumps expected forward while old packets are
        // still in flight; the late arrival then starves the stream in a tight no-data
        // spin). Far-NEWER tops keep the > 1 guard.
        // EXCEPT a far-newer top carrying RFNM_RX_FLAG_DISCONT: that is a deliberate seam
        // (a re-gated scheduled window, or a kernel catch-up drop), not a reorder artifact.
        // Discarding it stalls scheduled/gapped capture whose windows sit >RX_RECOMB_BUF_LEN
        // past the last cc; leave it at the top so the DISCONT-adopt path below takes it now
        // (the freshness contract). Continuous streams only see this on a real discontinuity,
        // where adopting immediately is already the intended behaviour. The flag is only
        // trusted from the CURRENT epoch - a stale session's flagged leftover must not
        // survive the cull just to hijack the cc numbering downstream.
        bool far_newer = queue_size > 1 && buf->usb_cc > (rx_s.usb_cc[adc_id] + RX_RECOMB_BUF_LEN) &&
                !((buf->rx_flags & RFNM_RX_FLAG_DISCONT) && (uint8_t)RFNM_STREAM_FLAG_EPOCH(buf->rx_flags) == cur_epoch);
        if (buf->usb_cc < rx_s.usb_cc[adc_id] || far_newer) {

            uint64_t usb_cc = buf->usb_cc;
            if (getenv("RFNM_DEBUG_RX")) {
                spdlog::info("DISCARD adc {} buf_cc {} expected_cc {} qsize {}", adc_id, usb_cc, rx_s.usb_cc[adc_id], queue_size);
            }
            std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
            rx_s.out[adc_id].pop();
            rx_s.in.push(buf);
            rfnm_rx_in_nap_cv.notify_all();

            // do not log stale for eth transport as it could be retransmitted packets
            if (transport_status.transport != TRANSPORT_TCP) {
                discarded.push_back(usb_cc);
            }

            queue_size--;
            if (queue_size == 0) {
                break;	// drained the queue entirely; no top to inspect
            }
            buf = rx_s.out[adc_id].top();
        }
        else {
            break;
        }
    };

    if (queue_size == 0) {
        if (acquire_lock) {
            rx_s.out_mutex.unlock();
        }
        return 0;
    }

    if (!discarded.empty()) {
        std::string list;
        list.reserve(discarded.size() * 6);
        for (size_t i = 0; i < discarded.size(); ++i) {
            if (i) list += ", ";
            list += std::to_string(discarded[i]);
        }
        int qout = rx_s.out[adc_id].size();
        int qin = rx_s.in.size();
        spdlog::info("stale adc {} cc {} qin {} qout {}: [{}]", adc_id, rx_s.usb_cc[adc_id], qin, qout, list);
    }

    if (rx_s.usb_cc[adc_id] == buf->usb_cc) {
        if (acquire_lock) {
            rx_s.out_mutex.unlock();
        }
        return 1;
    }
    else {
        if (getenv("RFNM_DEBUG_RX") && buf->usb_cc != rx_s.usb_cc[adc_id]) {
            static int dbg_throttle;
            if (++dbg_throttle % 50 == 1) {
                spdlog::info("NOTCONT adc {} top_cc {} expected {} qsize {}", adc_id, buf->usb_cc, rx_s.usb_cc[adc_id], queue_size);
            }
        }
        // A FLAGGED discontinuity is an EXPECTED hole - the device re-gated, or the
        // kernel dropped a backlog to catch up and marked the seam. The missing ccs
        // will never arrive: adopt the top packet's numbering NOW. Waiting out the
        // reorder timeout per hole turns a lagging stream into a seconds-stale
        // trickle (the freshness contract: you may lose samples under lag, you never
        // silently get stale ones - usb_cc_dropped and the stamps carry the truth).
        // Trusted from the current epoch only, decided and applied under one lock
        // hold (the top must not change between decision and adoption), and the
        // adopted top is deliverable right now - report it as continuous.
        int adopted = 0;
        if ((buf->rx_flags & RFNM_RX_FLAG_DISCONT) && (uint8_t)RFNM_STREAM_FLAG_EPOCH(buf->rx_flags) == cur_epoch) {
            dqbuf_overwrite_cc_locked(adc_id);
            adopted = 1;
        }
        // A hole at the queue head with a healthy backlog behind it means the expected
        // packet was lost in transit, not reordered (the transport pipeline depth
        // bounds reordering): step over it now. Gating this on RX_RECOMB_BUF_LEN
        // hoards ~200 ms of stream behind a packet that will never arrive - under
        // full-duplex USB, where occasional single-packet losses are real, that
        // becomes a stall/discard cycle that collapses the whole stream.
        // usb_cc_dropped accounts for the holes.
        else if (queue_size > 16) {
            dqbuf_overwrite_cc_locked(adc_id);
            adopted = 1;
        }
        if (acquire_lock) {
            rx_s.out_mutex.unlock();
        }
        return adopted;
    }
}

rfnm_api_failcode device::impl::rx_dqbuf(struct rx_buf** buf, uint8_t ch_ids, uint32_t timeout_us) {
    int is_single_ch, required_adc_id;

    if (rx_s.qbuf_cnt < MIN_RX_BUFCNT) {
        return RFNM_API_MIN_QBUF_CNT_NOT_SATIFIED;
    }

    // the transport watchdog declared this session's RX pipe unrecoverable (resync
    // budget spent below a parked board-side producer). Distinct failcode so clients
    // reopen instead of spinning on NO_DATA against a pipe that will never move.
    if (ses.rx_pipe_dead_latch.load(std::memory_order_relaxed)) {
        return RFNM_API_RX_PIPE_DEAD;
    }

    switch (ch_ids) {
    case CH0:
    case CH1:
    case CH2:
    case CH3:
    case CH4:
    case CH5:
    case CH6:
    case CH7:
        is_single_ch = 1;
        break;
    case 0:
        is_single_ch = 0;
        ch_ids = 0xff;
        break;
    default:
        is_single_ch = 0;
        break;
    }

    if (is_single_ch) {
        required_adc_id = single_ch_id_bitmap_to_adc_id(ch_ids);
    }
    else {
        do {
            uint8_t mask = channel_flags[last_dqbuf_ch];
            required_adc_id = single_ch_id_bitmap_to_adc_id(ch_ids & mask);

            if (++last_dqbuf_ch == 8) {
                last_dqbuf_ch = 0;
            }
        } while (required_adc_id < 0);
    }

    if (!dqbuf_is_cc_continuous(required_adc_id, 1)) {
        if (!timeout_us) {
            return RFNM_API_DQBUF_NO_DATA;
        }

        {
            std::unique_lock lk(rx_s.out_mutex);
            rx_s.cv.wait_for(lk, std::chrono::microseconds(timeout_us),
                [this, required_adc_id] { return dqbuf_is_cc_continuous(required_adc_id, 0) ||
                rx_s.out[required_adc_id].size() > RX_RECOMB_BUF_LEN ||
                ses.rx_pipe_dead_latch.load(std::memory_order_relaxed); }
            );
            if (ses.rx_pipe_dead_latch.load(std::memory_order_relaxed)) {
                return RFNM_API_RX_PIPE_DEAD;   // latched mid-wait (the watchdog notifies)
            }

            // timeout-expiry anchor: an unanchored cc after a full client timeout means the
            // init window will never fill (sparse scheduled capture, or a slow full-record
            // stream whose 10-buffer wait costs hundreds of ms) - whatever is queued is all
            // there is. Anchor on the lowest queued cc; anything still in flight below it
            // is bounded by the pipeline depth and gets culled as stale, accounted.
            if (rx_s.usb_cc[required_adc_id] == UINT64_MAX && rx_s.out[required_adc_id].size() > 0) {
                rx_s.usb_cc[required_adc_id] = rx_s.out[required_adc_id].top()->usb_cc;
                spdlog::info("cc init anchor on timeout: adc {} cc {} qsize {}", required_adc_id,
                    rx_s.usb_cc[required_adc_id], rx_s.out[required_adc_id].size());
            }
        }

        if (!dqbuf_is_cc_continuous(required_adc_id, 1)) {
            if (getenv("RFNM_DEBUG_RX")) {
                std::lock_guard<std::mutex> lockGuard(rx_s.out_mutex);
                uint64_t topcc = rx_s.out[required_adc_id].size() ? rx_s.out[required_adc_id].top()->usb_cc : 0;
                spdlog::info("STUCK adc {} expected_cc {} qsize {} top_cc {}", required_adc_id,
                    rx_s.usb_cc[required_adc_id], rx_s.out[required_adc_id].size(), topcc);
            }
            if (timeout_us >= 10000) {
                spdlog::info("cc timeout {} adc {}", rx_s.usb_cc[required_adc_id], required_adc_id);
            }

            return RFNM_API_DQBUF_NO_DATA;
        }
    }

    {
        std::lock_guard<std::mutex> lockGuard(rx_s.out_mutex);
        *buf = rx_s.out[required_adc_id].top();
        rx_s.out[required_adc_id].pop();
    }

    struct rx_buf* lb;
    lb = *buf;

    rx_s.usb_cc[required_adc_id]++;

    // exactly one ok tick per DELIVERED packet (a cv-predicate re-evaluation must
    // never double-count)
    rx_s.usb_cc_ok[required_adc_id]++;
    // the delivered axis is cumulative across epochs and gaps - every in-order
    // dequeue advances it (this IS the consumer's sample clock). Under out_mutex:
    // get_rx_delivered reads the (samples, tick) pair from other threads and a torn
    // pair would skew the map by a packet.
    {
        std::lock_guard<std::mutex> lockGuard(rx_s.out_mutex);
        ses.delivered_samples[required_adc_id] += lb->elem_cnt;
    }

    // stamp-chain validation at the ordered pop point (packets are in usb_cc order per
    // adc here). Engages only when the packet's epoch matches the cached stream ack in
    // dev_status, so rate-change windows and stale-prefix drains can never manufacture
    // false breaks. R = 2^rx_r_shift / 2 ticks per sample, exact. The chain re-anchors
    // after every event so one break is one count, not a storm.
    {
        int a = required_adc_id;
        // a delivered-cc seam means the transport hole is already accounted in
        // usb_cc_dropped - the stamp jump that comes with it is NOT a silent break
        bool cc_seam = rx_s.delivered_cc_valid[a] && lb->usb_cc != rx_s.delivered_cc[a] + 1;
        rx_s.delivered_cc[a] = lb->usb_cc;
        rx_s.delivered_cc_valid[a] = true;
        uint8_t pkt_epoch = (uint8_t)RFNM_STREAM_FLAG_EPOCH(lb->rx_flags);
        if (pkt_epoch != (uint8_t)(dev_status.rx_epoch & 0xFF)) {
            rx_s.phytimer_valid[a] = false;
            ses.rx_ext[a].reset();
        } else {
            if (!rx_s.phytimer_valid[a] || pkt_epoch != rx_s.phytimer_epoch[a]) {
                // fresh anchor: first validated packet of this stream generation
            } else if (lb->rx_flags & RFNM_RX_FLAG_DISCONT) {
                // clean device self-heal re-gate: the stamp jump is flagged and stays exact
                rx_s.phytimer_discont[a]++;
            } else if (cc_seam) {
                // stamp jump across a transport hole: usb_cc_dropped already carries this
                // loss; counting it as a break too would make get_rx_timing_health's "any
                // break is a bug" contract impossible to hold on a saturated link. Re-anchor.
            } else if (lb->phytimer != rx_s.expected_phytimer[a]) {
                // silent chain break: samples were lost somewhere nothing flagged and
                // nothing counted - a genuine protocol error
                rx_s.phytimer_break[a]++;
                if (rx_s.phytimer_break[a] <= 10 || (rx_s.phytimer_break[a] % 1000) == 0) {
                    spdlog::warn("phytimer chain break #{} adc {}: got {} expected {} (usb_cc {})",
                        rx_s.phytimer_break[a], a, lb->phytimer, rx_s.expected_phytimer[a], lb->usb_cc);
                }
            }
            rx_s.phytimer_valid[a] = true;
            rx_s.phytimer_epoch[a] = pkt_epoch;
            rx_s.expected_phytimer[a] = lb->phytimer +
                    (uint32_t)tick_ratio{ dev_status.rx_r_shift }.samples_to_ticks(lb->elem_cnt);

            // 32->64 tick extension: packets are usb_cc-ordered here, so raw stamps are
            // monotonic within an epoch - advance the extension owner
            ses.rx_ext[a].advance(lb->phytimer);
            // splice the live delivered<->tick map at this packet - the tick just past
            // its last sample. Tracks every gap/re-gate the moment it is delivered; an
            // init-frozen (t0 + count*R) construction cannot. out_mutex pairs this with
            // delivered_samples for tear-free reads.
            {
                std::lock_guard<std::mutex> lockGuard(rx_s.out_mutex);
                ses.delivered_end_tick[a] = ses.rx_ext[a].extend_near(lb->phytimer) +
                        tick_ratio{ dev_status.rx_r_shift }.samples_to_ticks(lb->elem_cnt);
                ses.delivered_map_valid[a] = true;
            }
        }
    }

    return RFNM_API_OK;
}

rfnm_api_failcode device::impl::rx_flush(uint32_t timeout_us, uint8_t ch_ids) {
    // the settle nap exists so in-flight packets land in the queues before the drain
    // below - that only happens while an RX worker is live. With no active worker it
    // is pure dead latency on every rx_stream::start (which flushes pre-apply).
    if (timeout_us) {
        bool rx_live = false;
        for (size_t i = 0; i < RFNM_WORKER_COUNT; i++) {
            std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
            if (thread_data[i].rx_active) {
                rx_live = true;
                break;
            }
        }
        if (rx_live) {
            std::this_thread::sleep_for(std::chrono::microseconds(timeout_us));
        }
    }

    for (size_t ch_id = 0; ch_id < MAX_RX_CHANNELS; ch_id++) {
        if (!(ch_ids & channel_flags[ch_id])) continue;

        int adc_id = rx.ch[ch_id].adc_id;

        std::lock_guard lock_out(rx_s.out_mutex);

        while (rx_s.out[adc_id].size()) {
            struct rx_buf* buf = rx_s.out[adc_id].top();
            rx_s.out[adc_id].pop();
            std::lock_guard lock_in(rx_s.in_mutex);
            rx_s.in.push(buf);
            rfnm_rx_in_nap_cv.notify_all();
        }

        rx_s.usb_cc[adc_id] = UINT64_MAX;
        rx_s.usb_cc_max_seen[adc_id] = 0;
        rx_s.delivered_cc_valid[adc_id] = false;
        rx_s.phytimer_valid[adc_id] = false;
        ses.rx_ext[adc_id].reset();
    }

    return RFNM_API_OK;
}

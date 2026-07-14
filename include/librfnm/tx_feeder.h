#pragma once

#include <cstdint>
#include <cstddef>

#include "device.h"

namespace rfnm {

    // The library-owned TX feed pump (v2 P2): buffer pool + recycle, packet staging with
    // short-flush, seek policies and the gated-session bootstrap - the machinery every
    // TX consumer used to hand-roll (OAI shim, vsgd, bench tools), owned once.
    //
    // The feeder speaks the FEED AXIS only (cumulative samples since the session's TX
    // anchor; position p airs at tx_t0 + p*R). Mapping a consumer time axis (OAI ts,
    // wall clock) onto the feed axis stays with the consumer. The device's feed
    // position is the ONLY truth - the feeder keeps no shadow counter, so an apply()
    // that rebases the axis is detected loudly instead of silently mis-stamping.
    //
    // Duplex USB note: on a duplex session the consumer must keep its RX stream
    // drained while feeder calls block (an unconsumed RX backlog re-gates the stream
    // and re-mints the TX epoch device-side). Production consumers do this by
    // construction (their RX path runs concurrently); single-threaded probes drain
    // around the blocking calls.
    class tx_feeder {
    public:
        // dev must outlive the feeder. pool_packets bounds the in-flight pipeline
        // (64 full packets ~= 21 ms at 61.44 Msps - deeper than any receipted lead).
        MSDLL explicit tx_feeder(device &dev, size_t pool_packets = 64,
                enum tx_latency_policy policy = TX_LATENCY_POLICY_RELAXED);
        MSDLL ~tx_feeder();

        tx_feeder(const tx_feeder&) = delete;
        tx_feeder& operator=(const tx_feeder&) = delete;

        // ---- session start (one per TX timeline; restart after an apply() whose
        // v5 timing handle reported a TX break) ----
        // Continuous feeder (the gNB shape): no positional intent declared, writes
        // flow from the current feed position.
        MSDLL rfnm_api_failcode start_free_run();
        // Positional at an absolute feed position (256-sample grid): declares intent
        // (unstampable writes are refused, never demoted) and pins the session anchor.
        MSDLL rfnm_api_failcode start_at(uint64_t abs_samples);
        // The gated-session bootstrap (the mode-1b / OAI-UE shape): primes a VIRGIN
        // pump (a published TX anchor proves the pump already runs - then no prime),
        // waits for the anchor (bounded), and seeks to the earliest placeable feed
        // position: now + max(margin_us, the device's published feed lead), converted
        // through the derived tick ratio, 256-floor, never backward. pos_out returns
        // the position writes start at. margin_us = 0 selects the receipted 2 ms
        // transit+scheduling allowance.
        MSDLL rfnm_api_failcode start_earliest(uint64_t *pos_out = nullptr, uint32_t margin_us = 0,
                uint32_t anchor_timeout_ms = 3000);
        // Stop feeding (workers off). Does NOT flush - a staged tail is the caller's
        // decision (flush() first for a burst tail, or drop it by stopping).
        MSDLL void stop();

        // ---- the write surface ----
        // Append samples (cs16 interleaved IQ) at the cursor; full packets ship as
        // they fill, the remainder stages. Blocks on device backpressure (bounded;
        // drops loudly past the budget - counted, never silent).
        MSDLL rfnm_api_failcode write(const int16_t *iq, size_t samples);
        // Burst write at an absolute feed position (the sparse/TDD shape): short-flushes
        // the staged tail (EOB - the kernel scrub then blanks past it, so stale ring
        // content never airs into the gap), seeks forward on the 256 grid, zero-fills
        // the sub-256 remainder, then appends. Backward positions are refused
        // (RFNM_API_SCHED_ORDER) - samples cannot be unwritten.
        MSDLL rfnm_api_failcode write_at(uint64_t abs_samples, const int16_t *iq, size_t samples);
        // Ship the staged partial packet now (256-padded; end_of_burst marks EOB so the
        // device scrubs ahead of the gap). No-op when nothing is staged.
        MSDLL rfnm_api_failcode flush(bool end_of_burst = true);

        // The feed cursor: device feed position (the only truth) + staged samples.
        MSDLL uint64_t pos();

        // Optional pacing for free-running producers (the vsgd shape): hold the feed
        // lead near the device's published contract instead of a folklore constant.
        // 0 = unpaced (default; device backpressure only). Receipted at the vsgd
        // migration - probes that must outrun the drain (latch bursts) stay unpaced.
        MSDLL void set_pace_lead_us(uint32_t lead_us);

        struct stats {
            uint64_t writes;            // write()/write_at() calls
            uint64_t packets;           // full packets shipped
            uint64_t short_flushes;     // staged tails shipped (EOB bursts)
            uint64_t seeks;             // forward seeks (gap scrubs)
            uint64_t zero_fill_samples; // sub-256 gap remainders fed as zeros
            uint64_t primes;            // virgin-pump prime packets
            uint64_t stalls;            // backpressure wait episodes
            uint64_t drops;             // packets dropped after the retry budget
            uint64_t rebases;           // feed-axis rebase detections (apply broke TX)
        };
        MSDLL void get_stats(stats *out);

    private:
        struct impl;
        impl *pimpl;
    };
}

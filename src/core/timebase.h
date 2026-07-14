#pragma once

// The timebase is a TYPE (v2 P1): every tick<->ns<->samples conversion and every
// raw32<->extended tick operation in the library lives here, pure and x86-unit-tested
// (tests/). Nothing in this header talks to a device.

#include <cstdint>
#include <librfnm/rfnm_fw_api.h>

namespace rfnm {

    // The phy timer tick rate is a DEVICE PROPERTY: dcs_clk/2, plan-dependent (61.44 MHz
    // on the 122.88 plan, 100 MHz on the 50M/DCS-200 plan), published in hwinfo.
    // tick_hz == 0 means the clock plan is not known yet; callers treat that as "no
    // timebase" and every conversion returns an honest 0 - never a substituted constant.
    struct timebase {
        uint64_t tick_hz = 0;

        static timebase from_hwinfo(const struct rfnm_dev_hwinfo &hw) {
            return timebase{ hw.clock.dcs_clk / 2 };
        }

        bool valid() const {
            return tick_hz != 0;
        }

        // exact conversions (128-bit intermediates; ns rounds to nearest)
        uint64_t ticks_to_ns(uint64_t ticks) const {
            if (!tick_hz) {
                return 0;
            }
            return (uint64_t)(((unsigned __int128)ticks * 1000000000ull + tick_hz / 2) / tick_hz);
        }

        uint64_t ns_to_ticks(uint64_t ns) const {
            if (!tick_hz) {
                return 0;
            }
            return (uint64_t)(((unsigned __int128)ns * tick_hz + 500000000ull) / 1000000000ull);
        }

        // whole microseconds -> ticks, floor semantics: duration FLOORS (e.g. the TDD v2
        // 600 us span minimum) are expressed in us and evaluated against this plan's rate
        uint64_t us_to_ticks_floor(uint64_t us) const {
            return (uint64_t)(((unsigned __int128)us * tick_hz) / 1000000ull);
        }
    };

    // Ticks per sample R = 2^r_shift / 2, exact on every plan. r_shift comes from the
    // device's APPLIED stream word (dev_status.rx_r_shift / tx_r_shift), never from a
    // cached or assumed plan. r_shift == 0 (the 122.88M full-rate plan) means R = 1/2:
    // samples_to_ticks rounds down half a tick for odd sample counts - the data path
    // only ever converts 256-sample multiples, where the shift-then-halve is exact.
    struct tick_ratio {
        uint32_t r_shift = 0;

        uint32_t r_num() const {
            return 1u << r_shift;
        }

        static constexpr uint32_t r_den() {
            return 2;
        }

        uint64_t samples_to_ticks(uint64_t samples) const {
            return (samples << r_shift) >> 1;
        }

        // DAC ring slots are 256 samples: one slot's tick span (256 * R = 128 << r_shift).
        // A power of two that divides 2^32 for every real plan, so slot-grid residues are
        // identical on raw u32 ticks and extended u64 ticks (truncation is exact).
        uint64_t ticks_per_slot() const {
            return 128ull << r_shift;
        }
    };

    // One VSPA chunk = 768 ADC samples; in ticks of the applied plan that is
    // 384 << rx_dcs_div (the ADC runs at dcs_clk >> rx_dcs_div, ticks at dcs_clk/2).
    // TDD patterns are chunk-aligned by structure.
    inline uint64_t rx_chunk_ticks(const struct rfnm_dev_hwinfo &hw) {
        return 384ull << hw.clock.rx_dcs_div;
    }

    // ONE owner for the stream-anchored 32->64 bit tick extension (the wrapping phy
    // timer counter, ~70 s period). advance() runs at the ordered dequeue point, where
    // stamps are monotonic per adc within an epoch: a backwards step of more than half
    // the range is a genuine counter wrap; smaller steps (forward gaps up to ~35 s,
    // small reorder regressions) pass through unchanged. extend_near() maps any stamp
    // within +-35 s of the newest advanced one into the extended domain.
    struct tick_extender {
        uint64_t high = 0;
        uint32_t last = 0;
        bool valid = false;

        void reset() {
            high = 0;
            last = 0;
            valid = false;
        }

        void advance(uint32_t raw) {
            if (!valid) {
                valid = true;
                high = 0;
                last = raw;
                return;
            }
            if (raw < last && (last - raw) > 0x80000000u) {
                high += 0x100000000ull;
            }
            last = raw;
        }

        // signed modular distance from the newest advanced stamp; identity when nothing
        // was ever advanced (callers treat the raw stamp as already-extended then)
        uint64_t extend_near(uint32_t stamp) const {
            if (!valid) {
                return stamp;
            }
            int64_t d = (int64_t)(int32_t)(stamp - last);
            return high + last + d;
        }

        uint64_t value() const {
            return high + last;
        }
    };

    // The self-extending unwrap for a sporadically POLLED counter (get_phytimer's
    // stream-less mode): each read folds the signed modular step from the previous
    // read into a running 64-bit value. Exact while polled at least every ~35 s.
    struct tick_self_extender {
        uint64_t ext = 0;
        bool valid = false;

        void reset() {
            ext = 0;
            valid = false;
        }

        uint64_t step(uint32_t raw) {
            if (!valid) {
                ext = raw;
                valid = true;
            } else {
                ext += (int64_t)(int32_t)(raw - (uint32_t)ext);
            }
            return ext;
        }
    };

    // Pad-to-exact plan for tx_buf_schedule (timed TX): the DAC gate can only open on
    // the ring slot grid, so a mid-slot tick opens one slot boundary below and leads
    // with zeros - the CONTENT airs at `tick` exactly. ok == false means the request
    // is not airable (SCHED_MISALIGNED): the tick falls between two samples of this
    // plan, or the buffer has no slack left for the pad.
    struct tx_pad_plan {
        bool ok = false;
        uint32_t lead_samples = 0;      // zeros ahead of the content
        uint32_t padded_elem_cnt = 0;   // buffer length after the pad (256-multiple)
        uint64_t gate_tick = 0;         // slot-aligned tick the gate opens at
    };

    inline tx_pad_plan plan_tx_pad(uint64_t tick, uint32_t t0_raw, tick_ratio r, uint32_t elem_cnt, uint32_t max_elem_cnt) {
        tx_pad_plan p;
        uint64_t off_ticks = (tick - t0_raw) % r.ticks_per_slot();
        p.gate_tick = tick - off_ticks;
        if (!off_ticks) {
            p.ok = true;
            p.padded_elem_cnt = elem_cnt;
            return p;
        }
        uint64_t lead2 = off_ticks * 2ull;
        uint64_t rs = 1ull << r.r_shift;
        if (lead2 % rs) {
            return p;   // tick falls between samples: not airable
        }
        uint32_t lead = (uint32_t)(lead2 / rs);
        uint32_t padded = (elem_cnt + lead + 255u) & ~255u;
        if (padded > max_elem_cnt) {
            return p;   // no slack for the pad: align the tick or shorten the burst
        }
        p.ok = true;
        p.lead_samples = lead;
        p.padded_elem_cnt = padded;
        return p;
    }
}

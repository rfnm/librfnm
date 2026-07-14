#include "unit.h"
#include "core/timebase.h"

using namespace rfnm;

// pad-to-exact + slot-grid congruence. The device works mod 2^32 (schedule requests
// truncate extended ticks to u32) - that truncation is exact BECAUSE every slot span
// is a power of two dividing 2^32, so residues computed on extended ticks and on
// truncated ticks agree. These tests pin both the plan math and that invariant.

static const uint32_t MAX_ELEMS = 20480;    // RFNM_USB_TX_PACKET_ELEM_CNT on the current wire

void test_pad_plan() {
    // slot-aligned tick: no pad, elem_cnt passes through (including 0 = full packet)
    {
        tx_pad_plan p = plan_tx_pad(/*tick*/ 1000 + 256, /*t0*/ 1000, tick_ratio{ 1 }, 512, MAX_ELEMS);
        CHECK(p.ok);
        CHECK_EQ(p.lead_samples, 0);
        CHECK_EQ(p.padded_elem_cnt, 512);
        CHECK_EQ(p.gate_tick, 1256);
    }

    // mid-slot tick on the 61.44M plan (r_shift 1, R = 1 tick/sample): 100 ticks into
    // a slot = 100 samples of zero lead, padded up to the 256 grid
    {
        tx_pad_plan p = plan_tx_pad(1000 + 256 + 100, 1000, tick_ratio{ 1 }, 512, MAX_ELEMS);
        CHECK(p.ok);
        CHECK_EQ(p.lead_samples, 100);
        CHECK_EQ(p.padded_elem_cnt, 768);   // 512 + 100 -> 768
        CHECK_EQ(p.gate_tick, 1256);
    }

    // r_shift 0 (R = 1/2): every tick is 2 samples; ticks_per_slot = 128
    {
        tx_pad_plan p = plan_tx_pad(2000 + 128 + 3, 2000, tick_ratio{ 0 }, 256, MAX_ELEMS);
        CHECK(p.ok);
        CHECK_EQ(p.lead_samples, 6);
        CHECK_EQ(p.padded_elem_cnt, 512);   // 256 + 6 -> 512
        CHECK_EQ(p.gate_tick, 2128);
    }

    // r_shift 4 (R = 8): a tick offset not divisible by 8 falls between samples
    {
        tx_pad_plan p = plan_tx_pad(4096 + 12, 4096, tick_ratio{ 4 }, 256, MAX_ELEMS);
        CHECK(!p.ok);
    }
    // ... and a divisible one is airable
    {
        tx_pad_plan p = plan_tx_pad(4096 + 16, 4096, tick_ratio{ 4 }, 256, MAX_ELEMS);
        CHECK(p.ok);
        CHECK_EQ(p.lead_samples, 2);        // 16 ticks at R = 8 ticks/sample
        CHECK_EQ(p.padded_elem_cnt, 512);
    }

    // no slack: a full packet with a misaligned tick is rejected
    {
        tx_pad_plan p = plan_tx_pad(1000 + 256 + 100, 1000, tick_ratio{ 1 }, MAX_ELEMS, MAX_ELEMS);
        CHECK(!p.ok);
    }
    // exactly-fitting pad is accepted (padded == max)
    {
        tx_pad_plan p = plan_tx_pad(1000 + 256 + 100, 1000, tick_ratio{ 1 }, MAX_ELEMS - 256, MAX_ELEMS);
        CHECK(p.ok);
        CHECK_EQ(p.padded_elem_cnt, MAX_ELEMS);
    }

    // congruence: adding whole 2^32 spans to the extended tick changes NOTHING in the
    // plan (u32 VALUE residues - the wire truncation is exact by construction)
    for (uint32_t sh = 0; sh <= 6; sh++) {
        tx_pad_plan a = plan_tx_pad(3000 + 77 * (1ull << sh) * 2, 3000, tick_ratio{ sh }, 512, MAX_ELEMS);
        tx_pad_plan b = plan_tx_pad(3000 + 77 * (1ull << sh) * 2 + 0x100000000ull, 3000, tick_ratio{ sh }, 512, MAX_ELEMS);
        CHECK_EQ(a.ok, b.ok);
        CHECK_EQ(a.lead_samples, b.lead_samples);
        CHECK_EQ(a.padded_elem_cnt, b.padded_elem_cnt);
        CHECK_EQ((uint32_t)a.gate_tick, (uint32_t)b.gate_tick);
    }

    // congruence across a wrap: an extended tick whose u32 truncation is BELOW the
    // raw t0 still resolves to the same slot residue (unsigned mod arithmetic)
    {
        uint64_t tick_ext = 0x100000000ull + 500 * 256;     // one era up, slot-aligned vs t0 below
        tx_pad_plan p = plan_tx_pad(tick_ext, 0, tick_ratio{ 1 }, 512, MAX_ELEMS);
        CHECK(p.ok);
        CHECK_EQ(p.lead_samples, 0);
    }
}

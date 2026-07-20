#include "unit.h"
#include "core/timebase.h"

using namespace rfnm;

// The two real bench plans: 122.88 plan (dcs 122.88M -> tick 61.44 MHz) and the
// 50M/DCS-200 plan (dcs 200M -> tick 100 MHz). The tick rate is dcs_clk/2 and NOTHING
// else - these tests hold on both plans precisely so no 61.44-flavored constant can
// creep back in as "the" clock.
static timebase tb_6144() { return timebase{ 61440000ull }; }
static timebase tb_100m() { return timebase{ 100000000ull }; }

void test_timebase() {
    // muldiv_u64 pins: the portable 128-bit (a*b+add)/div under every conversion.
    // The >64-bit-product cases execute the MSVC intrinsic/fallback paths on Windows
    // CI (this exact helper replaced __int128, which MSVC lacks - C4235 receipt).
    CHECK_EQ(muldiv_u64(6, 7, 0, 2), 21ull);
    CHECK_EQ(muldiv_u64(10, 10, 5, 10), 10ull);                          // rounding add
    CHECK_EQ(muldiv_u64(0xffffffffffffffffull, 0xffffffffffffffffull, 0,
                        0xffffffffffffffffull), 0xffffffffffffffffull);  // max*max/max, hi != 0
    CHECK_EQ(muldiv_u64(0x123456789abcdefull, 1000000000ull, 0,
                        61440000ull), 1334399889591258056ull);           // ticks_to_ns shape, hi != 0
    CHECK_EQ(muldiv_u64(1, 1, 0xfffffffffffffffeull, 0xffffffffffffffffull), 1ull);  // add carries to quotient
    CHECK_EQ(muldiv_u64(2, 0x8000000000000000ull, 0, 2), 0x8000000000000000ull);     // product exactly 2^64

    // hwinfo is the ONLY tick_hz source: dcs_clk/2, zero-until-published
    struct rfnm_dev_hwinfo hw = {};
    CHECK(!timebase::from_hwinfo(hw).valid());
    hw.clock.dcs_clk = 122880000ull;
    CHECK_EQ(timebase::from_hwinfo(hw).tick_hz, 61440000ull);
    hw.clock.dcs_clk = 200000000ull;
    CHECK_EQ(timebase::from_hwinfo(hw).tick_hz, 100000000ull);

    // invalid timebase converts to honest zeros, never a constant guess
    timebase dead = {};
    CHECK(!dead.valid());
    CHECK_EQ(dead.ticks_to_ns(123456), 0);
    CHECK_EQ(dead.ns_to_ticks(123456), 0);

    // exact conversions, nearest-rounded ns
    CHECK_EQ(tb_6144().ticks_to_ns(61440000ull), 1000000000ull);        // 1 s
    CHECK_EQ(tb_100m().ticks_to_ns(100000000ull), 1000000000ull);
    CHECK_EQ(tb_6144().ticks_to_ns(1), 16);                             // 16.276 ns -> 16
    CHECK_EQ(tb_6144().ticks_to_ns(2), 33);                             // 32.552 -> 33
    CHECK_EQ(tb_100m().ticks_to_ns(1), 10);
    CHECK_EQ(tb_6144().ns_to_ticks(1000000000ull), 61440000ull);
    CHECK_EQ(tb_100m().ns_to_ticks(1000000000ull), 100000000ull);
    // nearest rounding on the way in: 8.14 ns = 0.5 ticks at 61.44M rounds up
    CHECK_EQ(tb_6144().ns_to_ticks(9), 1);      // 0.553 ticks -> 1
    CHECK_EQ(tb_6144().ns_to_ticks(8), 0);      // 0.491 ticks -> 0

    // 128-bit intermediates: no overflow at large tick counts (days of uptime)
    uint64_t week_ticks = 61440000ull * 3600ull * 24ull * 7ull;
    CHECK_EQ(tb_6144().ticks_to_ns(week_ticks), 604800000000000ull);
    CHECK_EQ(tb_6144().ns_to_ticks(604800000000000ull), week_ticks);

    // duration floors are us-denominated and evaluated per plan: the TDD v2 600 us
    // span floor is 36864 ticks on the 61.44M plan but 60000 on the 100M plan (the
    // retired 245760-tick relic was 4 ms ONLY at 61.44M - the class this type kills)
    CHECK_EQ(tb_6144().us_to_ticks_floor(600), 36864);
    CHECK_EQ(tb_100m().us_to_ticks_floor(600), 60000);

    // chunk ticks come from the hwinfo divider: 768 ADC samples = 384 << rx_dcs_div
    hw.clock.rx_dcs_div = 0;
    CHECK_EQ(rx_chunk_ticks(hw), 384);
    hw.clock.rx_dcs_div = 1;
    CHECK_EQ(rx_chunk_ticks(hw), 768);
}

void test_tick_ratio() {
    // R = 2^r_shift / 2 ticks per sample, exact
    CHECK_EQ(tick_ratio{ 0 }.r_num(), 1);
    CHECK_EQ(tick_ratio{ 0 }.r_den(), 2);
    CHECK_EQ(tick_ratio{ 4 }.r_num(), 16);

    // 256-sample packets across the whole real shift range
    for (uint32_t sh = 0; sh <= 9; sh++) {
        tick_ratio r{ sh };
        CHECK_EQ(r.samples_to_ticks(256), 128ull << sh);
        CHECK_EQ(r.ticks_per_slot(), 128ull << sh);
        // slot span divides 2^32: raw-u32 slot residues == extended-u64 slot residues
        CHECK_EQ((0x100000000ull % r.ticks_per_slot()), 0);
    }

    // r_shift 0 (122.88M full rate): R = 1/2, odd counts round down half a tick
    CHECK_EQ(tick_ratio{ 0 }.samples_to_ticks(3), 1);
    CHECK_EQ(tick_ratio{ 0 }.samples_to_ticks(2), 1);
    // r_shift 1 (61.44M): 1 tick per sample
    CHECK_EQ(tick_ratio{ 1 }.samples_to_ticks(12345), 12345);
    // r_shift 4: 8 ticks per sample
    CHECK_EQ(tick_ratio{ 4 }.samples_to_ticks(100), 800);
}

// Gated-delivery lead projection (r12p2 physics): feed_lead = delivered_lead * P/duty.
void test_gated_lead() {
    // the OAI r12p2 receipt numbers: 400/320 pattern, growth == exactly (P-duty)/duty = 0.25
    CHECK_EQ(gated_feed_lead(35200000, 400, 320), 44000000);
    CHECK_EQ(gated_feed_lead(35200000, 400, 320) - 35200000, 8800000);
    // ratio is unitless: same result from tick-domain params (400/320 chunks * 768 ticks)
    CHECK_EQ(gated_feed_lead(35200000, 307200, 245760), 44000000);
    // wall-form identity: growth vs wall lead = (P-duty)/P; wall = delivered * P/duty
    // -> for P=400 duty=320: wall 44.0M x 0.2 == delivered growth 8.8M
    CHECK_EQ(44000000ull * (400 - 320) / 400, 8800000);
    // no gating = identity; malformed (duty > period) never projects
    CHECK_EQ(gated_feed_lead(12345, 400, 400), 12345);
    CHECK_EQ(gated_feed_lead(12345, 400, 500), 12345);
    CHECK_EQ(gated_feed_lead(12345, 400, 0), 12345);
    // floor on non-divisible: 100 * 3/2 = 150; 101 * 3/2 = 151 (floor of 151.5)
    CHECK_EQ(gated_feed_lead(101, 3, 2), 151);
    // u128 path: no overflow near the top of u64
    CHECK_EQ(gated_feed_lead(1ull << 62, 4, 2), 1ull << 63);
}

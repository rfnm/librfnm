#include "unit.h"
#include "core/timebase.h"

using namespace rfnm;

void test_extender() {
    // stream extension: first advance anchors, forward steps accumulate no wrap
    tick_extender e;
    CHECK_EQ(e.extend_near(0x1234), 0x1234);    // identity before any advance
    e.advance(1000);
    CHECK(e.valid);
    CHECK_EQ(e.value(), 1000);
    e.advance(2000);
    CHECK_EQ(e.value(), 2000);
    CHECK_EQ(e.extend_near(1500), 1500);        // near-past stamp, same era
    CHECK_EQ(e.extend_near(2500), 2500);        // near-future stamp

    // a genuine counter wrap: backwards step of more than half the range
    e.reset();
    e.advance(0xFFFFFF00u);
    e.advance(0x00000100u);                     // wrapped
    CHECK_EQ(e.value(), 0x100000100ull);
    // stamps from just before the wrap still extend into the previous era
    CHECK_EQ(e.extend_near(0xFFFFFF80u), 0xFFFFFF80ull);
    // stamps after the wrap land in the new era
    CHECK_EQ(e.extend_near(0x00000200u), 0x100000200ull);

    // small BACKWARD steps (reorder regressions) are not wraps
    e.reset();
    e.advance(5000);
    e.advance(4000);
    CHECK_EQ(e.value(), 4000);

    // forward gaps up to ~35 s (< half range) pass through unchanged - and larger
    // forward jumps too (scheduled windows): only backward-past-half counts as wrap
    e.reset();
    e.advance(0x10000000u);
    e.advance(0xA0000000u);                     // +0x90000000 forward jump, no wrap added
    CHECK_EQ(e.value(), 0xA0000000ull);

    // two consecutive wraps accumulate
    e.reset();
    e.advance(0xFFFFFFF0u);
    e.advance(0x10u);
    e.advance(0xFFFFFFF0u);                     // forward to just below the next wrap
    e.advance(0x10u);
    CHECK_EQ(e.value(), 0x200000010ull);

    // self-extending unwrap (the polled-counter idiom): each poll folds the SIGNED
    // modular step from the previous read, so a monotonic counter polled at least
    // every ~35 s extends exactly - and any apparent jump of more than half the range
    // reads as the shorter (possibly backward) step by design
    tick_self_extender p;
    CHECK_EQ(p.step(100), 100);
    CHECK_EQ(p.step(200), 200);
    p.reset();
    CHECK(!p.valid);
    CHECK_EQ(p.step(0xFFFFFF00u), 0xFFFFFF00ull);
    CHECK_EQ(p.step(0x00000100u), 0x100000100ull);  // wrap folded in (+0x200 signed step)
    CHECK_EQ(p.step(0x00000080u), 0x100000080ull);  // small backward poll goes back
    p.reset();
    CHECK_EQ(p.step(7), 7);
}

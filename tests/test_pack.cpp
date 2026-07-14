#include <cstring>
#include <cmath>

#include "unit.h"
#include "core/pack.h"

using namespace rfnm;

// Golden vectors for the 12-bit wire codec. Contract: the wire carries bits [15:4]
// of each cs16 value (pack drops the low 4 bits, unpack restores them as zeros),
// cs8 keeps bits [15:8], cf32 = cs16 / 32767. Buffers use the device granularities
// the vectorizer hints promise (multiples of 64/256 samples).

static const size_t N = 256;    // samples (I+Q pairs)

// distinctive cs16 pattern, low nibble deliberately non-zero on some entries so the
// 12-bit truncation is visible
static void fill_pattern(int16_t *iq) {
    const int16_t vals[] = { 0x0120, -0x0120, 0x7FF0, -0x7FF0, 0x0010, -0x0010, 0x0123, -0x4567 };
    for (size_t s = 0; s < N; s++) {
        iq[2 * s + 0] = vals[s % 8];
        iq[2 * s + 1] = (int16_t)~vals[(s + 3) % 8];
    }
}

void test_pack() {
    static int16_t src[N * 2];
    static uint8_t wire[N * 3];
    static int16_t back16[N * 2];
    static int8_t back8[N * 2];
    static float backf[N * 2];

    fill_pattern(src);

    // ---- pack golden bytes (first two samples, hand-computed) ----
    // sample0: i=0x0120 q=~0x7FF0=0x800F -> 12-bit i=0x012 q=0x800>>4=0x800... q>>4
    // (arith) of int16 0x800F = 0xF800; wire triplet = [i&0xFF, (i>>8)&0xF | (q<<4)&0xF0, (q>>4)&0xFF]
    pack_cs16_to_12(wire, (uint8_t*)src, N);
    {
        int16_t i0 = src[0] >> 4;   // 0x0120 >> 4 = 0x012
        int16_t q0 = src[1] >> 4;   // arithmetic
        CHECK_EQ(wire[0], (uint8_t)(i0 & 0xFF));
        CHECK_EQ(wire[1], (uint8_t)(((i0 >> 8) & 0xF) | ((q0 << 4) & 0xF0)));
        CHECK_EQ(wire[2], (uint8_t)((q0 >> 4) & 0xFF));
        int16_t i1 = src[2] >> 4;
        CHECK_EQ(wire[3], (uint8_t)(i1 & 0xFF));
    }

    // ---- round trip: cs16 -> 12 -> cs16 == original with the low 4 bits zeroed ----
    CHECK(unpack_12_to_cs16((uint8_t*)back16, wire, N));
    for (size_t v = 0; v < N * 2; v++) {
        int16_t expect = (int16_t)(src[v] & ~0xF);   // truncation toward -inf: -0x4567 -> -0x4570
        if (back16[v] != expect) {
            CHECK_EQ(back16[v], expect);
            break;
        }
    }

    // ---- 12 -> cs8 keeps bits [15:8] of the restored value ----
    CHECK(unpack_12_to_cs8((uint8_t*)back8, wire, N));
    for (size_t v = 0; v < N * 2; v++) {
        int8_t expect = (int8_t)(((int16_t)(src[v] & ~0xF)) >> 8);
        if (back8[v] != expect) {
            CHECK_EQ(back8[v], expect);
            break;
        }
    }

    // ---- 12 -> cf32 = restored cs16 / 32767 ----
    CHECK(unpack_12_to_cf32((uint8_t*)backf, wire, N));
    for (size_t v = 0; v < N * 2; v++) {
        float expect = ((int16_t)(src[v] & ~0xF)) / 32767.0f;
        if (std::fabs(backf[v] - expect) > 1e-9f) {
            CHECK(std::fabs(backf[v] - expect) <= 1e-9f);
            break;
        }
    }

    // ---- odd sample counts are refused by the unpack family ----
    CHECK(!unpack_12_to_cs16((uint8_t*)back16, wire, 3));
    CHECK(!unpack_12_to_cs8((uint8_t*)back8, wire, 3));
    CHECK(!unpack_12_to_cf32((uint8_t*)backf, wire, 3));

    // ---- local-transport cs16 converters (no 12-bit stage, full precision) ----
    static int16_t conv16[N * 2];
    CHECK(convert_cs16_to_cs16((uint8_t*)conv16, (uint8_t*)src, N));
    CHECK(memcmp(conv16, src, sizeof(src)) == 0);

    CHECK(convert_cs16_to_cs8((uint8_t*)back8, (uint8_t*)src, N));
    for (size_t v = 0; v < N * 2; v++) {
        int8_t expect = (int8_t)(src[v] >> 8);
        if (back8[v] != expect) {
            CHECK_EQ(back8[v], expect);
            break;
        }
    }

    CHECK(convert_cs16_to_cf32((uint8_t*)backf, (uint8_t*)src, N));
    for (size_t v = 0; v < N * 2; v++) {
        float expect = src[v] / 32767.0f;
        if (std::fabs(backf[v] - expect) > 1e-9f) {
            CHECK(std::fabs(backf[v] - expect) <= 1e-9f);
            break;
        }
    }
}

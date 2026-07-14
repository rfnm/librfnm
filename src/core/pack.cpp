#include <cstring>

#include "pack.h"

// Wire packing note: samples travel as 12-bit pairs packed 2-per-3-bytes, decoded via
// unaligned 64-bit loads - little-endian hosts only (x86 + the aarch64 boards). The
// __builtin_unreachable hints promise the device's transfer granularities (64-sample
// multiples from the variable-size packet sub granularity, 256-sample multiples on the
// TX ring grid) so the vectorizer drops its peel loops.

bool rfnm::unpack_12_to_cs16(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
    uint64_t buf{};
    uint64_t r0{};
    uint64_t* dest_64{};
    uint64_t* src_64{};

    if (sample_cnt % 2) {
        return false;
    }

#ifdef __has_builtin
    if (sample_cnt & 63)
        __builtin_unreachable();
#endif

    // process two samples at a time
    sample_cnt = sample_cnt / 2;
    for (size_t c = 0; c < sample_cnt; c++) {
        src_64 = (uint64_t*)((uint8_t*)src + (c * 6));
        buf = *(src_64);
        r0 = 0;
        r0 |= (buf & (0xfffll << 0)) << 4;
        r0 |= (buf & (0xfffll << 12)) << 8;
        r0 |= (buf & (0xfffll << 24)) << 12;
        r0 |= (buf & (0xfffll << 36)) << 16;

        dest_64 = (uint64_t*)(dest + (c * 8));
        *dest_64 = r0;
    }
    return true;
}

bool rfnm::unpack_12_to_cf32(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
    uint64_t buf{};

    if (sample_cnt % 2) {
        return false;
    }

    // process two samples at a time
    sample_cnt = sample_cnt / 2;
    for (size_t c = 0; c < sample_cnt; c++) {
        uint64_t* src_64 = (uint64_t*)((uint8_t*)src + (c * 6));
        buf = *(src_64);

        float* i1, * i2, * q1, * q2;

        i1 = (float*)((uint8_t*)dest + (c * 16) + 0);
        q1 = (float*)((uint8_t*)dest + (c * 16) + 4);
        i2 = (float*)((uint8_t*)dest + (c * 16) + 8);
        q2 = (float*)((uint8_t*)dest + (c * 16) + 12);

        *i1 = ((int16_t)((buf & (0xfffll << 0)) << 4)) / 32767.0f;
        *q1 = ((int16_t)((buf & (0xfffll << 12)) >> 8)) / 32767.0f;
        *i2 = ((int16_t)((buf & (0xfffll << 24)) >> 20)) / 32767.0f;
        *q2 = ((int16_t)((buf & (0xfffll << 36)) >> 32)) / 32767.0f;
    }
    return true;
}

bool rfnm::unpack_12_to_cs8(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
    uint64_t buf{};
    uint32_t r0{};
    uint32_t* dest_32{};
    uint64_t* src_64{};

    if (sample_cnt % 2) {
        return false;
    }

    // process two samples at a time
    sample_cnt = sample_cnt / 2;
    for (size_t c = 0; c < sample_cnt; c++) {
        src_64 = (uint64_t*)((uint8_t*)src + (c * 6));
        buf = *(src_64);
        r0 = 0;
        r0 |= (buf & (0xffll << 4)) >> 4;
        r0 |= (buf & (0xffll << 16)) >> 8;
        r0 |= (buf & (0xffll << 28)) >> 12;
        r0 |= (buf & (0xffll << 40)) >> 16;

        dest_32 = (uint32_t*)((uint8_t*)dest + (c * 4));
        *dest_32 = r0;
    }
    return true;
}

void rfnm::pack_cs16_to_12(uint8_t* dest, uint8_t* src8, int sample_cnt) {
#ifndef __has_builtin
    uint64_t buf;
    uint64_t r0;
    int32_t c;
    uint64_t* dest_64;
    uint64_t* src;

    src = (uint64_t*)src8;
    sample_cnt = sample_cnt / 2;

    for (c = 0; c < sample_cnt; c++) {
        buf = *(src + c);
        r0 = 0;
        r0 |= (buf & (0xfffll << 4)) >> 4;
        r0 |= (buf & (0xfffll << 20)) >> 8;
        r0 |= (buf & (0xfffll << 36)) >> 12;
        r0 |= (buf & (0xfffll << 52)) >> 16;

        dest_64 = (uint64_t*)(dest + (c * 6));
        *dest_64 = r0;
    }
#else
    uint16_t *src16 = (uint16_t *) src8;
    sample_cnt *= 4;
    if (sample_cnt & 255)
        __builtin_unreachable();

    while (sample_cnt >= 4) {
        int16_t i = (*src16++) >> 4;
        int16_t q = (*src16++) >> 4;
        *dest++ = i & 0xFF;
        *dest++ = ((i >> 8) & 0xF) | ((q << 4) & 0xF0);
        *dest++ = (q >> 4) & 0xFF;
        sample_cnt -= 4;
    }
#endif
}

// cs16 payload converters for the local transport: the kernel already unpacked
// 12->16 (NEON) into the local ring, so the only remaining pass is the output
// format conversion. Scale conventions match the unpack_12_to_* family.
bool rfnm::convert_cs16_to_cs16(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
    memcpy(dest, src, sample_cnt * 4);
    return true;
}

bool rfnm::convert_cs16_to_cf32(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
    const int16_t* __restrict s16 = (const int16_t*)src;
    float* __restrict f = (float*)dest;
    size_t vals = sample_cnt * 2;
#ifdef __has_builtin
    if (vals & 127)
        __builtin_unreachable();
#endif
    for (size_t c = 0; c < vals; c++) {
        f[c] = s16[c] / 32767.0f;
    }
    return true;
}

bool rfnm::convert_cs16_to_cs8(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
    const int16_t* __restrict s16 = (const int16_t*)src;
    int8_t* __restrict d = (int8_t*)dest;
    size_t vals = sample_cnt * 2;
#ifdef __has_builtin
    if (vals & 127)
        __builtin_unreachable();
#endif
    for (size_t c = 0; c < vals; c++) {
        d[c] = (int8_t)(s16[c] >> 8);
    }
    return true;
}

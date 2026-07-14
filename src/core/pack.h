#pragma once

// The 12-bit wire codec + the local-transport cs16 output converters. Pure functions,
// x86-unit-tested against golden vectors (tests/). Scale conventions across the family:
// the 12-bit wire carries bits [15:4] of the cs16 sample (the low 4 bits are dropped on
// pack and restored as zeros on unpack); cs8 keeps bits [15:8]; cf32 is cs16 / 32767.

#include <cstdint>
#include <cstddef>

namespace rfnm {
    bool unpack_12_to_cs16(uint8_t* dest, uint8_t* src, size_t sample_cnt);
    bool unpack_12_to_cf32(uint8_t* dest, uint8_t* src, size_t sample_cnt);
    bool unpack_12_to_cs8(uint8_t* dest, uint8_t* src, size_t sample_cnt);
    void pack_cs16_to_12(uint8_t* dest, uint8_t* src8, int sample_cnt);

    // local-transport cs16 payloads: the kernel already unpacked 12->16 (NEON) into the
    // local ring, so only the output format conversion remains on this SoC
    bool convert_cs16_to_cs16(uint8_t* dest, uint8_t* src, size_t sample_cnt);
    bool convert_cs16_to_cf32(uint8_t* dest, uint8_t* src, size_t sample_cnt);
    bool convert_cs16_to_cs8(uint8_t* dest, uint8_t* src, size_t sample_cnt);
}

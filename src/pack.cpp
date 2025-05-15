#include <algorithm>
#include <librfnm/device.h>
#include <librfnm/rx_stream.h>

using namespace rfnm;

MSDLL bool device::unpack_12_to_cs16(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
    //#ifndef __has_builtin
        uint64_t buf{};
        uint64_t r0{};
        uint64_t* dest_64{};
        uint64_t* src_64{};
    
        if (sample_cnt % 2) {
        //    spdlog::error("RFNM::Conversion::unpack12to16() -> sample_cnt {} is not divisible by 2", sample_cnt);
            return false;
        }

        if (sample_cnt & 255)
            __builtin_unreachable();
    
        // process two samples at a time
        sample_cnt = sample_cnt / 2;
        for (size_t c = 0; c < sample_cnt; c++) {
            src_64 = (uint64_t*)((uint8_t*)src + (c * 6));
            buf = *(src_64); //unaligned read?
            r0 = 0;
            r0 |= (buf & (0xfffll << 0)) << 4;
            r0 |= (buf & (0xfffll << 12)) << 8;
            r0 |= (buf & (0xfffll << 24)) << 12;
            r0 |= (buf & (0xfffll << 36)) << 16;
    
            dest_64 = (uint64_t*)(dest + (c * 8));
            *dest_64 = r0;
        }
        return true;
    /*#else
        // must be a multiple of 256 samples for a peel-free NEON loop
        if (sample_cnt & 255)
        __builtin_unreachable();
    
        // we consume 2 output samples per iteration
        size_t pairs = sample_cnt >> 1;
    
        // promise 16-byte alignment for src and dest
        const uint8_t* __restrict s = (const uint8_t*)
            __builtin_assume_aligned((void*)src, 16);
        int16_t* __restrict d = (int16_t*)
            __builtin_assume_aligned((void*)dest, 16);
    
        #if defined(__clang__)
        #pragma clang loop vectorize_width(16) interleave_count(4)
        #elif defined(__GNUC__)
        #pragma GCC ivdep
        #endif
        for (size_t i = 0; i < pairs; i++) {
            uint8_t lo  = s[0];
            uint8_t mid = s[1];
            uint8_t hi  = s[2];
            s += 3;
    
            // rebuild two 12-bit signed values
            int16_t x12 = (int16_t)(((mid & 0x0F) << 8) | lo);
            int16_t y12 = (int16_t)((hi << 4)        | (mid >> 4));
    
            // sign-extend 12→16 and shift left 4 (low nibble = 0)
            d[0] = ((x12 << 4) >> 4) << 4;
            d[1] = ((y12 << 4) >> 4) << 4;
            d += 2;
        }
    
        return true;
    
    
    
        
    
    #endif*/
    }
    
    MSDLL bool device::unpack_12_to_cf32(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
        uint64_t buf{};
        uint64_t r0{};
        uint64_t* dest_64{};
        uint64_t* src_64{};
    
        if (sample_cnt % 2) {
        //    spdlog::error("RFNM::Conversion::unpack12to16() -> sample_cnt {} is not divisible by 2", sample_cnt);
            return false;
        }
    
        // process two samples at a time
        sample_cnt = sample_cnt / 2;
        for (size_t c = 0; c < sample_cnt; c++) {
            src_64 = (uint64_t*)((uint8_t*)src + (c * 6));
            buf = *(src_64); //unaligned read?
    
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
    
    MSDLL bool device::unpack_12_to_cs8(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
        uint64_t buf{};
        uint32_t r0{};
        uint32_t* dest_32{};
        uint64_t* src_64{};
    
        if (sample_cnt % 2) {
        //    spdlog::error("RFNM::Conversion::unpack12to16() -> sample_cnt {} is not divisible by 2", sample_cnt);
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
    
    MSDLL void device::pack_cs16_to_12(uint8_t* dest, uint8_t* src8, int sample_cnt) {
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
    
#pragma once

// Minimal dependency-free check harness for the x86 unit layer: the pure functions
// (timebase math, tick extension, pad/congruence plans, wire pack/unpack) are pinned
// here BEFORE internal callers rely on them. Run via ctest or ./rfnm_unit directly.

#include <cstdio>
#include <cstdint>

extern int g_failures;

#define CHECK(cond) do { \
    if (!(cond)) { \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        g_failures++; \
    } \
} while (0)

#define CHECK_EQ(a, b) do { \
    unsigned long long _a = (unsigned long long)(a); \
    unsigned long long _b = (unsigned long long)(b); \
    if (_a != _b) { \
        std::printf("FAIL %s:%d: %s == %s (%llu vs %llu)\n", __FILE__, __LINE__, #a, #b, _a, _b); \
        g_failures++; \
    } \
} while (0)

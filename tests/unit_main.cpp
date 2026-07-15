#include "unit.h"

int g_failures = 0;

void test_timebase();
void test_tick_ratio();
void test_extender();
void test_pad_plan();
void test_gated_lead();
void test_pack();
void test_errors();

int main() {
    test_timebase();
    test_tick_ratio();
    test_extender();
    test_pad_plan();
    test_gated_lead();
    test_pack();
    test_errors();

    if (g_failures) {
        std::printf("rfnm_unit: %d FAILURE(S)\n", g_failures);
        return 1;
    }
    std::printf("rfnm_unit: all checks passed\n");
    return 0;
}

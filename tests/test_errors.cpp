#include "unit.h"
#include <librfnm/device.h>

using namespace rfnm;

void test_errors() {
    // the corrected names alias the PINNED v5 values (wire ecodes carry these
    // numbers - they must not move until the v6 break)
    CHECK_EQ(RFNM_API_WOULD_BLOCK, RFNM_API_MIN_QBUF_QUEUE_FULL);
    CHECK_EQ(RFNM_API_WOULD_BLOCK, 11);
    CHECK_EQ(RFNM_API_MIN_QBUF_CNT_NOT_SATISFIED, RFNM_API_MIN_QBUF_CNT_NOT_SATIFIED);
    CHECK_EQ(RFNM_API_MIN_QBUF_CNT_NOT_SATISFIED, 10);

    CHECK_EQ(failcode_class(RFNM_API_OK), ERR_OK);

    CHECK_EQ(failcode_class(RFNM_API_PROBE_FAIL), ERR_VALIDATION);
    CHECK_EQ(failcode_class(RFNM_API_TUNE_FAIL), ERR_VALIDATION);
    CHECK_EQ(failcode_class(RFNM_API_GAIN_FAIL), ERR_VALIDATION);
    CHECK_EQ(failcode_class(RFNM_API_NOT_SUPPORTED), ERR_VALIDATION);

    CHECK_EQ(failcode_class(RFNM_API_USB_FAIL), ERR_TRANSPORT_TERMINAL);
    CHECK_EQ(failcode_class(RFNM_API_RX_PIPE_DEAD), ERR_TRANSPORT_TERMINAL);
    CHECK_EQ(failcode_class(RFNM_API_SW_UPGRADE_REQUIRED), ERR_TRANSPORT_TERMINAL);

    CHECK_EQ(failcode_class(RFNM_API_WOULD_BLOCK), ERR_FLOW);
    CHECK_EQ(failcode_class(RFNM_API_DQBUF_NO_DATA), ERR_FLOW);
    CHECK_EQ(failcode_class(RFNM_API_DQBUF_OVERFLOW), ERR_FLOW);
    CHECK_EQ(failcode_class(RFNM_API_TIMEOUT), ERR_FLOW);

    CHECK_EQ(failcode_class(RFNM_API_SCHED_NO_ANCHOR), ERR_SCHEDULE);
    CHECK_EQ(failcode_class(RFNM_API_SCHED_MISALIGNED), ERR_SCHEDULE);
    CHECK_EQ(failcode_class(RFNM_API_SCHED_LATE), ERR_SCHEDULE);
    CHECK_EQ(failcode_class(RFNM_API_SCHED_FULL), ERR_SCHEDULE);
    CHECK_EQ(failcode_class(RFNM_API_SCHED_ORDER), ERR_SCHEDULE);
    CHECK_EQ(failcode_class(RFNM_API_SCHED_NOT_RESET), ERR_SCHEDULE);

    CHECK_EQ(failcode_class(RFNM_API_TX_NOT_ANCHORED), ERR_STATE);
    CHECK_EQ(failcode_class(RFNM_API_MIN_QBUF_CNT_NOT_SATISFIED), ERR_STATE);
}

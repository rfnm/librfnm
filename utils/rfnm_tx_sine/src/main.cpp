#include <stdio.h>
#include <librfnm/device.h>
#include <librfnm/constants.h>
#include <math.h>

#define NBUF 300

int main() {

    auto lrfnm = new rfnm::device(rfnm::TRANSPORT_FIND);
    lrfnm->set_dcs(122880000 / 2);

    lrfnm->set_tx_channel_active(0, RFNM_CH_ON, RFNM_CH_STREAM_ON, false);
    lrfnm->set(rfnm::tx_channel_apply_flags[0]);

    auto stream_format = rfnm::STREAM_FORMAT_CS16;

    int inbufsize = RFNM_USB_TX_PACKET_ELEM_CNT * lrfnm->get_transport_status()->tx_stream_format;
    
    std::queue<struct rfnm::tx_buf*> ltxqueue;

    uint8_t* s[NBUF];

    struct rfnm::tx_buf txbuf[NBUF];

    uint16_t* d;
    static double t;
    float progressive = 0.2;
    int progressive_delay = 0;
    int progressive_int = 0;

    for (int i = 0; i < NBUF; i++) {
        txbuf[i].buf = (uint8_t*)malloc(inbufsize);
        s[i] = txbuf[i].buf;

#if 0
        d = (uint16_t*)txbuf[i].buf;
        if (i < (NBUF / 2)) {
            for (int q = 0; q + 1 < (inbufsize / 2); q += 2) {
                d[q] = (int16_t) int(0x8000 * sin(t) * 0.5);
                t += (2 * 3.1415) / 300;
                //d[q+1] = d[q];
            }
        }
        else {
            for (int q = 0; q + 1 < (inbufsize / 2); q += 2) {
                d[q] = (int16_t) int(0x8000 * sin(t) + 0);
                t += (2 * 3.1415) / 300;
                //d[q+1] = d[q];
            }
        }
#else
        d = (uint16_t*)txbuf[i].buf;
        if (i < NBUF) {
            for (int q = 0; q + 1 < (inbufsize / 2); q += 2) {
                d[q] = (int16_t) int(0x8000 * sin(t) * 0.5);
                //d[q+1] = (int16_t) int(0x8000 * sin(t) * 0.5);
                d[q+1] = (int16_t) int(0x8000 * cos(t) * (0.2 + progressive));
                //d[q + 1] = (int16_t) int((progressive_int++) << 4);
                t += (2 * 3.1415) / 300;
                //d[q+1] = d[q];
                if (++progressive_delay == 800) {
                    progressive_delay = 0;
                    progressive += 0.05;
                }
                if (progressive > 0.8) {
                    progressive = 0;
                }
            }
        }
#endif
        ltxqueue.push(&txbuf[i]);
    }

    lrfnm->tx_work_start(rfnm::TX_LATENCY_POLICY_RELAXED);

    while (1) {
        struct rfnm::tx_buf* ltxbuf;
        rfnm_api_failcode err;

        while (ltxqueue.size()) {
            ltxbuf = ltxqueue.front();
            static int ss = 0;
            ltxbuf->buf = s[ss];
            if (!lrfnm->tx_qbuf(ltxbuf)) {
                if (++ss >= NBUF) ss = 0;
                ltxqueue.pop();
            }
            else {
                break;
            }
        }

        while (!lrfnm->tx_dqbuf(&ltxbuf)) {
            //free(ltxbuf->buf);
            //free(ltxbuf);
            ltxqueue.push(ltxbuf);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    return 0;
}

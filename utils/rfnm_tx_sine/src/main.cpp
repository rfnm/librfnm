#include <stdio.h>
#include <librfnm/device.h>
#include <librfnm/constants.h>
#include <math.h>

#define NBUF 300

int main() {

    auto lrfnm = new rfnm::device(rfnm::TRANSPORT_FIND);
    lrfnm->set_dcs(122880000 / 4); // for now tx frequency is half this, so max is 61 msps

    lrfnm->set_tx_channel_active(0, RFNM_CH_ON, RFNM_CH_STREAM_ON, false);
    lrfnm->set(rfnm::APPLY_CH0_TX);

    auto stream_format = rfnm::STREAM_FORMAT_CS16;

    int inbufsize = RFNM_USB_TX_PACKET_ELEM_CNT * lrfnm->get_transport_status()->tx_stream_format;
    
    std::queue<struct rfnm::tx_buf*> ltxqueue;

    uint8_t* s[NBUF];

    struct rfnm::tx_buf txbuf[NBUF];

    uint16_t* d;
    static double t;
    double progressive = 0.2;
    int progressive_direction = 1;

    for (int i = 0; i < NBUF; i++) {
        txbuf[i].buf = (uint8_t*)malloc(inbufsize);
        s[i] = txbuf[i].buf;

#if 0
        
        d = (uint16_t*)txbuf[i].buf;
        for (int q = 0; q + 1 < (inbufsize / 2); q += 2) {
            // Generate two uniform random numbers in (0,1]
            double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 1.0);
            double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 1.0);
            // Box–Muller transform to obtain two independent Gaussian variables
            double z0 = sqrt(-2.0 * log(u1)) * cos(2 * 3.14159265358979323846 * u2);
            double z1 = sqrt(-2.0 * log(u1)) * sin(2 * 3.14159265358979323846 * u2);
            double amplitude = 0.5; // adjust amplitude as needed

            d[q]   = (int16_t)(0x8000 * z0 * amplitude);
            d[q+1] = (int16_t)(0x8000 * z1 * amplitude);
        }

#else
        d = (uint16_t*)txbuf[i].buf;
        if (i < NBUF) {
            for (int q = 0; q + 1 < (inbufsize / 2); q += 2) {
                d[q] = (int16_t) int(0x8000 * sin(t) * 0.5);
                d[q+1] = (int16_t) int(0x8000 * cos(t) * (0.2 + progressive));
                if(progressive_direction) {
                    progressive += 0.05 / 32000;
                } else {
                    progressive -= 0.05 / 32000;
                }
                
                t += (2 * 3.1415) / 300;
                
                if (progressive > 0.8) {
                    progressive_direction = 0;
                }
                if (progressive < 0.2) {
                    progressive_direction = 1;
                }
            }
        }
#endif
        ltxqueue.push(&txbuf[i]);
    }

    lrfnm->tx_work_start(rfnm::TX_LATENCY_POLICY_RELAXED);

    int dequed = 0;
    static auto tstart = std::chrono::high_resolution_clock::now();

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
                dequed++;
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

        if (ltxqueue.empty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        } else {
            std::this_thread::yield();
        }


        auto tnow = std::chrono::high_resolution_clock::now();
        auto us_int = std::chrono::duration_cast<std::chrono::microseconds>(tnow - tstart);
        if (us_int.count() > 1000000) {
            float sps = (inbufsize * dequed / 4);
            printf("Queued: %d TX: %.2f Msps %.2f Mbps\n", dequed, sps / 1000000, (sps * 24) / 1000000);
            dequed = 0;
            tstart = tnow;
        }
        
    }
    return 0;
}

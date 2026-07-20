#include <stdio.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <queue>
#include <thread>
#include <librfnm/device.h>
#include <librfnm/constants.h>
#include <math.h>

#define NBUF 300

std::atomic<bool> shutdown_req(false);

void signal_handler(int) {
    shutdown_req = true;
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
#ifdef _WIN32
    signal(SIGBREAK, signal_handler);
#endif

    rfnm_api_failcode lret;
    auto lrfnm = new rfnm::device(rfnm::TRANSPORT_FIND);
    lrfnm->set_samp_rate(122880000 / 4); // for now tx frequency is half this, so max is 61 msps

    rfnm::ch_helper tx_ch = lrfnm->tx_ch_helper(0);

    lrfnm->set_tx_channel_status(tx_ch.id, RFNM_CH_RF_ON, RFNM_CH_STREAM_ON);

    lrfnm->set_tx_channel_freq(tx_ch.id, RFNM_MHZ_TO_HZ(3050));

    if ((lret = lrfnm->apply(tx_ch.apply))) {
        printf("Error applying TX channel configuration: %s (code: %d)\n", rfnm::device::failcode_to_string(lret), lret);
        return -1;
    }
    size_t inbufsize;
    uint8_t bytes_per_ele;

    lrfnm->set_stream_format(rfnm::STREAM_FORMAT_CS16, &inbufsize, &bytes_per_ele);
    printf("bufsize: %zu, %d bytes per element\n", inbufsize, bytes_per_ele);
    
    std::queue<struct rfnm::tx_buf*> ltxqueue;

    uint8_t* s[NBUF];

    // zero-initialized on purpose: garbage tx_flags can carry RFNM_TX_FLAG_TIME_VALID
    // with a junk phytimer, which flips the device into timed mode and parks the pump
    struct rfnm::tx_buf txbuf[NBUF] = {};

    uint16_t* d;
    static double t;
    double progressive = 0.2;
    int progressive_direction = 1;

    for (int i = 0; i < NBUF; i++) {
        txbuf[i].buf = (uint8_t*)malloc(inbufsize);
        if (!txbuf[i].buf) {
            printf("Out of memory allocating TX buffers\n");
            return -1;
        }
        s[i] = txbuf[i].buf;

        d = (uint16_t*)txbuf[i].buf;
        {
            for (size_t q = 0; q + 1 < (inbufsize / 2); q += 2) {
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
        ltxqueue.push(&txbuf[i]);
    }

    lrfnm->tx_work_start(rfnm::TX_LATENCY_POLICY_RELAXED);

    int dequed = 0;
    static auto tstart = std::chrono::high_resolution_clock::now();

    while (!shutdown_req) {
        struct rfnm::tx_buf* ltxbuf;

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

    delete lrfnm;
    return 0;
}

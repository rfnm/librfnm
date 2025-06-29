#include <stdio.h>
#include <librfnm/device.h>
#include <librfnm/constants.h>

#define NBUF 1500

std::atomic<bool> shutdown_req(false);

void signal_handler(int signum) {
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
    lrfnm->set_samp_rate(122880000 / 4);

    rfnm::ch_helper rx_ch = lrfnm->rx_ch_helper(1);
    //rfnm::ch_helper tx_ch = lrfnm->tx_ch_helper(0);

    //lrfnm->set_tx_channel_status(tx_ch.ch_id, RFNM_CH_RF_OFF, RFNM_CH_STREAM_OFF);
    lrfnm->set_rx_channel_status(rx_ch.id, RFNM_CH_RF_ON, RFNM_CH_STREAM_ON);

    lrfnm->set_rx_channel_freq(rx_ch.id, RFNM_MHZ_TO_HZ(3050));

    //lrfnm->apply(tx_ch.apply);
    lrfnm->apply(rx_ch.apply);

    if (lret = lrfnm->apply(rx_ch.apply)) {
        printf("Error applying RX channel configuration: %s (code: %d)\n", rfnm::device::failcode_to_string(lret), lret);
        return -1;
    }

    size_t inbufsize;
    uint8_t bytes_per_ele;

    lrfnm->set_stream_format(rfnm::STREAM_FORMAT_CS16, &inbufsize, &bytes_per_ele);
    printf("bufsize: %zd, %d bytes per element\n", inbufsize, bytes_per_ele);

    std::queue<struct rfnm::tx_buf*> ltxqueue;

    uint8_t* s[NBUF];

    struct rfnm::rx_buf rxbuf[NBUF];

    for (int i = 0; i < NBUF; i++) {
        rxbuf[i].buf = (uint8_t*)malloc(inbufsize);
        lrfnm->rx_qbuf(&rxbuf[i], true);
    }

    //lrfnm->tx_work_start(rfnm::TX_LATENCY_POLICY_RELAXED);
    lrfnm->rx_work_start();

    int dequed = 0;
    static auto tstart = std::chrono::high_resolution_clock::now();

    while (!shutdown_req) {
        struct rfnm::rx_buf* lrxbuf;
        struct rfnm::tx_buf* ltxbuf;
        rfnm_api_failcode err;
        int dequed_cycle = 0;

        while (!lrfnm->rx_dqbuf(&lrxbuf, rx_ch.mask, 0)) {
            lrfnm->rx_qbuf(lrxbuf);
            dequed++;
            dequed_cycle++;

            auto tnow = std::chrono::high_resolution_clock::now();
            auto us_int = std::chrono::duration_cast<std::chrono::microseconds>(tnow - tstart);
            if (us_int.count() > 1000000) {
                float sps = (inbufsize * dequed / 4);
                printf("Dequed: %d RX: %.2f Msps %.2f Mbps\n", dequed, sps / 1000000, (sps * 24) / 1000000);
                dequed = 0;
                tstart = tnow;
            }
        }

        if (dequed_cycle) {
            std::this_thread::yield();
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }

    delete lrfnm;
    return 0;
}

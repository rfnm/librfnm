#include <stdio.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <queue>
#include <thread>
#include <librfnm/device.h>
#include <librfnm/constants.h>

#define NBUF 1500

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

    rfnm::ch_helper rx_ch = lrfnm->rx_ch_helper(1);
    rfnm::ch_helper tx_ch = lrfnm->tx_ch_helper(0);

    lrfnm->set_samp_rate(122880000 / 4);

    lrfnm->set_tx_channel_status(tx_ch.id, RFNM_CH_RF_ON, RFNM_CH_STREAM_ON);
    lrfnm->set_rx_channel_status(rx_ch.id, RFNM_CH_RF_ON, RFNM_CH_STREAM_ON);

    lrfnm->set_tx_channel_freq(tx_ch.id, RFNM_MHZ_TO_HZ(3050));
    lrfnm->set_rx_channel_freq(rx_ch.id, RFNM_MHZ_TO_HZ(3050));

    if ((lret = lrfnm->apply(rx_ch.apply | tx_ch.apply))) {
        printf("Error applying TX/RX channel configuration: %s (code: %d)\n", rfnm::device::failcode_to_string(lret), lret);
        return -1;
    }

    size_t inbufsize;
    uint8_t bytes_per_ele;

    lrfnm->set_stream_format(rfnm::STREAM_FORMAT_CS16, &inbufsize, &bytes_per_ele);
    printf("bufsize: %zu, %d bytes per element\n", inbufsize, bytes_per_ele);

    std::queue<struct rfnm::tx_buf*> ltxqueue;

    std::vector<rfnm::rx_buf> rxbufs(NBUF);
    for (int i = 0; i < NBUF; i++) {
        rxbufs[i].buf = (uint8_t*)malloc(inbufsize);
        lrfnm->rx_qbuf(&rxbufs[i], true);
    }

    lrfnm->tx_work_start(rfnm::TX_LATENCY_POLICY_RELAXED);
    lrfnm->rx_work_start();

    int dequed = 0;
    static auto tstart = std::chrono::high_resolution_clock::now();

    std::queue<rfnm::tx_buf*> tx_queue;
    while (!shutdown_req) {
        rfnm::rx_buf* rxb;
        rfnm::tx_buf* txb;
        int dequed_cycle = 0;

        while (!lrfnm->rx_dqbuf(&rxb, rx_ch.mask, 0) /*&& dequed_cycle < 10*/) {
            txb = (rfnm::tx_buf*)rxb;
            tx_queue.push(txb);
            dequed_cycle++;
        }

        dequed_cycle = 0;
        while (!tx_queue.empty() /*&& dequed_cycle < 10*/) {
            txb = tx_queue.front();
            if (!lrfnm->tx_qbuf(txb)) {
                tx_queue.pop();
                dequed++;
                dequed_cycle++;
            }
            else {
                break;
            }
        }

        while (!lrfnm->tx_dqbuf(&txb)) {
            lrfnm->rx_qbuf((rfnm::rx_buf*)txb);
        }

        if (tx_queue.empty() && dequed_cycle == 0) {
            //    std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
        else {
            //    std::this_thread::yield();
        }

        auto tnow = std::chrono::high_resolution_clock::now();
        auto us_int = std::chrono::duration_cast<std::chrono::microseconds>(tnow - tstart);
        if (us_int.count() > 1000000) {
            float sps = (inbufsize * dequed / 4);
            printf("Count: %d Looped: %.2f Msps %.2f Mbps\n", dequed, sps / 1000000, (sps * 24) / 1000000);
            dequed = 0;
            tstart = tnow;
        }
    }



    delete lrfnm;
    return 0;
}

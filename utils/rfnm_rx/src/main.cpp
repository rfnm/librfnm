#include <stdio.h>
#include <librfnm/device.h>
#include <librfnm/constants.h>

#define NBUF 1500

int main() {

    auto lrfnm = new rfnm::device(rfnm::TRANSPORT_FIND);
    lrfnm->set_dcs(122880000 / 4);

    lrfnm->set_tx_channel_active(0, RFNM_CH_OFF, RFNM_CH_STREAM_OFF, false);
    lrfnm->set_rx_channel_active(0, RFNM_CH_ON, RFNM_CH_STREAM_ON, false);
    lrfnm->set(rfnm::rx_channel_apply_flags[0]);
    lrfnm->set(rfnm::tx_channel_apply_flags[0]);

    auto stream_format = rfnm::STREAM_FORMAT_CS16;

    int inbufsize = RFNM_USB_TX_PACKET_ELEM_CNT * lrfnm->get_transport_status()->tx_stream_format;

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

    while (1) {
        struct rfnm::rx_buf* lrxbuf;
        struct rfnm::tx_buf* ltxbuf;
        rfnm_api_failcode err;
        int dequed_cycle = 0;

        while (!lrfnm->rx_dqbuf(&lrxbuf, 0, 0)) {
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

    return 0;
}

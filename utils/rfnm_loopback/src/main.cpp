#include <stdio.h>
#include <librfnm/device.h>
#include <librfnm/constants.h>

#define NBUF 1500

int main() {


    /*
    nasty bug workaround:
    we use tx/rx streaming status to decide the dcs clock, which is set on the fly by the si5510
    somehow if we set two different si5510 dcs clock freqs too fast, nothing gets applied
    when calling ->set(), guess what, we set them in quick succession

    -> set rx first so that it's the same clock as it will be in the near future when tx's set gets called
    */

    auto lrfnm = new rfnm::device(rfnm::TRANSPORT_FIND);

    // set dcs clock to 61 msps

    //lrfnm->set_dcs(122880000 / 16);
    //lrfnm->set_rx_channel_active(0, RFNM_CH_ON, RFNM_CH_STREAM_ON, false);
    //lrfnm->set(rfnm::rx_channel_apply_flags[0]);

    // clock already set to 61, enable tx
    lrfnm->set_dcs(122880000 / 4);

    lrfnm->set_tx_channel_active(0, RFNM_CH_ON, RFNM_CH_STREAM_ON, false);
    lrfnm->set_rx_channel_active(1, RFNM_CH_ON, RFNM_CH_STREAM_ON, false);

    lrfnm->set_tx_channel_freq(0, RFNM_MHZ_TO_HZ(3050), false);
    lrfnm->set_rx_channel_freq(1, RFNM_MHZ_TO_HZ(3050), false);

    lrfnm->set(rfnm::APPLY_CH1_RX | rfnm::APPLY_CH0_TX);

    auto stream_format = rfnm::STREAM_FORMAT_CS16;

    int inbufsize = RFNM_USB_TX_PACKET_ELEM_CNT * lrfnm->get_transport_status()->tx_stream_format;

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
    while (true) {
        rfnm::rx_buf* rxb;
        rfnm::tx_buf* txb;
        int dequed_cycle = 0;

        while (!lrfnm->rx_dqbuf(&rxb, 0, 0) /*&& dequed_cycle < 10*/) {
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
            int qout = tx_queue.size();
            printf("Count: %d Looped: %.2f Msps %.2f Mbps\n", dequed, sps / 1000000, (sps * 24) / 1000000);
            dequed = 0;
            tstart = tnow;
        }
    }




    return 0;
}

#include <stdio.h>
#include <librfnm/device.h>

void dumpHwInfo(const rfnm_dev_hwinfo_bit& info, bool daughterboard, int offset = 0) {
    // Create offset string
    std::string ofs;
    for (int i = 0; i < offset; i++) { ofs += "    "; }

    // Dump info
    const uint8_t* mac = info.mac_addr;
    printf("%s* Board ID:         %d\n", ofs.c_str(), info.board_id);
    printf("%s* Board Revision:   %d\n", ofs.c_str(), info.board_revision_id);
    if (mac[5] || mac[4] || mac[3] || mac[2] || mac[1] || mac[0]) {
        printf("%s* MAC Address:      %02X:%02X:%02X:%02X:%02X:%02X\n", ofs.c_str(), mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    }
    if (daughterboard) {
        printf("%s* RX Channel Count: %d\n", ofs.c_str(), info.rx_ch_cnt);
        printf("%s* TX Channel Count: %d\n", ofs.c_str(), info.tx_ch_cnt);
    }
    printf("%s* Temperature:      %d Celcius\n", ofs.c_str(), info.temperature);
}

int main() {
    // List all available devices
    std::vector<rfnm_dev_hwinfo> list;
    try {
        list = rfnm::device::find(rfnm::transport::TRANSPORT_USB);
    }
    catch (const std::exception& e) {
        fprintf(stderr, "Failed to list devices: %s\n", e.what());
        return -1;
    }

    // If no device is available, give up
    if (list.empty()) {
        printf("No RFNM device found.\n\n");
        return 0;
    }

    // Display all of them
    for (const auto& info : list) {
        try {
            // Show device name and serial number
            printf("RFNM %s [Serial=%s]\n", info.motherboard.user_readable_name, info.motherboard.serial_number);
            printf("----------------------------------\n");

            // Show motherboard information
            printf("* Protocol Version: %d\n", info.protocol_version);
            dumpHwInfo(info.motherboard, false);

            // Show info for all daughterboards
            for (int i = 0; i < 2; i++) {
                // Get daughterboard info
                const rfnm_dev_hwinfo_bit& db = info.daughterboard[i];

                // If not installed, print and such and return
                if (!db.board_id) {
                    printf("* Daughterboard[%d]: Not Present\n", i);
                    continue;
                }

                // Show the name and title
                printf("* Daughterboard[%d]: %s [Serial=%s]\n", i, db.user_readable_name, db.serial_number);
                dumpHwInfo(db, true, 1);
            }
        }
        catch (const std::exception& e) {
            fprintf(stderr, "Failed to query device info: %s\n", e.what());
        }

        // Space
        printf("\n");
    }

    return 0;
}

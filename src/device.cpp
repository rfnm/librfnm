#include <algorithm>
#include <librfnm/device.h>
#include <librfnm/rx_stream.h>
#include <spdlog/spdlog.h>
#include <libusb-1.0/libusb.h>

using namespace rfnm;

struct rfnm::_usb_handle {
    libusb_device_handle* primary{};
    libusb_device_handle* boost{};
};

MSDLL device::device(enum transport transport, std::string address, enum debug_level dbg) {
    rx_s.qbuf_cnt = 0;

    //if (transport != TRANSPORT_USB) {
    //    spdlog::error("Transport not supported");
    //    throw std::runtime_error("Transport not supported");
    //}

    

    s = new struct status();
    usb_handle = new _usb_handle;

    int cnt = 0;
    int dev_cnt = 0;
    int r;
    std::vector<struct rfnm_dev_hwinfo> found;
    libusb_device** devs = NULL;

    // set default/native stream format
    s->transport_status.rx_stream_format = STREAM_FORMAT_CS16;
    s->transport_status.tx_stream_format = STREAM_FORMAT_CS16;
    if (transport == TRANSPORT_USB || transport == TRANSPORT_FIND) {
#if LIBUSB_API_VERSION >= 0x0100010A
        r = libusb_init_context(nullptr, nullptr, 0);
#else
        r = libusb_init(nullptr);
#endif
        if (r < 0) {
            spdlog::error("RFNMDevice::activateStream() -> failed to initialize libusb");
            goto error_usb;
        }

        dev_cnt = libusb_get_device_list(NULL, &devs);
        if (dev_cnt < 0) {
            spdlog::error("failed to get list of usb devices");
            goto error_usb;
        }

        for (int d = 0; d < dev_cnt; d++) {
            struct libusb_device_descriptor desc;
            int r = libusb_get_device_descriptor(devs[d], &desc);
            if (r < 0) {
                spdlog::error("failed to get usb dev descr");
                goto next;
            }

            if (desc.idVendor != RFNM_USB_VID || desc.idProduct != RFNM_USB_PID) {
                goto next;
            }

            r = libusb_open(devs[d], &usb_handle->primary);
            if (r) {
                spdlog::error("Found RFNM device, but couldn't open it {}", r);
                goto next;
            }

            if (address.length()) {
                uint8_t sn[9];
                if (libusb_get_string_descriptor_ascii(usb_handle->primary, desc.iSerialNumber, sn, 9) >= 0) {
                    sn[8] = '\0';
                    if (strcmp((const char*)sn, address.c_str())) {
                        spdlog::info("This serial {} doesn't match the requested {}", (const char*)sn, address);
                        goto next;
                    }
                }
                else {
                    spdlog::error("Couldn't read serial descr");
                    goto next;
                }
            }

            if (libusb_get_device_speed(libusb_get_device(usb_handle->primary)) < LIBUSB_SPEED_SUPER) {
                spdlog::error("You are connected using USB 2.0 (480 Mbps), however USB 3.0 (5000 Mbps) is required. "
                    "Please make sure that the cable and port you are using can work with USB 3.0 SuperSpeed");
                goto next;
            }

            r = libusb_claim_interface(usb_handle->primary, 0);
            if (r < 0) {
                spdlog::error("Found RFNM device, but couldn't claim the interface, {}, {}", r, libusb_strerror((libusb_error)r));
                goto next;
            }

            s->transport_status.theoretical_mbps = 3500;

            usb_handle->boost = libusb_open_device_with_vid_pid(nullptr, RFNM_USB_VID, RFNM_USB_PID_BOOST);
            if (usb_handle->boost) {
                if (libusb_get_device_speed(libusb_get_device(usb_handle->boost)) >= LIBUSB_SPEED_SUPER) {
                    r = libusb_claim_interface(usb_handle->boost, 0);
                    if (r >= 0) {
                        s->transport_status.theoretical_mbps += 3500;
                        s->transport_status.usb_boost_connected = 1;
                    }
                }
            }

            spdlog::info("Max theoretical transport speed is {} Mbps", s->transport_status.theoretical_mbps);

            r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR),
                RFNM_B_REQUEST, RFNM_GET_SM_RESET, 0, NULL, 0, 500);
            if (r < 0) {
                spdlog::error("Couldn't reset state machine");
                goto next;
            }

            s->transport_status.transport = TRANSPORT_USB;
            THREAD_COUNT = MAX_THREAD_COUNT;

            if (get(REQ_ALL)) {
                goto next;
            }

            libusb_free_device_list(devs, 1);

            for (int8_t i = 0; i < THREAD_COUNT; i++) {
                thread_data[i].ep_id = i + 1;
                thread_data[i].rx_active = 0;
                thread_data[i].tx_active = 0;
                thread_data[i].shutdown_req = 0;
            }

            for (int i = 0; i < THREAD_COUNT; i++) {
                thread_c[i] = std::thread(&device::threadfn, this, i);
            }

            // Success
            return;

        next:
            if (usb_handle->primary) {
                libusb_release_interface(usb_handle->primary, 0);
                libusb_close(usb_handle->primary);
                usb_handle->primary = NULL;
            }
            if (usb_handle->boost) {
                libusb_release_interface(usb_handle->boost, 0);
                libusb_close(usb_handle->boost);
                usb_handle->boost = NULL;
            }
        }

    error_usb:
        libusb_free_device_list(devs, 1);
        libusb_exit(NULL);
        delete usb_handle;

        if (transport != TRANSPORT_FIND) {
            delete s;
            throw std::runtime_error("RFNM culdn't find any USB devices");
        }
        
    }
    if (transport == TRANSPORT_LOCAL || transport == TRANSPORT_FIND) {
#ifdef RFNM_COMPILE_LOCAL_TRANSPORT
        int fd = open("/dev/rfnm_ctrl_ep", O_RDWR);
        if (fd < 0) {
            goto exit_local;
        }
        uint8_t ep_ctrl_buf[RFNM_SYSCTL_TRANSFER_SIZE];

        if (ioctl(fd, RFNM_IOCTL_BASE + (0xff & RFNM_GET_DEV_HWINFO), &ep_ctrl_buf) < 0) {
            close(fd);
            goto exit_close_local;
        }

        rfnm_ctrl_ep_ioctl = fd;

        struct rfnm_dev_hwinfo r_hwinfo;

        memcpy(&r_hwinfo, &ep_ctrl_buf[0], sizeof(struct rfnm_dev_hwinfo));

        if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
            uint32_t pv = r_hwinfo.protocol_version;
            spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED detected protocol is v{} while your librfnm is for v{}", pv, RFNM_PROTOCOL_VERSION);
            goto exit_close_local;
        }

        s->transport_status.theoretical_mbps = 3500;

        //spdlog::info("Max theoretical transport speed is {} Mbps", s->transport_status.theoretical_mbps);

        if (ioctl(fd, RFNM_IOCTL_BASE + (0xff & RFNM_GET_SM_RESET), &ep_ctrl_buf) < 0) {
            //close(fd);
            spdlog::error("Couldn't reset state machine");
            goto exit_close_local;
        }

        s->transport_status.transport = TRANSPORT_LOCAL;
        THREAD_COUNT = 3;

        if (get(REQ_ALL)) {
            goto exit_close_local;
        }

        for (int8_t i = 0; i < THREAD_COUNT; i++) {
            thread_data[i].ep_id = i + 1;
            thread_data[i].rx_active = 0;
            thread_data[i].tx_active = 0;
            thread_data[i].shutdown_req = 0;
        }

        for (int i = 0; i < THREAD_COUNT; i++) {
            thread_c[i] = std::thread(&device::threadfn, this, i);
        }
        
        // Success
        return;

    exit_close_local:
        close(fd); 
#endif
    }
exit_local:
    if (transport == TRANSPORT_ETH || transport == TRANSPORT_FIND) {
        try {

            asio::io_context io_context;

            std::string remote_addr;
            //spdlog::info("Detected broadcast address: {}", remote_addr);

            if (address.length()) {
                remote_addr = address;
            }
            else {
                remote_addr = get_broadcast_address();
                if (remote_addr.empty()) {
                    //spdlog::error("Failed to detect broadcast address.");
                    goto exit_eth;
                }
            }

            asio::ip::udp::endpoint remote_endpoint(asio::ip::make_address(remote_addr), RFNM_UDP_CTRL_PORT);

            asio::ip::udp::socket socket(io_context);
            socket.open(asio::ip::udp::v4());

#ifdef _WIN32
            DWORD timeout = 50; // 10 ms for Windows.
            if (setsockopt(socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                reinterpret_cast<const char*>(&timeout), sizeof(timeout)) < 0)
            {
                spdlog::error("Failed to set receive timeout on Windows");
            }
#else
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 50000; // 10,000 microseconds = 10 ms.
            if (setsockopt(socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                &tv, sizeof(tv)) < 0)
            {
                spdlog::error("Failed to set receive timeout on Linux/macOS");
            }
#endif

            if (!address.length()) {
                asio::socket_base::broadcast broadcast_option(true);
                socket.set_option(broadcast_option);
            }

            uint8_t message[1];
            message[0] = ((int)RFNM_GET_DEV_HWINFO) & 0xff;

            socket.send_to(asio::buffer(message), remote_endpoint);
            //spdlog::info("Sent RFNM_GET_DEV_HWINFO request.");

            //auto local_ep = socket.local_endpoint();
            //spdlog::info("Local ephemeral port assigned: {}", local_ep.port());

            char reply[1152];
            asio::ip::udp::endpoint sender_endpoint;
            size_t reply_length = socket.receive_from(asio::buffer(reply), sender_endpoint);

            rfnm_dev_hwinfo r_hwinfo;
            if (reply_length == sizeof(rfnm_dev_hwinfo) + 1) {
                if (reply[0] != (RFNM_GET_DEV_HWINFO & 0xff)) {
                    spdlog::error("UDP control transfer conflict got {} should have been RFNM_GET_DEV_HWINFO", (uint8_t) reply[0]);
                    goto exit_eth;
                }
                memcpy(&r_hwinfo, &reply[1], sizeof(rfnm_dev_hwinfo));
                if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
                    uint32_t mv = r_hwinfo.protocol_version;
                    spdlog::error("Protocol version mismatch: received {}, expected {}",  mv, RFNM_PROTOCOL_VERSION);
                    goto exit_eth;
                }

                message[0] = ((int) RFNM_GET_SM_RESET) & 0xff;
                socket.send_to(asio::buffer(message), remote_endpoint);

                rfnm_ctrl_ep_udp = asio::ip::udp::endpoint(asio::ip::make_address(sender_endpoint.address().to_string()), RFNM_UDP_CTRL_PORT);
                rfnm_ctrl_ioctx_tcp = std::make_unique<asio::io_context>();
                rfnm_ctrl_socket_udp = std::make_unique<asio::ip::udp::socket>(*rfnm_ctrl_ioctx_tcp);
                rfnm_ctrl_socket_udp->open(asio::ip::udp::v4());

                rfnm_ctrl_socket_udp->set_option(asio::socket_base::receive_buffer_size(1024 * 1024 * 4));
                rfnm_ctrl_socket_udp->set_option(asio::socket_base::send_buffer_size(1024 * 1024 * 4));

#ifdef _WIN32
                DWORD timeout = 50; // 10 ms for Windows.
                if (setsockopt(rfnm_ctrl_socket_udp->native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                    reinterpret_cast<const char*>(&timeout), sizeof(timeout)) < 0)
                {
                    spdlog::error("Failed to set receive timeout on Windows");
                }
#else
                struct timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = 50000; // 10,000 microseconds = 10 ms.
                if (setsockopt(socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                    &tv, sizeof(tv)) < 0)
                {
                    spdlog::error("Failed to set receive timeout on Linux/macOS");
                }
#endif
#if 0
                rfnm_data_ep_udp_tx = asio::ip::udp::endpoint(asio::ip::make_address(sender_endpoint.address().to_string()), RFNM_UDP_DATA_PORT);
                //rfnm_data_ep_udp_rx = asio::ip::udp::endpoint(asio::ip::udp::v4(), RFNM_UDP_DATA_PORT);

                rfnm_data_ioctx_tcp = std::make_unique<asio::io_context>();
                rfnm_data_socket_udp = std::make_unique<asio::ip::udp::socket>(*rfnm_data_ioctx_tcp);
                rfnm_data_socket_udp->open(asio::ip::udp::v4());

                //rfnm_data_socket_udp->set_option(asio::socket_base::receive_buffer_size(1024 * 1024 * 4));
                //rfnm_data_socket_udp->set_option(asio::socket_base::send_buffer_size(1024 * 1024 * 4));


                asio::error_code ec;

                rfnm_data_socket_udp->set_option(asio::socket_base::receive_buffer_size(1024 * 1024 * 64), ec);
                if (ec) {
                    spdlog::error("Error setting SO_RCVBUF: {} ", ec.message());
                }

                rfnm_data_socket_udp->set_option(asio::socket_base::send_buffer_size(1024 * 1024 * 64), ec);
                if (ec) {
                    spdlog::error("Error setting SO_RCVBUF: {}", ec.message());
                }




                asio::ip::udp::endpoint remote_endpoint_data(asio::ip::make_address(remote_addr), RFNM_UDP_DATA_PORT);
                uint8_t empty_packet[1];
                empty_packet[0] = 0xfe;
                // give the remote host our current ephimeral port
                rfnm_data_socket_udp->send_to(asio::buffer(empty_packet, 1), remote_endpoint_data);
                rfnm_data_socket_udp->send_to(asio::buffer(empty_packet, 1), remote_endpoint_data);
                rfnm_data_socket_udp->send_to(asio::buffer(empty_packet, 1), remote_endpoint_data);

                //rfnm_data_socket_udp->bind(rfnm_data_ep_udp_rx);

                

#ifdef _WIN32
                timeout = 50; 
                if (setsockopt(rfnm_data_socket_udp->native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                    reinterpret_cast<const char*>(&timeout), sizeof(timeout)) < 0)
                {
                    spdlog::error("Failed to set receive timeout on Windows");
                }
#else
                //struct timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = 50000; // 10,000 microseconds = 10 ms.
                if (setsockopt(rfnm_data_socket_udp.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                    &tv, sizeof(tv)) < 0)
                {
                    spdlog::error("Failed to set receive timeout on Linux/macOS");
                }
#endif
#else
                rfnm_eth_transport_ip_addr = sender_endpoint.address().to_string();
#endif
                s->transport_status.theoretical_mbps = 123;

                s->transport_status.transport = TRANSPORT_ETH;
                THREAD_COUNT = 8;


                if (get(REQ_ALL)) {
                    goto exit_eth;
                }

                for (int8_t i = 0; i < THREAD_COUNT; i++) {
                    thread_data[i].ep_id = i + 1;
                    thread_data[i].rx_active = 0;
                    thread_data[i].tx_active = 0;
                    thread_data[i].shutdown_req = 0;
                }

                for (int i = 0; i < THREAD_COUNT; i++) {
                    thread_c[i] = std::thread(&device::threadfn, this, i);
                }

                return;
            }
            else {
                spdlog::error("Reply too short ({} bytes)", reply_length);
            }
        }
        catch (std::exception& e) {
            spdlog::error("UDP problem: {}", e.what());
        }
    }
exit_eth:
    throw std::runtime_error("Couldn't find any RFNM device");
}

MSDLL device::~device() {
    for (int8_t i = 0; i < THREAD_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
        thread_data[i].rx_active = 0;
        thread_data[i].tx_active = 0;
        thread_data[i].shutdown_req = 1;
        thread_data[i].cv.notify_one();
    }

    for (auto& i : thread_c) {
        i.join();
    }

    if (rx_buffers_allocated) {
        rx_flush(0);

        // no need to take in_mutex as threads are finished
        while (rx_s.in.size()) {
            rx_buf *rxbuf = rx_s.in.front();
            rx_s.in.pop();
            delete[] rxbuf->buf;
            delete rxbuf;
        }
    }

    delete s;

    if (usb_handle->primary) {
        libusb_release_interface(usb_handle->primary, 0);
        libusb_close(usb_handle->primary);
    }
    if (usb_handle->boost) {
        libusb_release_interface(usb_handle->boost, 0);
        libusb_close(usb_handle->boost);
    }

    if (rfnm_ctrl_ioctx_tcp)
        rfnm_ctrl_ioctx_tcp->stop();

    if (rfnm_ctrl_socket_udp && rfnm_ctrl_socket_udp->is_open()) {
        std::error_code ec;
        rfnm_ctrl_socket_udp->close(ec);
        if (ec) {
            spdlog::warn("Error closing UDP socket: {}", ec.message());
        }
    }

/*    if (rfnm_data_ioctx_tcp)
        rfnm_data_ioctx_tcp->stop();

    if (rfnm_data_socket_udp && rfnm_data_socket_udp->is_open()) {
        std::error_code ec;
        rfnm_data_socket_udp->close(ec);
        if (ec) {
            spdlog::warn("Error closing UDP socket: {}", ec.message());
        }
    }*/


    delete usb_handle;
    libusb_exit(NULL);
}

MSDLL bool device::unpack_12_to_cs16(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
    uint64_t buf{};
    uint64_t r0{};
    uint64_t* dest_64{};
    uint64_t* src_64{};

    if (sample_cnt % 2) {
        spdlog::error("RFNM::Conversion::unpack12to16() -> sample_cnt {} is not divisible by 2", sample_cnt);
        return false;
    }

    // process two samples at a time
    sample_cnt = sample_cnt / 2;
    for (size_t c = 0; c < sample_cnt; c++) {
        src_64 = (uint64_t*)((uint8_t*)src + (c * 6));
        buf = *(src_64); //unaligned read?
        r0 = 0;
        r0 |= (buf & (0xfffll << 0)) << 4;
        r0 |= (buf & (0xfffll << 12)) << 8;
        r0 |= (buf & (0xfffll << 24)) << 12;
        r0 |= (buf & (0xfffll << 36)) << 16;

        dest_64 = (uint64_t*)(dest + (c * 8));
        *dest_64 = r0;
    }
    return true;
}

MSDLL bool device::unpack_12_to_cf32(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
    uint64_t buf{};
    uint64_t r0{};
    uint64_t* dest_64{};
    uint64_t* src_64{};

    if (sample_cnt % 2) {
        spdlog::error("RFNM::Conversion::unpack12to16() -> sample_cnt {} is not divisible by 2", sample_cnt);
        return false;
    }

    // process two samples at a time
    sample_cnt = sample_cnt / 2;
    for (size_t c = 0; c < sample_cnt; c++) {
        src_64 = (uint64_t*)((uint8_t*)src + (c * 6));
        buf = *(src_64); //unaligned read?

        float* i1, * i2, * q1, * q2;

        i1 = (float*)((uint8_t*)dest + (c * 16) + 0);
        q1 = (float*)((uint8_t*)dest + (c * 16) + 4);
        i2 = (float*)((uint8_t*)dest + (c * 16) + 8);
        q2 = (float*)((uint8_t*)dest + (c * 16) + 12);


        *i1 = ((int16_t)((buf & (0xfffll << 0)) << 4)) / 32767.0f;
        *q1 = ((int16_t)((buf & (0xfffll << 12)) >> 8)) / 32767.0f;
        *i2 = ((int16_t)((buf & (0xfffll << 24)) >> 20)) / 32767.0f;
        *q2 = ((int16_t)((buf & (0xfffll << 36)) >> 32)) / 32767.0f;
    }
    return true;
}

MSDLL bool device::unpack_12_to_cs8(uint8_t* dest, uint8_t* src, size_t sample_cnt) {
    uint64_t buf{};
    uint32_t r0{};
    uint32_t* dest_32{};
    uint64_t* src_64{};

    if (sample_cnt % 2) {
        spdlog::error("RFNM::Conversion::unpack12to16() -> sample_cnt {} is not divisible by 2", sample_cnt);
        return false;
    }

    // process two samples at a time
    sample_cnt = sample_cnt / 2;
    for (size_t c = 0; c < sample_cnt; c++) {
        src_64 = (uint64_t*)((uint8_t*)src + (c * 6));
        buf = *(src_64);
        r0 = 0;
        r0 |= (buf & (0xffll << 4)) >> 4;
        r0 |= (buf & (0xffll << 16)) >> 8;
        r0 |= (buf & (0xffll << 28)) >> 12;
        r0 |= (buf & (0xffll << 40)) >> 16;

        dest_32 = (uint32_t*)((uint8_t*)dest + (c * 4));
        *dest_32 = r0;
    }
    return true;
}

MSDLL void device::pack_cs16_to_12(uint8_t* dest, uint8_t* src8, int sample_cnt) {
//#ifndef __builtin_unreachable
    uint64_t buf;
    uint64_t r0;
    int32_t c;
    uint64_t* dest_64;
    uint64_t* src;

    src = (uint64_t*)src8;
    sample_cnt = sample_cnt / 2;

    for (c = 0; c < sample_cnt; c++) {
        buf = *(src + c);
        r0 = 0;
        r0 |= (buf & (0xfffll << 4)) >> 4;
        r0 |= (buf & (0xfffll << 20)) >> 8;
        r0 |= (buf & (0xfffll << 36)) >> 12;
        r0 |= (buf & (0xfffll << 52)) >> 16;

        dest_64 = (uint64_t*)(dest + (c * 6));
        *dest_64 = r0;
    }
/*#else
    if (sample_cnt & 255)
        __builtin_unreachable();

    while (sample_cnt >= 4) {
        int16_t i = (*src8++) >> 4;
        int16_t q = (*src8++) >> 4;
        *dest++ = i & 0xFF;
        *dest++ = ((i >> 8) & 0xF) | ((q << 4) & 0xF0);
        *dest++ = (q >> 4) & 0xFF;
        sample_cnt -= 4;
    }
#endif*/
}

void device::threadfn(size_t thread_index) {
    struct rfnm_rx_usb_buf* lrxbuf = new rfnm_rx_usb_buf();
    struct rfnm_tx_usb_buf* ltxbuf = new rfnm_tx_usb_buf();
    int transferred;
    auto& tpm = thread_data[thread_index];
    int r;

#ifdef RFNM_COMPILE_LOCAL_TRANSPORT
    int data_ep_fp;
    if (s->transport_status.transport == TRANSPORT_LOCAL) {
        data_ep_fp = open("/dev/rfnm_data_ep", O_RDWR);
        if (data_ep_fp < 0) {
            return;
        }
    }
#endif

    asio::ip::udp::endpoint rfnm_data_ep_udp_tx;
    asio::ip::udp::endpoint rfnm_data_ep_udp_rx;
    asio::io_context rfnm_data_ioctx_udp;
    asio::ip::udp::socket rfnm_data_socket_udp(rfnm_data_ioctx_udp);

    if (s->transport_status.transport == TRANSPORT_ETH) {
        rfnm_data_ep_udp_tx = asio::ip::udp::endpoint(asio::ip::make_address(rfnm_eth_transport_ip_addr), RFNM_UDP_DATA_PORT);
        rfnm_data_socket_udp.open(asio::ip::udp::v4());

        rfnm_data_socket_udp.set_option(asio::socket_base::receive_buffer_size(1024 * 1024 * 4));
        rfnm_data_socket_udp.set_option(asio::socket_base::send_buffer_size(1024 * 1024 * 4));

        uint8_t empty_packet[1];
        empty_packet[0] = 0xfe;
        // give the remote host our current ephimeral port
        rfnm_data_socket_udp.send_to(asio::buffer(empty_packet, 1), rfnm_data_ep_udp_tx);

        //rfnm_data_socket_udp->bind(rfnm_data_ep_udp_rx);



#ifdef _WIN32
        DWORD timeout = 50;
        if (setsockopt(rfnm_data_socket_udp.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
            reinterpret_cast<const char*>(&timeout), sizeof(timeout)) < 0)
        {
            spdlog::error("Failed to set receive timeout on Windows");
        }
#else
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50000; // 10,000 microseconds = 10 ms.
        if (setsockopt(rfnm_data_socket_udp.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
            &tv, sizeof(tv)) < 0)
        {
            spdlog::error("Failed to set receive timeout on Linux/macOS");
        }
#endif
    }

    while (!tpm.shutdown_req) {
        if (!tpm.rx_active && !tpm.tx_active) {
            {
                std::unique_lock lk(tpm.cv_mutex);
                // spurious wakeups are acceptable
                tpm.cv.wait(lk,
                    [this, thread_index] { return thread_data[thread_index].rx_active ||
                                                  thread_data[thread_index].tx_active ||
                                                  thread_data[thread_index].shutdown_req; });
            }
        }

        if (s->transport_status.transport == TRANSPORT_ETH && (tpm.ep_id % 2) == 0) {
        //   goto skip_rx;
            //receiver is blocking, so skip it in half the threads... 
        }

        

        if (tpm.rx_active) {
            struct rx_buf* buf;

            {
                std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);

                if (rx_s.in.empty()) {
                    goto skip_rx;
                }

                buf = rx_s.in.front();
                rx_s.in.pop();
            }

            if (s->transport_status.transport == TRANSPORT_USB) {

                libusb_device_handle* lusb_handle = usb_handle->primary;
                if (1 && s->transport_status.usb_boost_connected) {
                    std::lock_guard<std::mutex> lockGuard(s_transport_pp_mutex);
                    //if (s->transport_status.boost_pp_rx) {
                    if ((tpm.ep_id % 2) == 0) {
                        lusb_handle = usb_handle->boost;
                    }
                    s->transport_status.boost_pp_rx = !s->transport_status.boost_pp_rx;
                }

                r = libusb_bulk_transfer(lusb_handle, (((tpm.ep_id % 4) + 1) | LIBUSB_ENDPOINT_IN),
                    (uint8_t*)lrxbuf, RFNM_USB_RX_PACKET_SIZE, &transferred, 1000);
                if (r) {
                    spdlog::error("RX bulk tx fail {} {}", tpm.ep_id, r);
                    std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                    rx_s.in.push(buf);
                    goto skip_rx;
                }

                if (transferred != RFNM_USB_RX_PACKET_SIZE) {
                    spdlog::error("thread loop RX usb wrong size, {}, {}", transferred, tpm.ep_id);
                    std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                    rx_s.in.push(buf);
                    goto skip_rx;
                }
            }

            if (s->transport_status.transport == TRANSPORT_LOCAL) {
#ifdef RFNM_COMPILE_LOCAL_TRANSPORT
                if (ioctl(data_ep_fp, RFNM_IOCTL_BASE_DATA | 1, (uint8_t*)lrxbuf) < 0) {
                    //spdlog::error("thread loop RX no data");
                    {
                        std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                        rx_s.in.push(buf);
                    }
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                    goto skip_rx;
                }
#endif
            }

            if (s->transport_status.transport == TRANSPORT_ETH) {
                try {
                    if (rfnm_data_socket_udp.receive(asio::buffer((uint8_t*)lrxbuf, RFNM_USB_RX_PACKET_SIZE)) != RFNM_USB_RX_PACKET_SIZE) {
                        
                        std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                        rx_s.in.push(buf); 

                        goto skip_rx;
                    }

                    {
                        std::lock_guard<std::mutex> lockGuard(rx_s.benchmark_mutex);
                        rx_s.usb_cc_benchmark[tpm.ep_id % 4]++;
                    }

                }
                catch (std::exception& e) {
                    std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                    rx_s.in.push(buf);

                    //spdlog::error("UDP problem: {}", e.what());
                }
                
            }


            if (lrxbuf->magic != 0x7ab8bd6f || lrxbuf->adc_id > 3) {
                //spdlog::error("Wrong magic");
                std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
                rx_s.in.push(buf);
                goto skip_rx;
            }

            if (s->transport_status.rx_stream_format == STREAM_FORMAT_CS8) {
                unpack_12_to_cs8(buf->buf, (uint8_t*)lrxbuf->buf, RFNM_USB_RX_PACKET_ELEM_CNT);
            }
            else if (s->transport_status.rx_stream_format == STREAM_FORMAT_CS16) {
                unpack_12_to_cs16(buf->buf, (uint8_t*)lrxbuf->buf, RFNM_USB_RX_PACKET_ELEM_CNT);
            }
            else if (s->transport_status.rx_stream_format == STREAM_FORMAT_CF32) {
                unpack_12_to_cf32(buf->buf, (uint8_t*)lrxbuf->buf, RFNM_USB_RX_PACKET_ELEM_CNT);
            }

            buf->adc_cc = lrxbuf->adc_cc;
            buf->adc_id = lrxbuf->adc_id;
            buf->usb_cc = lrxbuf->usb_cc;
            buf->phytimer = lrxbuf->phytimer;

            {
                std::lock_guard<std::mutex> lockGuard(rx_s.out_mutex);
                rx_s.out[lrxbuf->adc_id].push(buf);

                //if (rx_s.out[lrxbuf->adc_id].size() > 50) {
                rx_s.cv.notify_one();
                //}
            }
        }

    skip_rx:

        if (tpm.tx_active) {
            struct tx_buf* buf;

            if (tx_s.in.empty()) {
                goto read_dev_status;
            }


            {
                std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);

                if (tx_s.in.empty()) {
                    goto read_dev_status;
                }
                buf = tx_s.in.front();
                tx_s.in.pop();
            }

            pack_cs16_to_12((uint8_t*)ltxbuf->buf, buf->buf, RFNM_USB_TX_PACKET_ELEM_CNT);
            ltxbuf->dac_cc = buf->dac_cc;
            ltxbuf->dac_id = buf->dac_id;
            ltxbuf->usb_cc = buf->usb_cc;
            ltxbuf->phytimer = buf->phytimer;
            ltxbuf->magic = 0x758f4d4a;

            if (s->transport_status.transport == TRANSPORT_USB) {
                libusb_device_handle* lusb_handle = usb_handle->primary;
                if (0 && s->transport_status.usb_boost_connected) {
                    std::lock_guard<std::mutex> lockGuard(s_transport_pp_mutex);
                    if (s->transport_status.boost_pp_tx) {
                        lusb_handle = usb_handle->boost;
                    }
                    s->transport_status.boost_pp_tx = !s->transport_status.boost_pp_tx;
                }

                r = libusb_bulk_transfer(lusb_handle, (((tpm.ep_id % 4) + 1) | LIBUSB_ENDPOINT_OUT),
                    (uint8_t*)ltxbuf, RFNM_USB_TX_PACKET_SIZE, &transferred, 1000);
                if (r) {
                    spdlog::error("TX bulk tx fail {} {}", tpm.ep_id, r);
                    std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                    tx_s.in.push(buf);
                    goto read_dev_status;
                }

                if (transferred != RFNM_USB_TX_PACKET_SIZE) {
                    spdlog::error("thread loop TX usb wrong size, {}, {}", transferred, tpm.ep_id);
                    std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                    tx_s.in.push(buf);
                    goto read_dev_status;
                }
            }

            if (s->transport_status.transport == TRANSPORT_LOCAL) {
#ifdef RFNM_COMPILE_LOCAL_TRANSPORT
                if (ioctl(data_ep_fp, RFNM_IOCTL_BASE_DATA | 0, (uint8_t*)ltxbuf) < 0) {
                    spdlog::error("thread loop TX busy");
                    {
                        std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                        tx_s.in.push(buf);
                    }
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                    goto read_dev_status;
                }
#endif
            }

            if (s->transport_status.transport == TRANSPORT_ETH) {
                if (rfnm_data_socket_udp.send_to(asio::buffer((uint8_t*)ltxbuf, RFNM_USB_TX_PACKET_SIZE), rfnm_data_ep_udp_tx) != RFNM_USB_TX_PACKET_SIZE) {
                    spdlog::error("TX UDP queue error");
                    std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
                    tx_s.in.push(buf);
                    goto read_dev_status;
                }
            }

            

            {
                std::lock_guard<std::mutex> lockGuard(tx_s.out_mutex);
                tx_s.out.push(buf);
                tx_s.cv.notify_one();
            }
        }

read_dev_status:

        {
#if 0
            using std::chrono::high_resolution_clock;
            using std::chrono::duration_cast;
            using std::chrono::duration;
            using std::chrono::milliseconds;

            auto tlast = s->last_dev_time;
            auto tnow = high_resolution_clock::now();
            auto ms_int = duration_cast<milliseconds>(tnow - tlast);

            if (1 && ms_int.count() > 5) {
                if (s_dev_status_mutex.try_lock())
                {
                    struct rfnm_dev_status dev_status[1];

                    if (/*(rand() % 10 == 0) ||*/ control_transfer(RFNM_GET_DEV_STATUS, sizeof(struct rfnm_dev_status), (unsigned char*)&dev_status[0], 50) != RFNM_API_OK) {
                        spdlog::error("control_transfer for RFNM_GET_DEV_STATUS failed");
                        //return RFNM_API_USB_FAIL;

                        if (1 && ms_int.count() > 25 && s->transport_status.transport != TRANSPORT_ETH) {
                            spdlog::error("stopping stream");

                            for (int8_t i = 0; i < THREAD_COUNT; i++) {
                                thread_data[i].rx_active = 0;
                                thread_data[i].tx_active = 0;
                                thread_data[i].shutdown_req = 1;
                            }
                        }
                    }
                    else {
                        memcpy(&s->dev_status, &dev_status[0], sizeof(struct rfnm_dev_status));
                        s->last_dev_time = high_resolution_clock::now();
                    }

                    s_dev_status_mutex.unlock();
                }
            }
#endif        
        }
    }

    spdlog::error("exiting thread");
    if (lrxbuf) {
        delete lrxbuf;
    }
    spdlog::error("past delete");
    if (ltxbuf) {
        delete ltxbuf;
    }
    spdlog::error("past second delete");

    if (s->transport_status.transport == TRANSPORT_ETH) {
        //if (rfnm_data_ioctx_udp)
            rfnm_data_ioctx_udp.stop();

        if (rfnm_data_socket_udp.is_open()) {
            std::error_code ec;
            rfnm_data_socket_udp.close(ec);
            if (ec) {
                spdlog::warn("Error closing UDP socket: {}", ec.message());
            }
        }
    }
    

    if (s->transport_status.transport == TRANSPORT_LOCAL) {
#ifdef RFNM_COMPILE_LOCAL_TRANSPORT
        close(data_ep_fp);
#endif
    }
}




MSDLL std::vector<struct rfnm_dev_hwinfo> device::find(enum transport transport, std::string address, int bind) {
    //if (transport == TRANSPORT_ETH) {
    //    spdlog::error("Transport not supported");
    //    return {};
    //}

    int cnt = 0;
    int dev_cnt = 0;
    int r;
    std::vector<struct rfnm_dev_hwinfo> found = {};
    
    if (transport == TRANSPORT_USB || transport == TRANSPORT_FIND) {
        libusb_device** devs = NULL;

#if LIBUSB_API_VERSION >= 0x0100010A
        r = libusb_init_context(nullptr, nullptr, 0);
#else
        r = libusb_init(nullptr);
#endif
        if (r < 0) {
            spdlog::error("RFNMDevice::activateStream() -> failed to initialize libusb");
            goto exit_usb;
        }

        dev_cnt = libusb_get_device_list(NULL, &devs);
        if (dev_cnt < 0) {
            spdlog::error("failed to get list of usb devices");
            goto exit_usb;
        }

        for (int d = 0; d < dev_cnt; d++) {
            struct libusb_device_descriptor desc;
            libusb_device_handle* thandle{};
            int r = libusb_get_device_descriptor(devs[d], &desc);
            if (r < 0) {
                spdlog::error("failed to get usb dev descr");
                continue;
            }

            if (desc.idVendor != RFNM_USB_VID || desc.idProduct != RFNM_USB_PID) {
                continue;
            }

            r = libusb_open(devs[d], &thandle);
            if (r) {
                spdlog::error("Found RFNM device, but couldn't open it {}", r);
                continue;
            }

            if (address.length()) {
                uint8_t sn[9];
                if (libusb_get_string_descriptor_ascii(thandle, desc.iSerialNumber, sn, 9) >= 0) {
                    sn[8] = '\0';
                    if (strcmp((const char*)sn, address.c_str())) {
                        spdlog::info("This serial {} doesn't match the requested {}", (const char*)sn, address);
                        goto next;
                    }
                }
                else {
                    spdlog::error("Couldn't read serial descr");
                    goto next;
                }
            }

            if (libusb_get_device_speed(libusb_get_device(thandle)) < LIBUSB_SPEED_SUPER) {
                spdlog::error("You are connected using USB 2.0 (480 Mbps), however USB 3.0 (5000 Mbps) is required. "
                    "Please make sure that the cable and port you are using can work with USB 3.0 SuperSpeed");
                goto next;
            }

            r = libusb_claim_interface(thandle, 0);
            if (r < 0) {
                spdlog::error("Found RFNM device, but couldn't claim the interface, {}, {}", r, libusb_strerror((libusb_error)r));
                goto next;
            }

            struct rfnm_dev_hwinfo r_hwinfo;

            r = libusb_control_transfer(thandle, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                RFNM_GET_DEV_HWINFO, 0, (unsigned char*)&r_hwinfo, sizeof(struct rfnm_dev_hwinfo), 50);
            if (r < 0) {
                spdlog::error("libusb_control_transfer for REQ_HWINFO failed");
                goto next;
            }

            if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
                uint32_t pv = r_hwinfo.protocol_version;
                spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED detected protocol is v{} while your librfnm is for v{}", pv, RFNM_PROTOCOL_VERSION);
                goto next; 
            }

            found.push_back(r_hwinfo);

        next:
            libusb_release_interface(thandle, 0);
            libusb_close(thandle);
        }

    exit_usb:
        libusb_free_device_list(devs, 1);
        libusb_exit(NULL);
    }

    if (transport == TRANSPORT_LOCAL || transport == TRANSPORT_FIND) {
#ifdef RFNM_COMPILE_LOCAL_TRANSPORT
        int fd = open("/dev/rfnm_ctrl_ep", O_RDWR);
        if (fd < 0) {
            perror("open");
            goto exit_local;
        }

        uint8_t ep_ctrl_buf[RFNM_SYSCTL_TRANSFER_SIZE];

        if (ioctl(fd, RFNM_IOCTL_BASE + (0xff & RFNM_GET_DEV_HWINFO), &ep_ctrl_buf) < 0) {
            perror("ioctl GET_STATS");
            close(fd);
            goto exit_close_local;
        }

        struct rfnm_dev_hwinfo r_hwinfo;

        memcpy(&r_hwinfo, &ep_ctrl_buf[0], sizeof(struct rfnm_dev_hwinfo));

        if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
            uint32_t pv = r_hwinfo.protocol_version;
            spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED detected protocol is v{} while your librfnm is for v{}", pv, RFNM_PROTOCOL_VERSION);
            goto exit_close_local;
        }

        found.push_back(r_hwinfo);
        
exit_close_local:
        close(fd);
#endif
    }

exit_local:
    if (transport == TRANSPORT_ETH || transport == TRANSPORT_FIND) {
        
        try {
            asio::io_context io_context;

            std::string remote_addr;
            //spdlog::info("Detected broadcast address: {}", remote_addr);

            if (address.length()) {
                remote_addr = address;
            }
            else {
                remote_addr = get_broadcast_address();
                if (remote_addr.empty()) {
                    //spdlog::error("Failed to detect broadcast address.");
                    goto exit_eth;
                }
            }

            asio::ip::udp::endpoint remote_endpoint(asio::ip::make_address(remote_addr), RFNM_UDP_CTRL_PORT);

            asio::ip::udp::socket socket(io_context);
            socket.open(asio::ip::udp::v4());

#ifdef _WIN32
            DWORD timeout = 50; // 10 ms for Windows.
            if (setsockopt(socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                reinterpret_cast<const char*>(&timeout), sizeof(timeout)) < 0)
            {
                spdlog::error("Failed to set receive timeout on Windows");
            }
#else
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 50000; // 10,000 microseconds = 10 ms.
            if (setsockopt(socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                &tv, sizeof(tv)) < 0)
            {
                spdlog::error("Failed to set receive timeout on Linux/macOS");
            }
#endif

            if (!address.length()) {
                asio::socket_base::broadcast broadcast_option(true);
                socket.set_option(broadcast_option);
            }

            uint8_t message[1];
            message[0] = ((int)RFNM_GET_DEV_HWINFO) & 0xff;

            socket.send_to(asio::buffer(message), remote_endpoint);
            //spdlog::info("Sent RFNM_GET_DEV_HWINFO request.");

            //auto local_ep = socket.local_endpoint();
            //spdlog::info("Local ephemeral port assigned: {}", local_ep.port());

            char reply[1152];
            asio::ip::udp::endpoint sender_endpoint;
            size_t reply_length = socket.receive_from(asio::buffer(reply), sender_endpoint);
            //spdlog::info("Received {} bytes back from {}:{}", reply_length, sender_endpoint.address().to_string(), sender_endpoint.port());

            rfnm_dev_hwinfo r_hwinfo;
            if (reply_length == sizeof(rfnm_dev_hwinfo) + 1) {
                if (reply[0] != (RFNM_GET_DEV_HWINFO & 0xff)) {
                    spdlog::error("UDP control transfer conflict got {} should have been RFNM_GET_DEV_HWINFO", (uint8_t) reply[0]);
                    goto exit_eth;
                }
                memcpy(&r_hwinfo, &reply[1], sizeof(rfnm_dev_hwinfo));
                if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
                    uint32_t mv = r_hwinfo.protocol_version;
                    spdlog::error("Protocol version mismatch: received {}, expected {}", mv, RFNM_PROTOCOL_VERSION);
                    goto exit_eth;
                }
                found.push_back(r_hwinfo);
            }
            else {
                spdlog::error("Reply too short ({} bytes)", reply_length);
            }
        }
        catch (std::exception& e) {
            spdlog::error("UDP problem: {}", e.what());
        }
    }
exit_eth:
    return found;
}

MSDLL rfnm_api_failcode device::set_stream_format(enum stream_format format, size_t *bufsize) {
    if (stream_format_locked) {
        if (bufsize) {
            *bufsize = RFNM_USB_RX_PACKET_ELEM_CNT * s->transport_status.rx_stream_format;
        }

        if (format == s->transport_status.rx_stream_format) {
            return RFNM_API_OK;
        } else {
            return RFNM_API_NOT_SUPPORTED;
        }
    }

    switch (format) {
    case STREAM_FORMAT_CS8:
    case STREAM_FORMAT_CS16:
    case STREAM_FORMAT_CF32:
        s->transport_status.rx_stream_format = format;
        s->transport_status.tx_stream_format = format;
        if (bufsize) {
            *bufsize = RFNM_USB_RX_PACKET_ELEM_CNT * format;
        }
        break;
    default:
        if (bufsize) {
            *bufsize = 0;
        }
        return RFNM_API_NOT_SUPPORTED;
    }

    return RFNM_API_OK;
}

MSDLL rx_stream * device::rx_stream_create(uint8_t ch_ids) {
    stream_format_locked = true;
    return new rx_stream(*this, ch_ids);
}

MSDLL rfnm_api_failcode device::rx_work_start() {
    rfnm_api_failcode ret = RFNM_API_OK;

    rx_stream_count++;

    // no need to start workers if they're already running
    if (rx_stream_count > 1) return ret;

    stream_format_locked = true;

    // allocate buffers if the user didn't allocate them themselves
    if (!rx_s.qbuf_cnt) {
        rx_buffers_allocated = true;
        size_t bufsize = RFNM_USB_RX_PACKET_ELEM_CNT * s->transport_status.rx_stream_format;

        for (size_t i = 0; i < MIN_RX_BUFCNT; i++) {
            rx_buf *rxbuf = new rx_buf();
            rxbuf->buf = new uint8_t[bufsize];
            rx_qbuf(rxbuf, true);
        }
    }

    // expected CC of UINT64_MAX is a special value meaning to accept whatever comes
    for (int adc_id = 0; adc_id < 4; adc_id++) {
        rx_s.usb_cc[adc_id] = UINT64_MAX;
    }

    for (int8_t i = 0; i < THREAD_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
        thread_data[i].rx_active = 1;
        thread_data[i].cv.notify_one();
    }

    return ret;
}

MSDLL rfnm_api_failcode device::rx_work_stop() {
    rfnm_api_failcode ret = RFNM_API_OK;

    if (rx_stream_count > 0) rx_stream_count--;

    if (rx_stream_count == 0) {
        for (int8_t i = 0; i < THREAD_COUNT; i++) {
            std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
            thread_data[i].rx_active = 0;
        }
    }

    return ret;
}

MSDLL rfnm_api_failcode device::tx_work_start(enum tx_latency_policy policy) {
    rfnm_api_failcode ret = RFNM_API_OK;

    for (int8_t i = 0; i < THREAD_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
        thread_data[i].tx_active = 1;
        thread_data[i].cv.notify_one();
    }

    return ret;
}

MSDLL rfnm_api_failcode device::tx_work_stop() {
    rfnm_api_failcode ret = RFNM_API_OK;

    for (int8_t i = 0; i < THREAD_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
        thread_data[i].tx_active = 0;
    }

    return ret;
}

MSDLL rfnm_api_failcode device::rx_qbuf(struct rx_buf * buf, bool new_buffer) {
    std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
    if (new_buffer) rx_s.qbuf_cnt++;
    rx_s.in.push(buf);
    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::tx_qbuf(struct tx_buf* buf, uint32_t timeout_us) {
    //std::lock_guard<std::mutex> lockGuard1(tx_s.cc_mutex);
    std::lock_guard<std::mutex> lockGuard1(s_dev_status_mutex);

    if (tx_s.usb_cc - s->dev_status.usb_dac_last_dqbuf > 200) {
        return RFNM_API_MIN_QBUF_QUEUE_FULL;
    }

    std::lock_guard<std::mutex> lockGuard2(tx_s.in_mutex);

    tx_s.qbuf_cnt++;
    tx_s.usb_cc++;

    buf->usb_cc = (uint32_t)tx_s.usb_cc;
    tx_s.in.push(buf);

    return RFNM_API_OK;
}

MSDLL int device::single_ch_id_bitmap_to_adc_id(uint8_t ch_ids) {
    int ch_id = 0;
    while (ch_id < MAX_RX_CHANNELS) {
        if ((ch_ids & 0x1) == 1) {
            return s->rx.ch[ch_id].adc_id;
        }
        ch_id++;
        ch_ids = ch_ids >> 1;
    }
    return -1;
}

MSDLL void device::dqbuf_overwrite_cc(uint8_t adc_id, int acquire_lock) {
    if (acquire_lock) {
        rx_s.out_mutex.lock();
    }
    rx_s.in_mutex.lock();

    uint64_t old_cc = rx_s.usb_cc[adc_id];
    size_t queue_size = rx_s.out[adc_id].size();

    // use whatever buffer is at the top of the queue
    if (queue_size) {
        rx_s.usb_cc[adc_id] = rx_s.out[adc_id].top()->usb_cc;
    } else {
        rx_s.usb_cc[adc_id]++;
    }

    rx_s.in_mutex.unlock();
    if (acquire_lock) {
        rx_s.out_mutex.unlock();
    }

    spdlog::info("cc {} overwritten to {} at queue size {} adc {}",
        old_cc, rx_s.usb_cc[adc_id], queue_size, adc_id);
}

MSDLL int device::dqbuf_is_cc_continuous(uint8_t adc_id, int acquire_lock) {
    struct rx_buf* buf;
    size_t queue_size;

    if (acquire_lock) {
        rx_s.out_mutex.lock();
    }

    queue_size = rx_s.out[adc_id].size();
    if (queue_size < 1) {
        if (acquire_lock) {
            rx_s.out_mutex.unlock();
        }
        return 0;
    }

    buf = rx_s.out[adc_id].top();

    // special case for first buffer of stream
    if (rx_s.usb_cc[adc_id] == UINT64_MAX) {
        int ret = 0;

        // wait for at least 10 buffers to come in case they are out-of-order
        if (queue_size >= 10) {
            rx_s.usb_cc[adc_id] = buf->usb_cc;
            //spdlog::info("initial cc {} adc {}", rx_s.usb_cc[adc_id], adc_id);
            ret = 1;
        }

        if (acquire_lock) {
            rx_s.out_mutex.unlock();
        }
        return ret;
    }

    while (queue_size > 1) {
        if (buf->usb_cc < rx_s.usb_cc[adc_id]) {
            uint64_t usb_cc = buf->usb_cc;
            std::lock_guard<std::mutex> lockGuard(rx_s.in_mutex);
            rx_s.out[adc_id].pop();
            rx_s.in.push(buf);
            queue_size--;
            spdlog::info("stale cc {} discarded from adc {}", usb_cc, adc_id);
            buf = rx_s.out[adc_id].top();
        }
        else {
            break;
        }
    };

    if (acquire_lock) {
        rx_s.out_mutex.unlock();
    }

    if (rx_s.usb_cc[adc_id] == buf->usb_cc) {
        return 1;
    }
    else {
        if (queue_size > RX_RECOMB_BUF_LEN) {
            dqbuf_overwrite_cc(adc_id, acquire_lock);
        }
        return 0;
    }
}

MSDLL rfnm_api_failcode device::rx_dqbuf(struct rx_buf ** buf, uint8_t ch_ids, uint32_t timeout_us) {
    int is_single_ch, required_adc_id;

    if (rx_s.qbuf_cnt < MIN_RX_BUFCNT) {
        return RFNM_API_MIN_QBUF_CNT_NOT_SATIFIED;
    }

    switch (ch_ids) {
    case CH0:
    case CH1:
    case CH2:
    case CH3:
    case CH4:
    case CH5:
    case CH6:
    case CH7:
        is_single_ch = 1;
        break;
    case 0:
        is_single_ch = 0;
        ch_ids = 0xff;
        break;
    default:
        is_single_ch = 0;
        break;
    }

    if (is_single_ch) {
        required_adc_id = single_ch_id_bitmap_to_adc_id(ch_ids);
    }
    else {
        do {
            uint8_t mask = channel_flags[last_dqbuf_ch];
            required_adc_id = single_ch_id_bitmap_to_adc_id(ch_ids & mask);

            if (++last_dqbuf_ch == 8) {
                last_dqbuf_ch = 0;
            }
        } while (required_adc_id < 0);
    }

    if(!dqbuf_is_cc_continuous(required_adc_id, 1)) {
        if (!timeout_us) {
            return RFNM_API_DQBUF_NO_DATA;
        }

        {
            std::unique_lock lk(rx_s.out_mutex);
            rx_s.cv.wait_for(lk, std::chrono::microseconds(timeout_us),
                [this, required_adc_id] { return dqbuf_is_cc_continuous(required_adc_id, 0) ||
                                rx_s.out[required_adc_id].size() > RX_RECOMB_BUF_LEN; }
            );
        }

        if (!dqbuf_is_cc_continuous(required_adc_id, 1)) {
            if (timeout_us >= 10000) {
                spdlog::info("cc timeout {} adc {}", rx_s.usb_cc[required_adc_id], required_adc_id);
            }

            return RFNM_API_DQBUF_NO_DATA;
        }
    }

    {
        std::lock_guard<std::mutex> lockGuard(rx_s.out_mutex);
        *buf = rx_s.out[required_adc_id].top();
        rx_s.out[required_adc_id].pop();
    }

    struct rx_buf* lb;
    lb = *buf;

    rx_s.usb_cc[required_adc_id]++;

    //if ((lb->usb_cc & 0xff) < 0x10) {
    //    std::lock_guard<std::mutex> lockGuard(rx_s.out_mutex);
    //    spdlog::info("cc {} {} {}", lb->usb_cc, lcc, rx_s.out.size());
    //}

    //spdlog::info("cc {} adc {}", lb->usb_cc, lb->adc_id);

    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::rx_flush(uint32_t timeout_us, uint8_t ch_ids) {
    std::this_thread::sleep_for(std::chrono::microseconds(timeout_us));

    for (int ch_id = 0; ch_id < MAX_RX_CHANNELS; ch_id++) {
        if (!(ch_ids & channel_flags[ch_id])) continue;

        int adc_id = s->rx.ch[ch_id].adc_id;

        std::lock_guard lock_out(rx_s.out_mutex);

        while (rx_s.out[adc_id].size()) {
            struct rx_buf *buf = rx_s.out[adc_id].top();
            rx_s.out[adc_id].pop();
            std::lock_guard lock_in(rx_s.in_mutex);
            rx_s.in.push(buf);
        }

        rx_s.usb_cc[adc_id] = UINT64_MAX;
    }

    return RFNM_API_OK;
}

MSDLL enum rfnm_rf_path device::string_to_rf_path(std::string path) {
    std::transform(path.begin(), path.end(), path.begin(),
        [](unsigned char c) { return std::tolower(c); });

    if (!path.compare("embed") || !path.compare("emb") || !path.compare("embedded") || !path.compare("internal") || !path.compare("onboard")) {
        return RFNM_PATH_EMBED_ANT;
    }

    if (!path.compare("loop") || !path.compare("loopback")) {
        return RFNM_PATH_LOOPBACK;
    }

    if (path.find("sma") != std::string::npos) {
        path.replace(path.find("sma"), 3, "");
    }

    if (path.find("ant") != std::string::npos) {
        path.replace(path.find("ant"), 3, "");
    }

    if (path.find("-") != std::string::npos) {
        path.replace(path.find("-"), 1, "");
    }

    if (path.find("_") != std::string::npos) {
        path.replace(path.find("_"), 1, "");
    }

    if (path.find(" ") != std::string::npos) {
        path.replace(path.find(" "), 1, "");
    }

    if (path.length() != 1 || path.c_str()[0] < 'a' || path.c_str()[0] > 'h') {
        return RFNM_PATH_NULL;
    }

    return (enum rfnm_rf_path) (path.c_str()[0] - 'a');
}

MSDLL std::string device::rf_path_to_string(enum rfnm_rf_path path) {
    if (path == RFNM_PATH_NULL) {
        return "null";
    }
    else if (path == RFNM_PATH_EMBED_ANT) {
        return "embed";
    }
    else if (path == RFNM_PATH_LOOPBACK) {
        return "loopback";
    }
    else {
        return std::string(1, 'A' + (int)(path));
    }
}

MSDLL rfnm_api_failcode device::tx_dqbuf(struct tx_buf** buf) {
    std::lock_guard<std::mutex> lockGuard(tx_s.out_mutex);

    if (tx_s.out.size()) {
        *buf = tx_s.out.front();
        tx_s.out.pop();
        return RFNM_API_OK;
    }
    else {
        return RFNM_API_DQBUF_NO_DATA;
    }
}

void dump_udp_socket_state(const asio::ip::udp::socket& socket)
{
    try {
        // Check if the socket is open.
        bool isOpen = socket.is_open();
        spdlog::debug("UDP Socket is open: {}", isOpen);

        if (isOpen) {
            // Dump local endpoint information.
            try {
                asio::ip::udp::endpoint localEndpoint = socket.local_endpoint();
                spdlog::info("Local endpoint: {}:{}",
                    localEndpoint.address().to_string(),
                    localEndpoint.port());
            }
            catch (const std::exception& ex) {
                spdlog::error("Error retrieving local endpoint: {}", ex.what());
            }

            // Dump remote endpoint information (if available).
            try {
                asio::ip::udp::endpoint remoteEndpoint = socket.remote_endpoint();
                spdlog::info("Remote endpoint: {}:{}",
                    remoteEndpoint.address().to_string(),
                    remoteEndpoint.port());
            }
            catch (const std::exception& ex) {
                // It's common for UDP sockets not to have a remote endpoint until they are connected.
                spdlog::debug("Remote endpoint not set or error retrieving it: {}", ex.what());
            }

            // Optionally, dump additional socket options.
            try {
                asio::socket_base::receive_buffer_size rcvBufOpt;
                socket.get_option(rcvBufOpt);
                spdlog::info("Receive buffer size: {}", rcvBufOpt.value());
            }
            catch (const std::exception& ex) {
                spdlog::info("Could not get receive buffer size: {}", ex.what());
            }

            try {
                asio::socket_base::send_buffer_size sndBufOpt;
                socket.get_option(sndBufOpt);
                spdlog::info("Send buffer size: {}", sndBufOpt.value());
            }
            catch (const std::exception& ex) {
                spdlog::info("Could not get send buffer size: {}", ex.what());
            }
        }
    }
    catch (const std::exception& ex) {
        spdlog::error("Exception while dumping UDP socket state: {}", ex.what());
    }
}

MSDLL rfnm_api_failcode device::control_transfer(enum rfnm_control_ep type, uint32_t size, uint8_t* buf, uint32_t timeout_ms) {
    uint32_t r = -1;
    if (s->transport_status.transport == TRANSPORT_USB) {
        switch (type) {
        case RFNM_GET_DEV_HWINFO:
        case RFNM_GET_TX_CH_LIST:
        case RFNM_GET_RX_CH_LIST:
        case RFNM_GET_SET_RESULT:
        case RFNM_GET_DEV_STATUS:
        case RFNM_GET_SM_RESET:
        case RFNM_GET_LOCAL_MEMINFO:
            r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                type, 0, (unsigned char*) buf, size, timeout_ms);
            break;
        case RFNM_SET_TX_CH_LIST:
        case RFNM_SET_RX_CH_LIST:
        case RFNM_SET_DCS:
            r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_OUT) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                type, 0, (unsigned char*) buf, size, timeout_ms);
            break;
        }
        if (r < 0) {
            spdlog::error("libusb_control_transfer for req type {} failed with code {}", (int) type, r);
            return RFNM_API_USB_FAIL;
        }
        else {
            return RFNM_API_OK;
        }
    }
    if (s->transport_status.transport == TRANSPORT_LOCAL) {
#ifdef RFNM_COMPILE_LOCAL_TRANSPORT
        uint8_t ep_ctrl_buf[RFNM_SYSCTL_TRANSFER_SIZE];
        switch (type) {
        case RFNM_GET_DEV_HWINFO:
        case RFNM_GET_TX_CH_LIST:
        case RFNM_GET_RX_CH_LIST:
        case RFNM_GET_SET_RESULT:
        case RFNM_GET_DEV_STATUS:
        case RFNM_GET_SM_RESET:
        case RFNM_GET_LOCAL_MEMINFO:
            if (ioctl(rfnm_ctrl_ep_ioctl, RFNM_IOCTL_BASE + (0xff & type), &ep_ctrl_buf) < 0) {
                goto exit_error_local;
            }
            memcpy(buf, &ep_ctrl_buf[0], size);
            break;
        case RFNM_SET_TX_CH_LIST:
        case RFNM_SET_RX_CH_LIST:
        case RFNM_SET_DCS:
            memcpy(&ep_ctrl_buf[0], buf, size);
            if (ioctl(rfnm_ctrl_ep_ioctl, RFNM_IOCTL_BASE + (0xff & type), &ep_ctrl_buf) < 0) {
                goto exit_error_local;
            }
            break;
        }
        return RFNM_API_OK;
exit_error_local:
        spdlog::error("ioctl control transfer for req type {} failed with code {}", (int)type, r);
        return RFNM_API_USB_FAIL;
#endif
    }
    if (s->transport_status.transport == TRANSPORT_ETH) {
        try {

            uint8_t message[1152];
            message[0] = type & 0xff;

            memcpy(&message[1], buf, size);
            rfnm_ctrl_socket_udp->send_to(asio::buffer(message), rfnm_ctrl_ep_udp);
            char reply[1152];
            asio::ip::udp::endpoint sender_endpoint;
            //asio::ip::udp::endpoint sender_endpoint(asio::ip::udp::v4()); 
            size_t reply_length;

            switch (type) {
            case RFNM_GET_DEV_HWINFO:
            case RFNM_GET_TX_CH_LIST:
            case RFNM_GET_RX_CH_LIST:
            case RFNM_GET_SET_RESULT:
            case RFNM_GET_DEV_STATUS:
            case RFNM_GET_SM_RESET:
            case RFNM_GET_LOCAL_MEMINFO:
                
                /*if (type == RFNM_GET_DEV_STATUS) {
                    static int get_some = 0;
                    if (get_some++ > 20) {
                        break;
                    }
                }*/

                //dump_udp_socket_state(*rfnm_ctrl_socket_udp);
                //spdlog::info("before");

                reply_length = rfnm_ctrl_socket_udp->receive_from(asio::buffer(reply, 1152), sender_endpoint);

               // spdlog::info("after {}", reply_length);
                
                if (reply[0] != (type & 0xff)) {
                    spdlog::error("UDP control transfer conflict got {} should have been {}", (uint8_t)reply[0], (uint8_t)type);
                    return RFNM_API_USB_FAIL;
                }

                if (reply_length == size + 1) {
                    memcpy(buf, &reply[1], size);
                    break;
                }
                else {
                    spdlog::error("Control message reply too short ({} bytes)", reply_length);
                    return RFNM_API_USB_FAIL;
                }
            case RFNM_SET_TX_CH_LIST:
            case RFNM_SET_RX_CH_LIST:
            case RFNM_SET_DCS:
                break;
            }
            return RFNM_API_OK;
        }
        catch (std::exception& e) {
            //spdlog::error("UDP problem: {}", e.what());

            //if (type == RFNM_GET_DEV_STATUS) {
            //    return RFNM_API_OK;
            //}
            return RFNM_API_USB_FAIL;
        }
    }

    return RFNM_API_USB_FAIL;
}

MSDLL rfnm_api_failcode device::get(enum req_type type) {
    int r;

    if (type & REQ_HWINFO) {
        struct rfnm_dev_hwinfo r_hwinfo;
        if (control_transfer(RFNM_GET_DEV_HWINFO, sizeof(struct rfnm_dev_hwinfo), (unsigned char*)&r_hwinfo, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        memcpy(&s->hwinfo, &r_hwinfo, sizeof(struct rfnm_dev_hwinfo));

        if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
            uint32_t pv = r_hwinfo.protocol_version;
            spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED detected protocol is v{} while your librfnm is for v{}", pv, RFNM_PROTOCOL_VERSION);
            return RFNM_API_SW_UPGRADE_REQUIRED;
        }
    }

    if (type & REQ_TX) {
        struct rfnm_dev_tx_ch_list r_chlist;

        if (control_transfer(RFNM_GET_TX_CH_LIST, sizeof(struct rfnm_dev_tx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        memcpy(&s->tx, &r_chlist, sizeof(struct rfnm_dev_tx_ch_list));
    }

    if (type & REQ_RX) {
        struct rfnm_dev_rx_ch_list r_chlist;

        if (control_transfer(RFNM_GET_RX_CH_LIST, sizeof(struct rfnm_dev_rx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        memcpy(&s->rx, &r_chlist, sizeof(struct rfnm_dev_rx_ch_list));
    }

    if (type & REQ_DEV_STATUS) {
        struct rfnm_dev_status dev_status;

        if (control_transfer(RFNM_GET_DEV_STATUS, sizeof(struct rfnm_dev_status), (unsigned char*)&dev_status, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
        memcpy(&s->dev_status, &dev_status, sizeof(struct rfnm_dev_status));
    }

    return RFNM_API_OK;
}

MSDLL rfnm_api_failcode device::set(uint16_t applies, bool confirm_execution, uint32_t timeout_us) {
    int r;
    uint8_t applies_ch_tx = applies & 0xff;
    uint8_t applies_ch_rx = (applies & 0xff00) >> 8;

    if (applies_ch_tx) {
        struct rfnm_dev_tx_ch_list r_chlist;
        memcpy(&r_chlist, &s->tx, sizeof(struct rfnm_dev_tx_ch_list));
        r_chlist.apply = applies_ch_tx;
        r_chlist.cc = ++cc_tx;

        if (control_transfer(RFNM_SET_TX_CH_LIST, sizeof(struct rfnm_dev_tx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
    }

    if (applies_ch_rx) {
        struct rfnm_dev_rx_ch_list r_chlist;
        memcpy(&r_chlist, &s->rx, sizeof(struct rfnm_dev_rx_ch_list));
        r_chlist.apply = applies_ch_rx;
        r_chlist.cc = ++cc_rx;

        if (control_transfer(RFNM_SET_RX_CH_LIST, sizeof(struct rfnm_dev_rx_ch_list), (unsigned char*)&r_chlist, 50) != RFNM_API_OK) {
            return RFNM_API_USB_FAIL;
        }
    }

    if (confirm_execution) {
        using std::chrono::high_resolution_clock;
        using std::chrono::duration_cast;
        using std::chrono::duration;
        using std::chrono::microseconds;

        auto tstart = high_resolution_clock::now();

        while (1) {
            struct rfnm_dev_get_set_result r_res;

            if (control_transfer(RFNM_GET_SET_RESULT, sizeof(struct rfnm_dev_get_set_result), (unsigned char*)&r_res, 50) != RFNM_API_OK) {
                return RFNM_API_USB_FAIL;
            }

            if (r_res.cc_rx == cc_rx && r_res.cc_tx == cc_tx) {
                for (int q = 0; q < MAX_TX_CHANNELS; q++) {
                    if ((channel_flags[q] & applies_ch_tx) && r_res.tx_ecodes[q]) {
                        return (rfnm_api_failcode) r_res.tx_ecodes[q];
                    }
                }
                for (int q = 0; q < MAX_RX_CHANNELS; q++) {
                    if ((channel_flags[q] & applies_ch_rx) && r_res.rx_ecodes[q]) {
                        return (rfnm_api_failcode) r_res.rx_ecodes[q];
                    }
                }
                return RFNM_API_OK;
            }

            auto tnow = high_resolution_clock::now();
            auto us_int = duration_cast<microseconds>(tnow - tstart);

            if (us_int.count() > timeout_us) {
                return RFNM_API_TIMEOUT;
            }
        }
    }

    return RFNM_API_OK;
}


MSDLL rfnm_api_failcode device::set_dcs(uint64_t freq, uint32_t timeout_us) {
    if (control_transfer(RFNM_SET_DCS, sizeof(uint64_t), (unsigned char*)&freq, 50) != RFNM_API_OK) {
        return RFNM_API_USB_FAIL;
    }

    // uhm think about this design

    if (get(REQ_HWINFO)) {
        return RFNM_API_USB_FAIL;
    }

    return RFNM_API_OK;
} 

MSDLL const struct rfnm_dev_hwinfo * device::get_hwinfo() {
    return &(s->hwinfo);
}

MSDLL const struct rfnm_dev_status * device::get_dev_status() {
    return &(s->dev_status);
}

MSDLL const struct transport_status * device::get_transport_status() {
    return &(s->transport_status);
}

MSDLL const struct rfnm_api_rx_ch * device::get_rx_channel(uint32_t channel) {
    if (channel < MAX_RX_CHANNELS) {
        return &(s->rx.ch[channel]);
    } else {
        return nullptr;
    }
}

MSDLL const struct rfnm_api_tx_ch * device::get_tx_channel(uint32_t channel) {
    if (channel < MAX_RX_CHANNELS) {
        return &(s->tx.ch[channel]);
    } else {
        return nullptr;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_active(uint32_t channel, enum rfnm_ch_enable enable,
        enum rfnm_ch_stream stream, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].enable = enable;
        s->rx.ch[channel].stream = stream;

        if (apply) {
            return set(rx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

/*
MSDLL rfnm_api_failcode device::set_rx_channel_samp_freq_div(uint32_t channel, int16_t m, int16_t n, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].samp_freq_div_m = m;
        s->rx.ch[channel].samp_freq_div_n = n;

        if (apply) {
            return set(rx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}*/

MSDLL rfnm_api_failcode device::set_rx_channel_freq(uint32_t channel, int64_t freq, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].freq = freq;

        if (apply) {
            return set(rx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_rfic_lpf_bw(uint32_t channel, int16_t bw, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].rfic_lpf_bw = bw;

        if (apply) {
            return set(rx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_gain(uint32_t channel, int8_t gain, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].gain = gain;

        if (apply) {
            return set(rx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_agc(uint32_t channel, enum rfnm_agc_type agc, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].agc = agc;

        if (apply) {
            return set(rx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_fm_notch(uint32_t channel, enum rfnm_fm_notch fm_notch, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].fm_notch = fm_notch;

        if (apply) {
            return set(rx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_bias_tee(uint32_t channel, enum rfnm_bias_tee bias_tee, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].bias_tee = bias_tee;

        if (apply) {
            return set(rx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_rx_channel_path(uint32_t channel, enum rfnm_rf_path path, bool apply) {
    if (channel < MAX_RX_CHANNELS) {
        s->rx.ch[channel].path = path;

        if (apply) {
            return set(rx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_tx_channel_active(uint32_t channel, enum rfnm_ch_enable enable,
        enum rfnm_ch_stream stream, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].enable = enable;
        s->tx.ch[channel].stream = stream;

        if (apply) {
            return set(tx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}
/*
MSDLL rfnm_api_failcode device::set_tx_channel_samp_freq_div(uint32_t channel, int16_t m, int16_t n, bool apply)  {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].samp_freq_div_n = n;
        s->tx.ch[channel].samp_freq_div_m = m;

        if (apply) {
            return set(tx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}*/

MSDLL rfnm_api_failcode device::set_tx_channel_freq(uint32_t channel, int64_t freq, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].freq = freq;

        if (apply) {
            return set(tx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_tx_channel_rfic_lpf_bw(uint32_t channel, int16_t bw, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].rfic_lpf_bw = bw;

        if (apply) {
            return set(tx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_tx_channel_power(uint32_t channel, int8_t power, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].power = power;

        if (apply) {
            return set(tx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_tx_channel_bias_tee(uint32_t channel, enum rfnm_bias_tee bias_tee, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].bias_tee = bias_tee;

        if (apply) {
            return set(tx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}

MSDLL rfnm_api_failcode device::set_tx_channel_path(uint32_t channel, enum rfnm_rf_path path, bool apply) {
    if (channel < MAX_TX_CHANNELS) {
        s->tx.ch[channel].path = path;

        if (apply) {
            return set(tx_channel_apply_flags[channel]);
        } else {
            return RFNM_API_OK;
        }
    } else {
        return RFNM_API_NOT_SUPPORTED;
    }
}


MSDLL uint32_t device::get_rx_channel_count() {
    uint32_t count =  s->hwinfo.daughterboard[0].rx_ch_cnt + s->hwinfo.daughterboard[1].rx_ch_cnt;

    // should never happen unless firmware malfunctions
    if (count > MAX_RX_CHANNELS) {
        count = MAX_RX_CHANNELS;
    }

    return count;
}

MSDLL uint32_t device::get_tx_channel_count() {
    uint32_t count =  s->hwinfo.daughterboard[0].tx_ch_cnt + s->hwinfo.daughterboard[1].tx_ch_cnt;

    // should never happen unless firmware malfunctions
    if (count > MAX_TX_CHANNELS) {
        count = MAX_TX_CHANNELS;
    }

    return count;
}

#ifdef _WIN32
#include <winsock2.h>
#include <iphlpapi.h>
#pragma comment(lib, "Iphlpapi.lib")
#pragma comment(lib, "Ws2_32.lib")
#else
#include <ifaddrs.h>
#include <net/if.h>
#include <arpa/inet.h>
#endif

// Helper: Compute broadcast address from IP and netmask.
std::string rfnm::compute_broadcast_address(const std::string& ip_str, const std::string& mask_str)
{
    in_addr ip, mask, broadcast;
    if (inet_pton(AF_INET, ip_str.c_str(), &ip) != 1)
        throw std::runtime_error("Invalid IP address: " + ip_str);
    if (inet_pton(AF_INET, mask_str.c_str(), &mask) != 1)
        throw std::runtime_error("Invalid netmask: " + mask_str);

    // Compute broadcast = ip OR (~mask)
    broadcast.s_addr = ip.s_addr | ~(mask.s_addr);

    char buf[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &broadcast, buf, INET_ADDRSTRLEN) == nullptr)
        throw std::runtime_error("Failed to convert broadcast address to string");
    return std::string(buf);
}


static std::string rfnm::get_broadcast_address() {
#ifdef _WIN32
    // Initialize Winsock (if not already done by your application).
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        spdlog::error("WSAStartup failed");
        return "";
    }

    // Allocate a buffer for adapter addresses.
    ULONG flags = GAA_FLAG_INCLUDE_PREFIX;
    ULONG family = AF_INET; // IPv4 only.
    ULONG bufferSize = 15000;
    std::vector<char> buffer(bufferSize);
    PIP_ADAPTER_ADDRESSES pAddresses = reinterpret_cast<PIP_ADAPTER_ADDRESSES>(buffer.data());

    DWORD ret = GetAdaptersAddresses(family, flags, nullptr, pAddresses, &bufferSize);
    if (ret == ERROR_BUFFER_OVERFLOW) {
        buffer.resize(bufferSize);
        pAddresses = reinterpret_cast<PIP_ADAPTER_ADDRESSES>(buffer.data());
        ret = GetAdaptersAddresses(family, flags, nullptr, pAddresses, &bufferSize);
    }
    if (ret != NO_ERROR) {
        spdlog::error("GetAdaptersAddresses failed: {}", ret);
        WSACleanup();
        return "";
    }

    // Iterate over the adapters.
    for (PIP_ADAPTER_ADDRESSES adapter = pAddresses; adapter; adapter = adapter->Next) {
        // Skip if not up or if it's a loopback.
        if (adapter->OperStatus != IfOperStatusUp)
            continue;
        if (adapter->IfType == IF_TYPE_SOFTWARE_LOOPBACK)
            continue;

        // Skip adapters with names containing "vEthernet".
        if (adapter->Description && wcsstr(adapter->Description, L"Virtual") != nullptr) {
            //spdlog::info("Skipping virtual adapter (Description): {}", adapter->Description);
            continue;
        }

        // Filter out non-physical interfaces based on type.
        // Allow Ethernet and Wireless (Wi-Fi). Add additional physical types if needed.
        if (adapter->IfType != IF_TYPE_ETHERNET_CSMACD &&
            adapter->IfType != IF_TYPE_IEEE80211) {
            spdlog::info("Skipping non-physical adapter: {}", adapter->AdapterName);
            continue;
        }

        // Iterate over unicast addresses.
        for (PIP_ADAPTER_UNICAST_ADDRESS ua = adapter->FirstUnicastAddress; ua; ua = ua->Next) {
            if (ua->Address.lpSockaddr->sa_family == AF_INET) {
                // Get the IP address.
                sockaddr_in* sa_in = reinterpret_cast<sockaddr_in*>(ua->Address.lpSockaddr);
                char ipBuf[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &(sa_in->sin_addr), ipBuf, INET_ADDRSTRLEN);
                std::string ipStr(ipBuf);

                // Get the netmask from the OnLinkPrefixLength.
                ULONG prefix = ua->OnLinkPrefixLength;
                uint32_t mask_int = 0xFFFFFFFF << (32 - prefix);
                in_addr mask_addr;
                mask_addr.s_addr = htonl(mask_int);
                char maskBuf[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &mask_addr, maskBuf, INET_ADDRSTRLEN);
                std::string maskStr(maskBuf);

                try {
                    std::string broadcast = compute_broadcast_address(ipStr, maskStr);
                    //spdlog::info("Using adapter {}: IP = {}, Netmask = {}, Broadcast = {}", adapter->AdapterName, ipStr, maskStr, broadcast);
                    WSACleanup();
                    return broadcast;
                }
                catch (const std::exception& ex) {
                    spdlog::error("Error computing broadcast address: {}", ex.what());
                    WSACleanup();
                    return "";
                }
            }
        }
    }
    WSACleanup();
    return "";
#else
    // Linux / macOS implementation using getifaddrs.
    struct ifaddrs* ifaddr = nullptr, * ifa = nullptr;
    std::string broadcast_ip;

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return "";
    }

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr)
            continue;
        if (ifa->ifa_addr->sa_family == AF_INET &&
            (ifa->ifa_flags & IFF_UP) &&
            !(ifa->ifa_flags & IFF_LOOPBACK))
        {
            // Filter out interfaces with names that suggest virtual or non-physical interfaces.
            // For example, skip if name starts with "vir", "vmnet", "docker", or contains "vEthernet".
            std::string ifname(ifa->ifa_name);
            if (ifname.find("vir") == 0 ||
                ifname.find("vmnet") == 0 ||
                ifname.find("docker") == 0 ||
                ifname.find("vEthernet") != std::string::npos ||
                ifname.find("lo") == 0) {
                spdlog::info("Skipping virtual/non-physical interface: {}", ifname);
                continue;
            }

            // Get the IP address.
            struct sockaddr_in* sa = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
            char ipBuf[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &sa->sin_addr, ipBuf, INET_ADDRSTRLEN);
            std::string ipStr(ipBuf);

            // Get netmask.
            struct sockaddr_in* nm = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_netmask);
            char maskBuf[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &nm->sin_addr, maskBuf, INET_ADDRSTRLEN);
            std::string maskStr(maskBuf);

            try {
                broadcast_ip = compute_broadcast_address(ipStr, maskStr);
                //spdlog::info("Using interface {}: IP = {}, Netmask = {}, Broadcast = {}", ifa->ifa_name, ipStr, maskStr, broadcast_ip);
                break;  // Use the first eligible interface.
            }
            catch (const std::exception& ex) {
                spdlog::error("Error computing broadcast address: {}", ex.what());
            }
        }
    }

    freeifaddrs(ifaddr);
    return broadcast_ip;
#endif
}
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

    if (transport != LIBRFNM_TRANSPORT_USB) {
        spdlog::error("Transport not supported");
        throw std::runtime_error("Transport not supported");
    }

    s = new struct status();
    usb_handle = new _usb_handle;

    int cnt = 0;
    int dev_cnt = 0;
    int r;
    std::vector<struct rfnm_dev_hwinfo> found;
    libusb_device** devs = NULL;

    // set default/native stream format
    s->transport_status.rx_stream_format = LIBRFNM_STREAM_FORMAT_CS16;
    s->transport_status.tx_stream_format = LIBRFNM_STREAM_FORMAT_CS16;

#if LIBUSB_API_VERSION >= 0x0100010A
    r = libusb_init_context(nullptr, nullptr, 0);
#else
    r = libusb_init(nullptr);
#endif
    if (r < 0) {
        spdlog::error("RFNMDevice::activateStream() -> failed to initialize libusb");
        goto error;
    }

    dev_cnt = libusb_get_device_list(NULL, &devs);
    if (dev_cnt < 0) {
        spdlog::error("failed to get list of usb devices");
        goto error;
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

        if (get(LIBRFNM_REQ_ALL)) {
            goto next;
        }

        libusb_free_device_list(devs, 1);

        for (int8_t i = 0; i < LIBRFNM_THREAD_COUNT; i++) {
            thread_data[i].ep_id = i + 1;
            thread_data[i].rx_active = 0;
            thread_data[i].tx_active = 0;
            thread_data[i].shutdown_req = 0;
        }

        for (int i = 0; i < LIBRFNM_THREAD_COUNT; i++) {
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

error:
    libusb_free_device_list(devs, 1);
    libusb_exit(NULL);
    delete s;
    delete usb_handle;
    throw std::runtime_error("RFNM initialization failure");
}

MSDLL device::~device() {
    for (int8_t i = 0; i < LIBRFNM_THREAD_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
        thread_data[i].rx_active = 0;
        thread_data[i].tx_active = 0;
        thread_data[i].shutdown_req = 1;
        thread_data[i].cv.notify_one();
    }

    for (auto& i : thread_c) {
        i.join();
    }

    delete s;

    if (rx_buffers_allocated) {
        rx_flush();

        // no need to take in_mutex as threads are finished
        while (rx_s.in.size()) {
            rx_buf *rxbuf = rx_s.in.front();
            rx_s.in.pop();
            delete[] rxbuf->buf;
            delete rxbuf;
        }
    }

    if (usb_handle->primary) {
        libusb_release_interface(usb_handle->primary, 0);
        libusb_close(usb_handle->primary);
    }
    if (usb_handle->boost) {
        libusb_release_interface(usb_handle->boost, 0);
        libusb_close(usb_handle->boost);
    }

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
}

void device::threadfn(size_t thread_index) {
    struct rfnm_rx_usb_buf* lrxbuf = new rfnm_rx_usb_buf();
    struct rfnm_tx_usb_buf* ltxbuf = new rfnm_tx_usb_buf();
    int transferred;
    auto& tpm = thread_data[thread_index];
    int r;

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

            if (lrxbuf->magic != 0x7ab8bd6f || lrxbuf->adc_id > 3) {
                //spdlog::error("Wrong magic");
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

            if (s->transport_status.rx_stream_format == LIBRFNM_STREAM_FORMAT_CS8) {
                unpack_12_to_cs8(buf->buf, (uint8_t*)lrxbuf->buf, RFNM_USB_RX_PACKET_ELEM_CNT);
            }
            else if (s->transport_status.rx_stream_format == LIBRFNM_STREAM_FORMAT_CS16) {
                unpack_12_to_cs16(buf->buf, (uint8_t*)lrxbuf->buf, RFNM_USB_RX_PACKET_ELEM_CNT);
            }
            else if (s->transport_status.rx_stream_format == LIBRFNM_STREAM_FORMAT_CF32) {
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

            {
                std::lock_guard<std::mutex> lockGuard(tx_s.out_mutex);
                tx_s.out.push(buf);
                tx_s.cv.notify_one();
            }
        }

read_dev_status:

        {
            using std::chrono::high_resolution_clock;
            using std::chrono::duration_cast;
            using std::chrono::duration;
            using std::chrono::milliseconds;

            auto tlast = s->last_dev_time;
            auto tnow = high_resolution_clock::now();
            auto ms_int = duration_cast<milliseconds>(tnow - tlast);

            if (ms_int.count() > 5) {
                if (s_dev_status_mutex.try_lock())
                {
                    struct rfnm_dev_status dev_status;

                    r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                            RFNM_GET_DEV_STATUS, 0, (unsigned char*)&dev_status, sizeof(struct rfnm_dev_status), 50);
                    if (r < 0) {
                        spdlog::error("libusb_control_transfer for RFNM_GET_DEV_STATUS failed");
                        //return RFNM_API_USB_FAIL;

                        if (ms_int.count() > 25) {
                            spdlog::error("stopping stream");

                            for (int8_t i = 0; i < LIBRFNM_THREAD_COUNT; i++) {
                                thread_data[i].rx_active = 0;
                                thread_data[i].tx_active = 0;
                                thread_data[i].shutdown_req = 1;
                            }
                        }
                    }
                    else {
                        memcpy(&s->dev_status, &dev_status, sizeof(struct rfnm_dev_status));
                        s->last_dev_time = high_resolution_clock::now();
                    }

                    s_dev_status_mutex.unlock();
                }
            }
        }
    }

    delete lrxbuf;
    delete ltxbuf;
}

MSDLL std::vector<struct rfnm_dev_hwinfo> device::find(enum transport transport, std::string address, int bind) {
    if (transport != LIBRFNM_TRANSPORT_USB) {
        spdlog::error("Transport not supported");
        return {};
    }

    int cnt = 0;
    int dev_cnt = 0;
    int r;
    std::vector<struct rfnm_dev_hwinfo> found = {};
    libusb_device** devs = NULL;

#if LIBUSB_API_VERSION >= 0x0100010A
    r = libusb_init_context(nullptr, nullptr, 0);
#else
    r = libusb_init(nullptr);
#endif
    if (r < 0) {
        spdlog::error("RFNMDevice::activateStream() -> failed to initialize libusb");
        goto exit;
    }

    dev_cnt = libusb_get_device_list(NULL, &devs);
    if (dev_cnt < 0) {
        spdlog::error("failed to get list of usb devices");
        goto exit;
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
                if(strcmp((const char*)sn, address.c_str())) {
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
            spdlog::error("libusb_control_transfer for LIBRFNM_REQ_HWINFO failed");
            goto next;
        }

        if (r_hwinfo.protocol_version != 1) {
            spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED");
            goto next;
        }

        found.push_back(r_hwinfo);

next:
        libusb_release_interface(thandle, 0);
        libusb_close(thandle);
    }

exit:
    libusb_free_device_list(devs, 1);
    libusb_exit(NULL);
    return found;
}

MSDLL rfnm_api_failcode device::set_stream_format(enum stream_format format, size_t *bufsize) {
    if (stream_format_locked) {
        if (bufsize) {
            *bufsize = RFNM_USB_RX_PACKET_ELEM_CNT * s->transport_status.rx_stream_format;
        }
        return RFNM_API_NOT_SUPPORTED;
    }

    switch (format) {
    case LIBRFNM_STREAM_FORMAT_CS8:
    case LIBRFNM_STREAM_FORMAT_CS16:
    case LIBRFNM_STREAM_FORMAT_CF32:
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

        for (size_t i = 0; i < LIBRFNM_MIN_RX_BUFCNT; i++) {
            rx_buf *rxbuf = new rx_buf();
            rxbuf->buf = new uint8_t[bufsize];
            rx_qbuf(rxbuf, true);
        }
    }

    // expected CC of UINT64_MAX is a special value meaning to accept whatever comes
    for (int adc_id = 0; adc_id < 4; adc_id++) {
        rx_s.usb_cc[adc_id] = UINT64_MAX;
    }

    for (int8_t i = 0; i < LIBRFNM_THREAD_COUNT; i++) {
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
        for (int8_t i = 0; i < LIBRFNM_THREAD_COUNT; i++) {
            std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
            thread_data[i].rx_active = 0;
        }
    }

    return ret;
}

MSDLL rfnm_api_failcode device::tx_work_start(enum tx_latency_policy policy) {
    rfnm_api_failcode ret = RFNM_API_OK;

    for (int8_t i = 0; i < LIBRFNM_THREAD_COUNT; i++) {
        std::lock_guard<std::mutex> lockGuard(thread_data[i].cv_mutex);
        thread_data[i].tx_active = 1;
        thread_data[i].cv.notify_one();
    }

    return ret;
}

MSDLL rfnm_api_failcode device::tx_work_stop() {
    rfnm_api_failcode ret = RFNM_API_OK;

    for (int8_t i = 0; i < LIBRFNM_THREAD_COUNT; i++) {
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

    if (tx_s.usb_cc - s->dev_status.usb_dac_last_dqbuf > 100) {
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

    spdlog::info("cc {} overwritten to {} at queue size {} adc {}",
            old_cc, rx_s.usb_cc[adc_id], queue_size, adc_id);

    rx_s.in_mutex.unlock();
    if (acquire_lock) {
        rx_s.out_mutex.unlock();
    }
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
        if (queue_size > LIBRFNM_RX_RECOMB_BUF_LEN) {
            dqbuf_overwrite_cc(adc_id, acquire_lock);
        }
        return 0;
    }
}

MSDLL rfnm_api_failcode device::rx_dqbuf(struct rx_buf ** buf, uint8_t ch_ids, uint32_t timeout_us) {
    int is_single_ch, required_adc_id;

    if (rx_s.qbuf_cnt < LIBRFNM_MIN_RX_BUFCNT) {
        return RFNM_API_MIN_QBUF_CNT_NOT_SATIFIED;
    }

    switch (ch_ids) {
    case LIBRFNM_CH0:
    case LIBRFNM_CH1:
    case LIBRFNM_CH2:
    case LIBRFNM_CH3:
    case LIBRFNM_CH4:
    case LIBRFNM_CH5:
    case LIBRFNM_CH6:
    case LIBRFNM_CH7:
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
                                rx_s.out[required_adc_id].size() > LIBRFNM_RX_RECOMB_BUF_LEN; }
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

MSDLL rfnm_api_failcode device::get(enum req_type type) {
    int r;

    if (type & LIBRFNM_REQ_HWINFO) {
        struct rfnm_dev_hwinfo r_hwinfo;

        r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                RFNM_GET_DEV_HWINFO, 0, (unsigned char*)&r_hwinfo, sizeof(struct rfnm_dev_hwinfo), 50);
        if (r < 0) {
            spdlog::error("libusb_control_transfer for LIBRFNM_REQ_HWINFO failed");
            return RFNM_API_USB_FAIL;
        }
        memcpy(&s->hwinfo, &r_hwinfo, sizeof(struct rfnm_dev_hwinfo));

        if (r_hwinfo.protocol_version != 1) {
            spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED");
            return RFNM_API_SW_UPGRADE_REQUIRED;
        }
    }

    if (type & LIBRFNM_REQ_TX) {
        struct rfnm_dev_tx_ch_list r_chlist;

        r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                RFNM_GET_TX_CH_LIST, 0, (unsigned char*)&r_chlist, sizeof(struct rfnm_dev_tx_ch_list), 50);
        if (r < 0) {
            spdlog::error("libusb_control_transfer for LIBRFNM_REQ_TX failed");
            return RFNM_API_USB_FAIL;
        }
        memcpy(&s->tx, &r_chlist, sizeof(struct rfnm_dev_tx_ch_list));
    }

    if (type & LIBRFNM_REQ_RX) {
        struct rfnm_dev_rx_ch_list r_chlist;

        r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                RFNM_GET_RX_CH_LIST, 0, (unsigned char*)&r_chlist, sizeof(struct rfnm_dev_rx_ch_list), 50);
        if (r < 0) {
            spdlog::error("libusb_control_transfer for LIBRFNM_REQ_RX failed");
            return RFNM_API_USB_FAIL;
        }
        memcpy(&s->rx, &r_chlist, sizeof(struct rfnm_dev_rx_ch_list));
    }

    if (type & LIBRFNM_REQ_DEV_STATUS) {
        struct rfnm_dev_status dev_status;

        r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                RFNM_GET_DEV_STATUS, 0, (unsigned char*)&dev_status, sizeof(struct rfnm_dev_status), 50);
        if (r < 0) {
            spdlog::error("libusb_control_transfer for RFNM_GET_DEV_STATUS failed {}", r);
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

        r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_OUT) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                RFNM_SET_TX_CH_LIST, 0, (unsigned char*)&r_chlist, sizeof(struct rfnm_dev_tx_ch_list), 50);
        if (r < 0) {
            spdlog::error("libusb_control_transfer for LIBRFNM_REQ_TX failed");
            return RFNM_API_USB_FAIL;
        }
    }

    if (applies_ch_rx) {
        struct rfnm_dev_rx_ch_list r_chlist;
        memcpy(&r_chlist, &s->rx, sizeof(struct rfnm_dev_rx_ch_list));
        r_chlist.apply = applies_ch_rx;
        r_chlist.cc = ++cc_rx;

        r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_OUT) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                RFNM_SET_RX_CH_LIST, 0, (unsigned char*)&r_chlist, sizeof(struct rfnm_dev_rx_ch_list), 50);
        if (r < 0) {
            spdlog::error("libusb_control_transfer for LIBRFNM_REQ_RX failed");
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

            r = libusb_control_transfer(usb_handle->primary, uint8_t(LIBUSB_ENDPOINT_IN) | uint8_t(LIBUSB_REQUEST_TYPE_VENDOR), RFNM_B_REQUEST,
                    RFNM_GET_SET_RESULT, 0, (unsigned char*)&r_res, sizeof(struct rfnm_dev_get_set_result), 50);
            if (r < 0) {
                spdlog::error("libusb_control_transfer for LIBRFNM_REQ_RX failed");
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
}

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
}

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
    return s->hwinfo.daughterboard[0].rx_ch_cnt + s->hwinfo.daughterboard[1].rx_ch_cnt;
}

MSDLL uint32_t device::get_tx_channel_count() {
    return s->hwinfo.daughterboard[0].tx_ch_cnt + s->hwinfo.daughterboard[1].tx_ch_cnt;
}

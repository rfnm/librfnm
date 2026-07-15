// Local transport (on-device): zero-copy packet pools mmap'd from the kernel, control
// verbs as chardev ioctls. Compiled only where BUILD_RFNM_LOCAL_TRANSPORT is set (the
// aarch64 board build).

#ifdef BUILD_RFNM_LOCAL_TRANSPORT

#include <cstring>
#include <cstdio>
#include <cerrno>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <spdlog/spdlog.h>

#include "impl.h"

using namespace rfnm;

// zero-copy local transport: the kernel packet pools, mmap'd from /dev/rfnm_data_ep on
// the first local open and kept for the process lifetime (they alias one fixed system
// carve-out, so they are process-global by nature - NOT device members). Worker threads
// read RX packets and build TX packets in place, moving only pool indices through
// their data-ep ioctls.
static struct rfnm_local_rx_pkt* local_rx_pool = nullptr;
static struct rfnm_local_tx_pkt* local_tx_pool = nullptr;
static size_t local_rx_pool_cnt = 0;
static size_t local_tx_pool_cnt = 0;
static std::mutex local_pool_mutex;

bool device::impl::local_open() {
    int fd = open("/dev/rfnm_ctrl_ep", O_RDWR);
    if (fd < 0) {
        return false;
    }
    uint8_t ep_ctrl_buf[RFNM_SYSCTL_TRANSFER_SIZE];

    if (ioctl(fd, RFNM_IOCTL_BASE + (0xff & RFNM_GET_DEV_HWINFO), &ep_ctrl_buf) < 0) {
        close(fd);
        goto exit_close_local;
    }

    rfnm_ctrl_ep_ioctl = fd;

    {
        struct rfnm_dev_hwinfo r_hwinfo;

        memcpy(&r_hwinfo, &ep_ctrl_buf[0], sizeof(struct rfnm_dev_hwinfo));

        if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
            uint32_t pv = r_hwinfo.protocol_version;
            spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED detected protocol is v{} while your librfnm is for v{}", pv, RFNM_PROTOCOL_VERSION);
            goto exit_close_local;
        }
    }

    transport_status.theoretical_mbps = 3500;

    // RFNM_NO_SM_RESET: open without resetting the stream state machine, so a second
    // local client (e.g. an RX-only observer) can coexist with a running daemon's
    // stream instead of killing it. Bench/measurement knob - the reset stays the default.
    if (!getenv("RFNM_NO_SM_RESET")) {
        if (ioctl(fd, RFNM_IOCTL_BASE + (0xff & RFNM_GET_SM_RESET), &ep_ctrl_buf) < 0) {
            spdlog::error("Couldn't reset state machine");
            goto exit_close_local;
        }
    }
    reset_session();

    transport_status.transport = TRANSPORT_LOCAL;

    // zero-copy local data path: map the kernel packet pools (process-global,
    // first local open in this process does the work)
    {
        std::lock_guard<std::mutex> lockGuard(local_pool_mutex);
        if (local_rx_pool == nullptr) {
            struct rfnm_local_transport_meminfo meminfo;
            int pool_fd;

            if (ioctl(fd, RFNM_IOCTL_BASE + (0xff & RFNM_GET_LOCAL_MEMINFO), &ep_ctrl_buf) < 0) {
                spdlog::error("Couldn't fetch local transport meminfo");
                goto exit_close_local;
            }
            memcpy(&meminfo, &ep_ctrl_buf[0], sizeof(struct rfnm_local_transport_meminfo));
            local_rx_pool_cnt = meminfo.local_rx_bufsize;
            local_tx_pool_cnt = meminfo.local_tx_bufsize;

            // the mappings outlive the fd, so it closes right after; they alias a
            // fixed carve-out and are kept for the process lifetime (no munmap)
            pool_fd = open("/dev/rfnm_data_ep", O_RDWR);
            if (pool_fd < 0) {
                spdlog::error("Couldn't open data ep for pool mapping");
                goto exit_close_local;
            }
            local_rx_pool = (struct rfnm_local_rx_pkt*)mmap(NULL,
                local_rx_pool_cnt * sizeof(struct rfnm_local_rx_pkt),
                PROT_READ, MAP_SHARED, pool_fd, RFNM_LOCAL_MMAP_RX_OFFSET);
            local_tx_pool = (struct rfnm_local_tx_pkt*)mmap(NULL,
                local_tx_pool_cnt * sizeof(struct rfnm_local_tx_pkt),
                PROT_READ | PROT_WRITE, MAP_SHARED, pool_fd, RFNM_LOCAL_MMAP_TX_OFFSET);
            close(pool_fd);
            if (local_rx_pool == MAP_FAILED || local_tx_pool == MAP_FAILED) {
                spdlog::error("Couldn't map local packet pools");
                if (local_rx_pool != MAP_FAILED) {
                    munmap(local_rx_pool, local_rx_pool_cnt * sizeof(struct rfnm_local_rx_pkt));
                }
                if (local_tx_pool != MAP_FAILED) {
                    munmap(local_tx_pool, local_tx_pool_cnt * sizeof(struct rfnm_local_tx_pkt));
                }
                local_rx_pool = nullptr;
                local_tx_pool = nullptr;
                goto exit_close_local;
            }
        }
    }

    if (get(REQ_ALL)) {
        goto exit_close_local;
    }

    return true;

exit_close_local:
    close(fd);
    return false;
}

void rfnm::local_find_into(std::vector<struct dev_info> &found) {
    int fd = open("/dev/rfnm_ctrl_ep", O_RDWR);
    if (fd < 0) {
        perror("open");
        return;
    }

    uint8_t ep_ctrl_buf[RFNM_SYSCTL_TRANSFER_SIZE];

    if (ioctl(fd, RFNM_IOCTL_BASE + (0xff & RFNM_GET_DEV_HWINFO), &ep_ctrl_buf) < 0) {
        perror("ioctl GET_STATS");
        close(fd);
        goto exit_close_local;
    }

    {
        struct rfnm_dev_hwinfo r_hwinfo;

        memcpy(&r_hwinfo, &ep_ctrl_buf[0], sizeof(struct rfnm_dev_hwinfo));

        if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
            uint32_t pv = r_hwinfo.protocol_version;
            spdlog::error("RFNM_API_SW_UPGRADE_REQUIRED detected protocol is v{} while your librfnm is for v{}", pv, RFNM_PROTOCOL_VERSION);
            goto exit_close_local;
        }

        if (sn_already_found(found, TRANSPORT_LOCAL, r_hwinfo.motherboard.serial_number)) {
            spdlog::info("Skip local device (duplicate SN): {}", (char*)r_hwinfo.motherboard.serial_number);
            goto exit_close_local;
        }
        spdlog::info("Add local device: {}", (char*)r_hwinfo.motherboard.serial_number);
        found.push_back({ TRANSPORT_LOCAL, "", r_hwinfo });
    }

exit_close_local:
    close(fd);
}

rfnm_api_failcode device::impl::local_ctrl_transfer(enum rfnm_control_ep type, uint32_t size, uint8_t* buf, uint32_t timeout_ms) {
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
    case RFNM_SET_SAMP_RATE:
    case RFNM_SET_TDD:
    case RFNM_SET_TXN:
        memcpy(&ep_ctrl_buf[0], buf, size);
        if (ioctl(rfnm_ctrl_ep_ioctl, RFNM_IOCTL_BASE + (0xff & type), &ep_ctrl_buf) < 0) {
            // LOCAL is the one transport with a synchronous reject path: the txn
            // push errno says WHY (USB/TCP schedule SETs are fire-and-forget)
            if (type == RFNM_SET_TXN) {
                switch (errno) {
                case ETIME: return RFNM_API_SCHED_LATE;
                case ENOSPC: return RFNM_API_SCHED_FULL;
                case EINVAL: return RFNM_API_SCHED_ORDER;
                case ENXIO: return RFNM_API_SCHED_NOT_RESET;
                }
            }
            goto exit_error_local;
        }
        break;
    default:
        goto exit_error_local;
    }
    return RFNM_API_OK;
exit_error_local:
    spdlog::error("ioctl control transfer for req type {} failed with errno {}", (int)type, errno);
    return RFNM_API_USB_FAIL;
}

rx_pass_result device::impl::local_rx_fetch(worker_ctx &c) {
    struct pollfd pfd;
    pfd.fd = c.data_ep_fp;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, /* timeout_ms= */ 5);
    if (ret <= 0 || !(pfd.revents & POLLIN)) {
        // poll timeout: no local packet within 5 ms. Normal idle tick at low sample
        // rates (a 2.4 MSps stream fills a packet slower than the poll period), so it
        // is not logged.
        if (ret < 0) {
            perror("poll");
        }
        return rx_pass_result::skip;
    }
    // zero-copy: take the index of the next filled pool packet and read it in
    // place; PUT returns the slot after the convert (or on the reject path)
    if (ioctl(c.data_ep_fp, RFNM_IOCTL_BASE_DATA | RFNM_LOCAL_RX_GET, &c.local_rx_idx) < 0) {
        // benign: the sibling worker won the race for the packet poll() saw
        return rx_pass_result::skip;
    }
    c.local_rx_have = 1;
    c.lrxbuf = &local_rx_pool[c.local_rx_idx].p;
    return rx_pass_result::deliver;
}

void device::impl::local_rx_put(worker_ctx &c) {
    if (c.local_rx_have) {
        ioctl(c.data_ep_fp, RFNM_IOCTL_BASE_DATA | RFNM_LOCAL_RX_PUT, (unsigned long)c.local_rx_idx);
        c.local_rx_have = 0;
    }
}

bool device::impl::local_tx_prepare(worker_ctx &c, struct tx_buf *buf, uint32_t tx_elems, uint32_t &tx_wire_len) {
    // zero-copy: claim a pool slot BEFORE building and assemble the packet in
    // place (same SoC: cs16 payload, nobody packs/unpacks); SUB below hands the
    // filled index to the kernel, so the packet never exists anywhere else
    struct pollfd pfd;
    pfd.fd = c.data_ep_fp;
    pfd.events = POLLOUT;

    int ret = poll(&pfd, 1, /* timeout_ms= */ 5);
    if (ret <= 0 || !(pfd.revents & POLLOUT)) {
        if (ret < 0) {
            perror("poll");
        } else if (ret == 0) {
            spdlog::error("tx pool timeout");
        }
        std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
        tx_s.in.push(buf);
        reorder_tx_queue_nolock(tx_s);
        return false;
    }
    if (ioctl(c.data_ep_fp, RFNM_IOCTL_BASE_DATA | RFNM_LOCAL_TX_ACQ, &c.local_tx_idx) < 0) {
        // benign: the sibling worker claimed the slot poll() saw
        std::lock_guard<std::mutex> lockGuard(tx_s.in_mutex);
        tx_s.in.push(buf);
        reorder_tx_queue_nolock(tx_s);
        return false;
    }
    c.ltxbuf = &local_tx_pool[c.local_tx_idx].p;
    memcpy((uint8_t*)c.ltxbuf + RFNM_USB_TX_PACKET_HEAD_SIZE, buf->buf, (size_t)tx_elems * 4);
    c.ltxbuf->fmt = RFNM_PACKET_FMT_CS16;
    tx_wire_len = RFNM_USB_TX_PACKET_HEAD_SIZE + ((tx_elems * 3) / LA_TX_BASE_BUFSIZE_12) * LA_TX_BASE_BUFSIZE;
    return true;
}

void device::impl::local_tx_send(worker_ctx &c) {
    // the packet was built in the claimed pool slot - hand over the index
    if (ioctl(c.data_ep_fp, RFNM_IOCTL_BASE_DATA | RFNM_LOCAL_TX_SUB, (unsigned long)c.local_tx_idx) < 0) {
        spdlog::error("TX queue error");
    }
    else {
        if (debug_knob().rx) {
            static std::atomic<int> dbg_txs{0};
            int n = ++dbg_txs;
            if (n <= 3 || n % 50 == 0) {
                spdlog::info("TXSEND n {} cc {} flags {:x} ptmr {}", n,
                    (uint64_t)c.ltxbuf->usb_cc, (uint32_t)c.ltxbuf->tx_flags, (uint32_t)c.ltxbuf->phytimer);
            }
        }
    }
}

#endif // BUILD_RFNM_LOCAL_TRANSPORT

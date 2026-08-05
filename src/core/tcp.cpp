// TCP transport: control + data sockets over the private blocking-socket layer
// (net_socket.h), UDP broadcast discovery, and the worker RX/TX passes.

#include <cstring>
#include <stdexcept>

#include <spdlog/spdlog.h>

#include "impl.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>	// inet_ntop/inet_pton/INET_ADDRSTRLEN live here on Windows
#include <iphlpapi.h>
#pragma comment(lib, "Iphlpapi.lib")
#pragma comment(lib, "Ws2_32.lib")
#else
#include <ifaddrs.h>
#include <net/if.h>
#include <arpa/inet.h>
#endif

using namespace rfnm;

bool device::impl::tcp_open(const std::string &address) {
    try {
        // Reuse discovery to resolve the device address: it probes every host interface
        // and skips malformed or protocol-mismatched replies instead of failing on the
        // first bad one
        std::vector<struct dev_info> remote = device::find(TRANSPORT_TCP, address);
        if (remote.empty()) {
            return false;
        }
        rfnm_eth_transport_ip_addr = remote[0].address;

        // Connect to the TCP control port
        rfnm_ctrl_socket_tcp = net::tcp_connect(rfnm_eth_transport_ip_addr, RFNM_TCP_CTRL_PORT, true);
        // a silently-dead peer must fail control transfers, not hang them forever
        // under the ctrl mutex (which would freeze the whole control API on board
        // power loss). 3 s >> the worst observed ctrl RTT tail.
        net::sock_set_rcvtimeo(rfnm_ctrl_socket_tcp, 3000);
        net::sock_set_keepalive(rfnm_ctrl_socket_tcp);
        if (!net::sock_valid(rfnm_ctrl_socket_tcp)) {
            spdlog::error("TCP connection failed: {}", net::last_error_str());
            goto exit_eth;
        }
        spdlog::info("Connected TCP control to {}:{}", rfnm_eth_transport_ip_addr, RFNM_TCP_CTRL_PORT);

        transport_status.theoretical_mbps = 1000;
        transport_status.transport = TRANSPORT_TCP;

        {
            uint8_t dummy_buf[10];
            if (ctrl_transfer(RFNM_GET_SM_RESET, 1, dummy_buf, 50) != RFNM_API_OK) {
                spdlog::error("Failed to reset state machine");
                goto exit_eth;
            }
        }
        spdlog::info("State machine reset via TCP control channel");
        reset_session();

        if (get(REQ_ALL)) {
            goto exit_eth;
        }

        // Connect the data socket after the state machine reset (which invalidates data
        // clients), and here rather than in a worker thread: a worker blocked in connect()
        // can't observe a shutdown request, which would turn a device close during a
        // stalled connect into a use-after-free
        tcp_data_socket = net::tcp_connect(rfnm_eth_transport_ip_addr, RFNM_TCP_DATA_PORT, true);
        net::sock_set_keepalive(tcp_data_socket);
        if (!net::sock_valid(tcp_data_socket)) {
            spdlog::error("TCP connection failed: {}", net::last_error_str());
            goto exit_eth;
        }

        // Deliberately NOT setting SO_RCVBUF/SO_SNDBUF: an explicit value is clamped to
        // net.core.rmem_max (~208 KB on stock hosts) and locks the buffer there, pinning
        // the TCP receive window at ~256 KB - which caps streaming at ~1.6 Gbps (sender
        // 97.7% rwnd-limited). Kernel autotuning is exempt from that clamp and grows to
        // tcp_rmem max.

#if defined(__linux__) || defined(__APPLE__)
        // surface the negotiated MSS: jumbo frames decide whether the data path tops out
        // near ~77 or ~90 Msps on a 2.5G link, and a silently small MSS is otherwise
        // invisible to the user
        {
            int mss = net::tcp_get_mss(tcp_data_socket);
            if (mss > 0) {
                if (mss < 2000) {
                    spdlog::info("data path MSS {} (jumbo frames inactive): TCP streaming caps near ~77 Msps; "
                                 "set the host NIC MTU to 8000 on a jumbo-capable path for rates up to ~90 Msps", mss);
                } else {
                    spdlog::info("data path MSS {} (jumbo frames active)", mss);
                }
            }
        }
#endif

        tcp_data_connected = true;

        return true;
    }
    catch (std::exception& e) {
        spdlog::error("TCP connection failed: {}", e.what());
    }
exit_eth:
    net::sock_close(tcp_data_socket);
    net::sock_close(rfnm_ctrl_socket_tcp);
    return false;
}

void device::impl::tcp_close() {
    tcp_data_connected = false;
    // sock_close also resets the handle to invalid
    net::sock_close(tcp_data_socket);
    net::sock_close(rfnm_ctrl_socket_tcp);
}

rfnm_api_failcode device::impl::tcp_ctrl_transfer(enum rfnm_control_ep type, uint32_t size, uint8_t* buf, uint32_t timeout_ms) {
    // Serialize the whole request/response transaction on the shared control socket:
    // the worker status poll and main-thread get/apply/set interleave at transaction
    // granularity instead of corrupting each other's responses.
    std::lock_guard<std::mutex> ctrl_lock(rfnm_ctrl_socket_tcp_mutex);

    // once the control channel is declared dead, every further call fails fast
    // instead of re-eating a fresh receive timeout under the mutex. Session-terminal
    // by contract (reopen recovers), like RX_PIPE_DEAD.
    if (tcp_ctrl_dead) {
        return RFNM_API_USB_FAIL;
    }
    // honor the caller's budget where it's meaningful: the flat 3 s socket
    // default only bounds a DEAD peer; a live transaction answers in ms.
    net::sock_set_rcvtimeo(rfnm_ctrl_socket_tcp, timeout_ms < 500 ? 500 : (timeout_ms > 3000 ? 3000 : timeout_ms));

    {
        struct rfnm_tcp_ctrl_header header;
        header.cmd = type & 0xff;
        std::vector<uint8_t> message;

        switch (type) {
        case RFNM_SET_TX_CH_LIST:
        case RFNM_SET_RX_CH_LIST:
        case RFNM_SET_SAMP_RATE:
        case RFNM_SET_TDD:
        case RFNM_SET_TXN:
            /* SET commands: send header + data as a single message (atomicity) */
            header.size = size;

            message.resize(sizeof(header) + size);
            memcpy(message.data(), &header, sizeof(header));
            memcpy(message.data() + sizeof(header), buf, size);

            if (!net::write_exact(rfnm_ctrl_socket_tcp, message.data(), message.size())) {
                spdlog::error("TCP control transfer error: {} - latching control channel DEAD (reopen the device to recover)", net::last_error_str());
                tcp_ctrl_dead = true;
                return RFNM_API_USB_FAIL;
            }

            /* SET commands don't get responses in this protocol */
            return RFNM_API_OK;

        case RFNM_GET_DEV_HWINFO:
        case RFNM_GET_TX_CH_LIST:
        case RFNM_GET_RX_CH_LIST:
        case RFNM_GET_SET_RESULT:
        case RFNM_GET_DEV_STATUS:
        case RFNM_GET_SM_RESET:
        case RFNM_GET_LOCAL_MEMINFO:
            /* GET commands: send header only, receive header + data */
            header.size = 0;

            if (!net::write_exact(rfnm_ctrl_socket_tcp, &header, sizeof(header))) {
                spdlog::error("TCP control transfer error: {} - latching control channel DEAD (reopen the device to recover)", net::last_error_str());
                tcp_ctrl_dead = true;
                return RFNM_API_USB_FAIL;
            }

            /* Read response header */
            struct rfnm_tcp_ctrl_header resp_header;
            if (!net::read_exact(rfnm_ctrl_socket_tcp, &resp_header, sizeof(resp_header))) {
                spdlog::error("TCP control transfer error: {} - latching control channel DEAD (reopen the device to recover)", net::last_error_str());
                tcp_ctrl_dead = true;
                return RFNM_API_USB_FAIL;
            }

            if (resp_header.cmd != (type & 0xff)) {
                spdlog::error("TCP control response mismatch: got {}, expected {} - stream desynced, latching control channel DEAD",
                    resp_header.cmd, type & 0xff);
                tcp_ctrl_dead = true;
                return RFNM_API_USB_FAIL;
            }

            if (resp_header.size != size) {
                spdlog::error("TCP control size mismatch: got {}, expected {}", static_cast<unsigned>(resp_header.size), size);
                return RFNM_API_USB_FAIL;
            }

            /* Read response data */
            if (!net::read_exact(rfnm_ctrl_socket_tcp, buf, size)) {
                spdlog::error("TCP control transfer error: {}", net::last_error_str());
                return RFNM_API_USB_FAIL;
            }

            return RFNM_API_OK;

        default:
            spdlog::error("Unknown control transfer type: {}", (int)type);
            return RFNM_API_NOT_SUPPORTED;
        }
    }
}

rx_pass_result device::impl::tcp_rx_pass(worker_ctx &c) {
    bool tcp_rx_ok = false;
    {
        std::lock_guard<std::mutex> lock(tcp_data_rx_mutex);
        // v2 framing: head first, then exactly the payload the head declares -
        // elem_cnt elements of the head's fmt (PACKED12 3 B/elem, CS16 4 B/elem).
        // The RX wire mirrors the TX direction's variable framing: fixed full-size
        // records would floor RX latency at the 80-slot fill (333 us at 61.44M) and
        // make the rx_ship_slots / partial-flush levers TCP-dead. CS16 records
        // arrive while a LOCAL session owns the radio format (defect #96: the old
        // board dropped those wire-side, silently starving every remote reader);
        // c.lrxbuf is RFNM_LOCAL_RX_PACKET_SIZE so a full CS16 record fits, and the
        // deliver path branches per packet. A v1 board desyncs loudly on the magic
        // check below.
        tcp_rx_ok = net::read_exact(tcp_data_socket, (uint8_t*)c.lrxbuf, RFNM_USB_RX_PACKET_HEAD_SIZE);
        if (tcp_rx_ok) {
            if (c.lrxbuf->magic != 0x7ab8bd6f || c.lrxbuf->elem_cnt == 0 ||
                    c.lrxbuf->elem_cnt > RFNM_USB_RX_PACKET_ELEM_CNT ||
                    (c.lrxbuf->fmt != RFNM_PACKET_FMT_PACKED12 && c.lrxbuf->fmt != RFNM_PACKET_FMT_CS16)) {
                // a reliable byte stream only desyncs on a protocol bug (or a
                // v1 sender) - poison the socket so the session fails loudly
                // instead of spraying garbage packets upward
                spdlog::error("TCP RX framing lost (magic {:x} fmt {} elem_cnt {}) - v1 board or protocol bug, dropping connection",
                    (uint32_t)c.lrxbuf->magic, (uint32_t)c.lrxbuf->fmt, (uint32_t)c.lrxbuf->elem_cnt);
                net::sock_shutdown(tcp_data_socket);
                tcp_rx_ok = false;
            } else {
                tcp_rx_ok = net::read_exact(tcp_data_socket, (uint8_t*)c.lrxbuf->buf,
                    (size_t)c.lrxbuf->elem_cnt * (c.lrxbuf->fmt == RFNM_PACKET_FMT_CS16 ? 4 : 3));
            }
        }

        if (tcp_rx_ok) {
            if (rx_s.usb_cc[c.lrxbuf->adc_id] != UINT64_MAX && c.lrxbuf->usb_cc < rx_s.usb_cc[c.lrxbuf->adc_id]) {
                return rx_pass_result::skip;
            }

            {
                std::lock_guard<std::mutex> lockGuard(rx_s.benchmark_mutex);
                rx_s.usb_cc_benchmark[thread_data[c.index].ep_id % 4]++;
            }
        }
    }
    if (!tcp_rx_ok) {
        // read_exact on the blocking data socket only fails on EOF/hard error (or the
        // framing poison above, which already shut it down): the stream is over. Say
        // it once and latch so dqbuf callers get RX_PIPE_DEAD instead of a silent
        // forever-retry.
        if (!ses.rx_pipe_dead_latch.exchange(true)) {
            spdlog::error("TCP data stream ended ({}) - latching RX_PIPE_DEAD, reopen the device to recover", net::last_error_str());
            rx_s.cv.notify_all();
        }
        return rx_pass_result::skip;
    }
    return rx_pass_result::deliver;
}

bool device::impl::tcp_tx_send(worker_ctx &c, uint32_t tx_wire_len) {
    bool tcp_tx_ok;
    {
        std::lock_guard<std::mutex> lock(tcp_data_tx_mutex);
        tcp_tx_ok = net::write_exact(tcp_data_socket, (uint8_t*)c.ltxbuf, tx_wire_len);
    }
    if (!tcp_tx_ok) {
        spdlog::error("TCP TX error: {}", net::last_error_str());
    }
    return tcp_tx_ok;
}

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

static std::vector<std::string> get_broadcast_addresses() {
    std::vector<std::string> broadcasts;
#ifdef _WIN32
    // Initialize Winsock (if not already done by your application).
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        spdlog::error("WSAStartup failed");
        return broadcasts;
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
        return broadcasts;
    }

    // Iterate over the adapters.
    for (PIP_ADAPTER_ADDRESSES adapter = pAddresses; adapter; adapter = adapter->Next) {
        // Skip if not up or if it's a loopback.
        if (adapter->OperStatus != IfOperStatusUp)
            continue;
        if (adapter->IfType == IF_TYPE_SOFTWARE_LOOPBACK)
            continue;

        // Skip virtual adapters.
        if (adapter->Description && wcsstr(adapter->Description, L"Virtual") != nullptr) {
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
                    broadcasts.push_back(compute_broadcast_address(ipStr, maskStr));
                }
                catch (const std::exception& ex) {
                    spdlog::error("Error computing broadcast address: {}", ex.what());
                }
            }
        }
    }
    WSACleanup();
    return broadcasts;
#else
    // Linux / macOS implementation using getifaddrs.
    struct ifaddrs* ifaddr = nullptr, * ifa = nullptr;

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return broadcasts;
    }

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr)
            continue;
        if (ifa->ifa_addr->sa_family == AF_INET &&
            (ifa->ifa_flags & IFF_UP) &&
            !(ifa->ifa_flags & IFF_LOOPBACK))
        {
            // Skip names that suggest virtual or non-physical interfaces.
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
                broadcasts.push_back(compute_broadcast_address(ipStr, maskStr));
            }
            catch (const std::exception& ex) {
                spdlog::error("Error computing broadcast address: {}", ex.what());
            }
        }
    }

    freeifaddrs(ifaddr);
    return broadcasts;
#endif
}

void rfnm::tcp_find_into(std::vector<struct dev_info> &found, const std::string &address) {
    std::vector<std::string> probe_addrs;
    if (address.length()) {
        probe_addrs.push_back(address);
    }
    else {
        probe_addrs = get_broadcast_addresses();
    }

    if (!probe_addrs.empty()) {
        // non-blocking IPv4 socket, SO_BROADCAST only for undirected discovery
        uint64_t sock = net::udp_open_v4(!address.length());
        if (!net::sock_valid(sock)) {
            spdlog::error("UDP problem: {}", net::last_error_str());
            return;
        }

        uint8_t message[1];
        message[0] = ((int)RFNM_GET_DEV_HWINFO) & 0xff;

        for (auto& probe_addr : probe_addrs) {
            if (!net::udp_send_to(sock, message, sizeof(message), probe_addr, RFNM_UDP_CTRL_PORT)) {
                // one unroutable interface must not abort discovery on the others
                spdlog::info("Discovery probe to {} failed: {}", probe_addr, net::last_error_str());
            }
        }

        // A broadcast can be answered by any number of devices, so keep collecting
        // replies until the discovery window closes
        char reply[1152];
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
        while (std::chrono::steady_clock::now() < deadline) {
            std::string sender_addr;
            bool would_block = false;
            int reply_length = net::udp_recv_from(sock, reply, sizeof(reply), sender_addr, would_block);

            if (reply_length < 0 && would_block) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            if (reply_length < 0) {
                spdlog::error("UDP receive failed: {}", net::last_error_str());
                break;
            }

            if ((size_t)reply_length != sizeof(rfnm_dev_hwinfo) + 1) {
                spdlog::error("Reply too short ({} bytes)", reply_length);
                continue;
            }
            if (reply[0] != (RFNM_GET_DEV_HWINFO & 0xff)) {
                spdlog::error("UDP control transfer conflict got {} should have been RFNM_GET_DEV_HWINFO", (uint8_t)reply[0]);
                continue;
            }

            rfnm_dev_hwinfo r_hwinfo;
            memcpy(&r_hwinfo, &reply[1], sizeof(rfnm_dev_hwinfo));
            if (r_hwinfo.protocol_version != RFNM_PROTOCOL_VERSION) {
                uint32_t mv = r_hwinfo.protocol_version;
                spdlog::error("Protocol version mismatch: received {}, expected {}", mv, RFNM_PROTOCOL_VERSION);
                continue;
            }

            if (sn_already_found(found, TRANSPORT_TCP, r_hwinfo.motherboard.serial_number)) {
                spdlog::info("Skip Network device (duplicate SN): {}", (char*)r_hwinfo.motherboard.serial_number);
                continue;
            }

            spdlog::info("Add Network device: {}", (char*)r_hwinfo.motherboard.serial_number);
            // For a directed probe keep the address the caller reached the device at: the
            // reply's source address is whatever the device kernel picked for the route
            // (e.g. a DHCP lease next to the statically pinned address) and may not be stable
            found.push_back({ TRANSPORT_TCP, address.length() ? address : sender_addr, r_hwinfo });

            // A directed probe can only ever get one reply
            if (address.length()) {
                break;
            }
        }

        net::sock_close(sock);
    }
}

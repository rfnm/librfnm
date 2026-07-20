#include "net_socket.h"

#include <cstring>
#include <cstdio>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#ifdef _MSC_VER
#pragma comment(lib, "Ws2_32.lib")
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;	// MSVC has no POSIX ssize_t (MinGW provides its own)
#endif
typedef SOCKET native_sock_t;
typedef int native_socklen_t;
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
typedef int native_sock_t;
typedef socklen_t native_socklen_t;
#endif

// asio added MSG_NOSIGNAL on Linux sends (and SO_NOSIGPIPE on Apple sockets) so a dead peer
// surfaces as an error instead of SIGPIPE killing the process - keep that behavior
#if defined(__linux__)
#define NET_SEND_FLAGS MSG_NOSIGNAL
#else
#define NET_SEND_FLAGS 0
#endif

using namespace rfnm;

// 0 = no error recorded; NET_ERR_EOF = orderly peer close; NET_ERR_BADADDR = unparsable IPv4
// string; anything else is a raw errno / WSAGetLastError value
#define NET_ERR_EOF (-1)
#define NET_ERR_BADADDR (-2)
static thread_local int t_net_last_err = 0;

static void net_set_last_err() {
#ifdef _WIN32
    t_net_last_err = WSAGetLastError();
#else
    t_net_last_err = errno;
#endif
}

static native_sock_t to_native(uint64_t h) {
#ifdef _WIN32
    if (h == net::NET_SOCK_INVALID) {
        return INVALID_SOCKET;
    }
    return (native_sock_t)h;
#else
    if (h == net::NET_SOCK_INVALID) {
        return -1;
    }
    return (native_sock_t)h;
#endif
}

static uint64_t from_native(native_sock_t s) {
#ifdef _WIN32
    if (s == INVALID_SOCKET) {
        return net::NET_SOCK_INVALID;
    }
#else
    if (s < 0) {
        return net::NET_SOCK_INVALID;
    }
#endif
    return (uint64_t)s;
}

static void native_close(native_sock_t s) {
#ifdef _WIN32
    closesocket(s);
#else
    close(s);
#endif
}

#ifdef _WIN32
static bool wsa_init() {
    // winsock refcounts WSAStartup/WSACleanup internally; hold one live reference for the process
    // lifetime instead of pairing every socket (get_broadcast_addresses does its own pair on top)
    static bool ok = []() {
        WSADATA wsa;
        return WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
    }();
    return ok;
}
#endif

static bool make_sockaddr_v4(struct sockaddr_in& sa, const std::string& ipv4_addr, uint16_t port) {
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    if (inet_pton(AF_INET, ipv4_addr.c_str(), &sa.sin_addr) != 1) {
        t_net_last_err = NET_ERR_BADADDR;
        return false;
    }
    return true;
}

bool net::sock_valid(uint64_t h) {
    return h != NET_SOCK_INVALID;
}

std::string net::last_error_str() {
    if (t_net_last_err == NET_ERR_EOF) {
        return "connection closed by peer";
    }
    if (t_net_last_err == NET_ERR_BADADDR) {
        return "invalid IPv4 address";
    }
#ifdef _WIN32
    char buf[64];
    snprintf(buf, sizeof(buf), "winsock error %d", t_net_last_err);
    return std::string(buf);
#else
    return std::string(strerror(t_net_last_err)) + " (" + std::to_string(t_net_last_err) + ")";
#endif
}

uint64_t net::tcp_connect(const std::string& ipv4_addr, uint16_t port, bool nodelay) {
#ifdef _WIN32
    if (!wsa_init()) {
        net_set_last_err();
        return NET_SOCK_INVALID;
    }
#endif

    struct sockaddr_in sa;
    if (!make_sockaddr_v4(sa, ipv4_addr, port)) {
        return NET_SOCK_INVALID;
    }

    native_sock_t fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (from_native(fd) == NET_SOCK_INVALID) {
        net_set_last_err();
        return NET_SOCK_INVALID;
    }

#ifdef __APPLE__
    {
        int one = 1;
        setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
    }
#endif

    if (connect(fd, (struct sockaddr*)&sa, sizeof(sa)) != 0) {
        net_set_last_err();
        native_close(fd);
        return NET_SOCK_INVALID;
    }

    if (nodelay) {
        // best effort: a failed TCP_NODELAY only costs latency, never correctness
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
    }

    return from_native(fd);
}

bool net::sock_set_keepalive(uint64_t h) {
    // review wave 3: a silently-dead peer (board power loss) on an IDLE tcp
    // connection is otherwise only discovered at the next send/recv. Keepalive
    // probes surface it in ~11 s: idle 5 s, then 3 probes 2 s apart.
    int on = 1;
    if (setsockopt(to_native(h), SOL_SOCKET, SO_KEEPALIVE, (const char*)&on, sizeof(on)) != 0) {
        return false;
    }
#ifndef _WIN32
    int idle = 5, intvl = 2, cnt = 3;
#if defined(TCP_KEEPIDLE)
    setsockopt(to_native(h), IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
#elif defined(TCP_KEEPALIVE)
    // Darwin spells the idle-time option TCP_KEEPALIVE (TCP_KEEPIDLE is Linux-only)
    setsockopt(to_native(h), IPPROTO_TCP, TCP_KEEPALIVE, &idle, sizeof(idle));
#endif
#ifdef TCP_KEEPINTVL
    setsockopt(to_native(h), IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
#endif
#ifdef TCP_KEEPCNT
    setsockopt(to_native(h), IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#endif
#endif
    return true;
}

bool net::sock_set_rcvtimeo(uint64_t h, uint32_t ms) {
    // review F4-lite (2026-07-11): a peer that dies silently (board power loss)
    // left blocking recv() waiting forever - on the CONTROL socket that hang is
    // held under the ctrl mutex and freezes the whole control API. A receive
    // timeout converts it into a loud failure. Control-socket use only: the DATA
    // socket's blocking reads are by-design (a timeout there would read as a
    // stream failure).
#ifdef _WIN32
    DWORD tv = ms;
    return setsockopt(to_native(h), SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) == 0;
#else
    struct timeval tv;
    tv.tv_sec = ms / 1000;
    tv.tv_usec = (ms % 1000) * 1000;
    return setsockopt(to_native(h), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
#endif
}

bool net::read_exact(uint64_t h, void* buf, size_t len) {
    native_sock_t fd = to_native(h);
    uint8_t* p = (uint8_t*)buf;
    size_t got = 0;

    while (got < len) {
#ifdef _WIN32
        int r = recv(fd, (char*)p + got, (int)(len - got), 0);
        if (r == SOCKET_ERROR) {
            if (WSAGetLastError() == WSAEINTR) {
                continue;
            }
            net_set_last_err();
            return false;
        }
#else
        ssize_t r = recv(fd, p + got, len - got, 0);
        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            net_set_last_err();
            return false;
        }
#endif
        if (r == 0) {
            // orderly shutdown mid-message: same lump failure the old asio::read eof throw was
            t_net_last_err = NET_ERR_EOF;
            return false;
        }
        got += (size_t)r;
    }
    return true;
}

bool net::write_exact(uint64_t h, const void* buf, size_t len) {
    native_sock_t fd = to_native(h);
    const uint8_t* p = (const uint8_t*)buf;
    size_t sent = 0;

    while (sent < len) {
#ifdef _WIN32
        int r = send(fd, (const char*)p + sent, (int)(len - sent), NET_SEND_FLAGS);
        if (r == SOCKET_ERROR) {
            if (WSAGetLastError() == WSAEINTR) {
                continue;
            }
            net_set_last_err();
            return false;
        }
#else
        ssize_t r = send(fd, p + sent, len - sent, NET_SEND_FLAGS);
        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            net_set_last_err();
            return false;
        }
#endif
        sent += (size_t)r;
    }
    return true;
}

int net::tcp_get_mss(uint64_t h) {
#if defined(__linux__) || defined(__APPLE__)
    int mss = 0;
    native_socklen_t mss_len = sizeof(mss);
    if (getsockopt(to_native(h), IPPROTO_TCP, TCP_MAXSEG, &mss, &mss_len) == 0 && mss > 0) {
        return mss;
    }
    return 0;
#else
    (void)h;
    return 0;
#endif
}

uint64_t net::udp_open_v4(bool broadcast) {
#ifdef _WIN32
    if (!wsa_init()) {
        net_set_last_err();
        return NET_SOCK_INVALID;
    }
#endif

    native_sock_t fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (from_native(fd) == NET_SOCK_INVALID) {
        net_set_last_err();
        return NET_SOCK_INVALID;
    }

    // non-blocking: discovery polls for replies inside its deadline window
#ifdef _WIN32
    u_long nb = 1;
    if (ioctlsocket(fd, FIONBIO, &nb) != 0) {
        net_set_last_err();
        native_close(fd);
        return NET_SOCK_INVALID;
    }
#else
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0 || fcntl(fd, F_SETFL, fl | O_NONBLOCK) < 0) {
        net_set_last_err();
        native_close(fd);
        return NET_SOCK_INVALID;
    }
#endif

    if (broadcast) {
        int one = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, (const char*)&one, sizeof(one)) != 0) {
            net_set_last_err();
            native_close(fd);
            return NET_SOCK_INVALID;
        }
    }

#ifdef __APPLE__
    {
        int one = 1;
        setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
    }
#endif

    return from_native(fd);
}

bool net::udp_send_to(uint64_t h, const void* buf, size_t len, const std::string& ipv4_addr, uint16_t port) {
    struct sockaddr_in sa;
    if (!make_sockaddr_v4(sa, ipv4_addr, port)) {
        return false;
    }

#ifdef _WIN32
    int r = sendto(to_native(h), (const char*)buf, (int)len, NET_SEND_FLAGS, (struct sockaddr*)&sa, sizeof(sa));
    if (r == SOCKET_ERROR) {
        net_set_last_err();
        return false;
    }
#else
    ssize_t r = sendto(to_native(h), buf, len, NET_SEND_FLAGS, (struct sockaddr*)&sa, sizeof(sa));
    if (r < 0) {
        net_set_last_err();
        return false;
    }
#endif
    // datagrams go out whole or not at all, but keep the check honest
    return (size_t)r == len;
}

int net::udp_recv_from(uint64_t h, void* buf, size_t len, std::string& sender_addr, bool& would_block) {
    would_block = false;

    struct sockaddr_in sa;
    native_socklen_t sl = sizeof(sa);
    memset(&sa, 0, sizeof(sa));

#ifdef _WIN32
    int r;
    for (;;) {
        r = recvfrom(to_native(h), (char*)buf, (int)len, 0, (struct sockaddr*)&sa, &sl);
        if (r == SOCKET_ERROR && WSAGetLastError() == WSAEINTR) {
            continue;
        }
        break;
    }
    if (r == SOCKET_ERROR) {
        if (WSAGetLastError() == WSAEWOULDBLOCK) {
            would_block = true;
            return -1;
        }
        net_set_last_err();
        return -1;
    }
#else
    ssize_t r;
    for (;;) {
        r = recvfrom(to_native(h), buf, len, 0, (struct sockaddr*)&sa, &sl);
        if (r < 0 && errno == EINTR) {
            continue;
        }
        break;
    }
    if (r < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            would_block = true;
            return -1;
        }
        net_set_last_err();
        return -1;
    }
#endif

    char abuf[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &sa.sin_addr, abuf, sizeof(abuf))) {
        sender_addr = abuf;
    } else {
        sender_addr = "";
    }
    return (int)r;
}

void net::sock_shutdown(uint64_t h) {
    if (!sock_valid(h)) {
        return;
    }
#ifdef _WIN32
    shutdown(to_native(h), SD_BOTH);
#else
    shutdown(to_native(h), SHUT_RDWR);
#endif
}

void net::sock_close(uint64_t& h) {
    if (!sock_valid(h)) {
        return;
    }
    native_close(to_native(h));
    h = NET_SOCK_INVALID;
}

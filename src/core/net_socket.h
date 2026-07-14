#pragma once

// Minimal private blocking-socket layer replacing asio. librfnm never ran an event loop (the old
// io_context objects existed but run() was never called), so all it ever needed was portable
// blocking TCP with exact-length IO plus a non-blocking UDP socket for discovery. This header is
// NOT installed: the public headers must stay free of asio and platform socket headers alike.

#include <cstdint>
#include <cstddef>
#include <string>

namespace rfnm {
    namespace net {

        // Native socket handles travel as uint64_t (int fd on POSIX, SOCKET on Windows) so the
        // public device.h can store them without platform includes. NET_SOCK_INVALID = not open.
        static const uint64_t NET_SOCK_INVALID = UINT64_MAX;

        bool sock_valid(uint64_t h);

        // last failed call's error as text for log messages (thread-local, valid until the next
        // net:: call on the same thread fails)
        std::string last_error_str();

        // blocking TCP connect to an IPv4 dotted-quad address; returns NET_SOCK_INVALID on failure
        uint64_t tcp_connect(const std::string& ipv4_addr, uint16_t port, bool nodelay);

        // receive timeout in ms. Control sockets only - a timeout on the data socket
        // reads as a stream failure to the reader loop (review F4-lite 2026-07-11).
        bool sock_set_rcvtimeo(uint64_t h, uint32_t ms);
        bool sock_set_keepalive(uint64_t h);

        // exact-length blocking IO: loop over partial transfers and retry EINTR. Return true only
        // once the full length moved; false on any error or peer close, matching how the call
        // sites treated a throwing asio::read/asio::write (one lump failure, no partial results).
        bool read_exact(uint64_t h, void* buf, size_t len);
        bool write_exact(uint64_t h, const void* buf, size_t len);

        // negotiated TCP MSS via getsockopt(TCP_MAXSEG), 0 when unavailable on this platform
        int tcp_get_mss(uint64_t h);

        // non-blocking IPv4 UDP socket for discovery, optionally SO_BROADCAST; NET_SOCK_INVALID on failure
        uint64_t udp_open_v4(bool broadcast);

        // true if the whole datagram was handed to the stack
        bool udp_send_to(uint64_t h, const void* buf, size_t len, const std::string& ipv4_addr, uint16_t port);

        // >= 0: datagram length, sender_addr filled with the sender's IPv4 address.
        // -1 with would_block set: nothing pending yet (poll again).
        // -1 with would_block clear: real error, see last_error_str().
        int udp_recv_from(uint64_t h, void* buf, size_t len, std::string& sender_addr, bool& would_block);

        // shutdown() interrupts another thread blocked in recv on the same socket (close() alone
        // does not) - the device destructor relies on exactly this ordering to unstick workers
        void sock_shutdown(uint64_t h);

        // close and reset the handle to NET_SOCK_INVALID; safe to call on an invalid handle
        void sock_close(uint64_t& h);
    }
}

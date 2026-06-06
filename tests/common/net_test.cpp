#include "common/net.hpp"
#include "common/mavlink_helpers.hpp"

#include <arpa/inet.h>
#include <iostream>

namespace {

bool check(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "net_test failure: " << message << "\n";
        return false;
    }
    return true;
}

} // namespace

int main()
{
    in_addr address{};

    if (!check(resolve_ipv4("127.0.0.1", address),
               "failed to resolve numeric IPv4 address")) {
        return 1;
    }
    if (!check(address.s_addr == htonl(INADDR_LOOPBACK),
               "numeric IPv4 address resolved incorrectly")) {
        return 1;
    }

    address = {};
    if (!check(resolve_ipv4("localhost", address),
               "failed to resolve localhost")) {
        return 1;
    }
    if (!check((ntohl(address.s_addr) & 0xff000000U) == 0x7f000000U,
               "localhost did not resolve within 127.0.0.0/8")) {
        return 1;
    }

    address = {};
    if (!check(!resolve_ipv4("host.invalid.invalid.", address),
               "reserved invalid hostname unexpectedly resolved")) {
        return 1;
    }

    if constexpr (MAVLINK_COMM_NUM_BUFFERS > 1) {
        for (uint8_t id = 1; id < 16; ++id) {
            if (!check(mavhelper::channel_for(id) != MAVLINK_COMM_0,
                       "drone MAVLink channel collided with GCS receive channel")) {
                return 1;
            }
        }
    }

    return 0;
}

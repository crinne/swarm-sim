#pragma once

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>

#include <string>

inline bool resolve_ipv4(const std::string& host, in_addr& address)
{
    if (inet_pton(AF_INET, host.c_str(), &address) == 1) {
        return true;
    }

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    addrinfo* results = nullptr;
    if (getaddrinfo(host.c_str(), nullptr, &hints, &results) != 0) {
        return false;
    }

    address = reinterpret_cast<sockaddr_in*>(results->ai_addr)->sin_addr;
    freeaddrinfo(results);
    return true;
}

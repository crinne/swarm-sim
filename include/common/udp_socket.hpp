#pragma once

#include <cerrno>
#include <cstring>
#include <string>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

class UdpSocket {
public:
    UdpSocket() = default;
    explicit UdpSocket(int fd) : fd_(fd) {}

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    UdpSocket(UdpSocket&& other) noexcept
        : fd_(other.fd_)
    {
        other.fd_ = -1;
    }

    UdpSocket& operator=(UdpSocket&& other) noexcept
    {
        if (this != &other) {
            close();
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

    ~UdpSocket()
    {
        close();
    }

    bool open()
    {
        close();
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        return fd_ >= 0;
    }

    bool bind(const sockaddr_in& address)
    {
        return fd_ >= 0 &&
               ::bind(fd_, reinterpret_cast<const sockaddr*>(&address),
                      sizeof(address)) == 0;
    }

    ssize_t recv_from(void* buffer, size_t length, int flags,
                      sockaddr_in& from)
    {
        socklen_t from_len = sizeof(from);
        return ::recvfrom(fd_, buffer, length, flags,
                          reinterpret_cast<sockaddr*>(&from), &from_len);
    }

    ssize_t send_to(const void* buffer, size_t length,
                    const sockaddr_in& to) const
    {
        return ::sendto(fd_, buffer, length, 0,
                        reinterpret_cast<const sockaddr*>(&to), sizeof(to));
    }

    int fd() const
    {
        return fd_;
    }

    void close()
    {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    static std::string last_error()
    {
        return std::strerror(errno);
    }

private:
    int fd_ = -1;
};

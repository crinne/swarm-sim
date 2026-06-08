#include "proxy/proxy.hpp"
#include "common/udp_socket.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <map>
#include <mutex>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace std::chrono;

static volatile bool running = true;
void handle_signal(int) { running = false; }

// known endpoints
static std::mutex             endpoints_mutex;
static sockaddr_in            gcs_addr{};
static bool                   gcs_known = false;
static std::map<uint32_t, sockaddr_in> drone_addrs; // key: sin_addr+sin_port

int main(int argc, char* argv[]) {
    uint16_t drone_port = 14550;
    uint16_t gcs_port   = 14560;
    float    loss_pct   = 0.0f;

    for (int i = 1; i < argc - 1; i++) {
        std::string arg = argv[i];
        if (arg == "--drone-port") drone_port = std::stoi(argv[i+1]);
        if (arg == "--gcs-port")   gcs_port   = std::stoi(argv[i+1]);
        if (arg == "--loss")       loss_pct   = std::stof(argv[i+1]);
    }

    std::signal(SIGINT,  handle_signal);
    std::signal(SIGTERM, handle_signal);

    Proxy proxy(drone_port, gcs_port, loss_pct);

    UdpSocket drone_sock;
    if (!drone_sock.open()) {
        std::cerr << "event=proxy_error msg=drone_socket_failed"
                  << " error=\"" << UdpSocket::last_error() << "\"\n";
        return 1;
    }
    sockaddr_in drone_bind{};
    drone_bind.sin_family      = AF_INET;
    drone_bind.sin_port        = htons(drone_port);
    drone_bind.sin_addr.s_addr = INADDR_ANY;
    if (!drone_sock.bind(drone_bind)) {
        std::cerr << "event=proxy_error msg=drone_bind_failed port="
                  << drone_port
                  << " error=\"" << UdpSocket::last_error() << "\"\n";
        return 1;
    }

    UdpSocket gcs_sock;
    if (!gcs_sock.open()) {
        std::cerr << "event=proxy_error msg=gcs_socket_failed"
                  << " error=\"" << UdpSocket::last_error() << "\"\n";
        return 1;
    }
    sockaddr_in gcs_bind{};
    gcs_bind.sin_family      = AF_INET;
    gcs_bind.sin_port        = htons(gcs_port);
    gcs_bind.sin_addr.s_addr = INADDR_ANY;
    if (!gcs_sock.bind(gcs_bind)) {
        std::cerr << "event=proxy_error msg=gcs_bind_failed port="
                  << gcs_port
                  << " error=\"" << UdpSocket::last_error() << "\"\n";
        return 1;
    }

    std::cout << "event=proxy_start"
              << " drone_port=" << drone_port
              << " gcs_port="   << gcs_port
              << " loss="       << loss_pct << "\n";

    // thread: drone → gcs
    std::jthread drone_rx([&](std::stop_token st) {
        uint8_t buf[1024];
        while (!st.stop_requested() && running) {
            sockaddr_in from{};
            ssize_t     len = drone_sock.recv_from(buf, sizeof(buf),
                                                   MSG_DONTWAIT, from);
            if (len <= 0) {
                std::this_thread::sleep_for(milliseconds(1));
                continue;
            }

            // register drone
            uint32_t key = from.sin_addr.s_addr ^ from.sin_port;
            {
                std::lock_guard lk(endpoints_mutex);
                drone_addrs[key] = from;
            }

            if (proxy.should_drop()) {
                std::cout << "event=packet_drop dir=drone_to_gcs\n";
                continue;
            }

            if (!proxy.within_budget(len)) {
                std::cout << "event=packet_drop dir=drone_to_gcs "
                             "reason=budget\n";
                continue;
            }

            std::lock_guard lk(endpoints_mutex);
            if (gcs_known) {
                if (gcs_sock.send_to(buf, len, gcs_addr) < 0) {
                    std::cerr << "event=proxy_error msg=send_to_gcs_failed"
                              << " error=\"" << UdpSocket::last_error()
                              << "\"\n";
                }
            }
        }
    });

    // thread: gcs → drones
    std::jthread gcs_rx([&](std::stop_token st) {
        uint8_t buf[1024];
        while (!st.stop_requested() && running) {
            sockaddr_in from{};
            ssize_t     len = gcs_sock.recv_from(buf, sizeof(buf),
                                                 MSG_DONTWAIT, from);
            if (len <= 0) {
                std::this_thread::sleep_for(milliseconds(1));
                continue;
            }

            // register gcs
            {
                std::lock_guard lk(endpoints_mutex);
                gcs_addr  = from;
                gcs_known = true;
            }
            if (len == 1 && buf[0] == 0) {
                continue;
            }

            if (proxy.should_drop()) {
                std::cout << "event=packet_drop dir=gcs_to_drone\n";
                continue;
            }

            // forward to all known drones
            std::lock_guard lk(endpoints_mutex);
            for (auto& [key, addr] : drone_addrs) {
                if (drone_sock.send_to(buf, len, addr) < 0) {
                    std::cerr << "event=proxy_error msg=send_to_drone_failed"
                              << " error=\"" << UdpSocket::last_error()
                              << "\"\n";
                }
            }
        }
    });

    // budget reset every second
    while (running) {
        std::this_thread::sleep_for(seconds(1));
        proxy.reset_budget();
    }

    std::cout << "event=proxy_stop\n";
    return 0;
}

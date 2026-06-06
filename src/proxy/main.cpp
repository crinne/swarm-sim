#include "proxy/proxy.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <map>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

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

    // drone-facing socket
    int drone_sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in drone_bind{};
    drone_bind.sin_family      = AF_INET;
    drone_bind.sin_port        = htons(drone_port);
    drone_bind.sin_addr.s_addr = INADDR_ANY;
    bind(drone_sock, (sockaddr*)&drone_bind, sizeof(drone_bind));

    // gcs-facing socket
    int gcs_sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in gcs_bind{};
    gcs_bind.sin_family      = AF_INET;
    gcs_bind.sin_port        = htons(gcs_port);
    gcs_bind.sin_addr.s_addr = INADDR_ANY;
    bind(gcs_sock, (sockaddr*)&gcs_bind, sizeof(gcs_bind));

    std::cout << "event=proxy_start"
              << " drone_port=" << drone_port
              << " gcs_port="   << gcs_port
              << " loss="       << loss_pct << "\n";

    // thread: drone → gcs
    std::jthread drone_rx([&](std::stop_token st) {
        uint8_t buf[1024];
        while (!st.stop_requested() && running) {
            sockaddr_in from{};
            socklen_t   from_len = sizeof(from);
            ssize_t     len = recvfrom(drone_sock, buf, sizeof(buf),
                                       MSG_DONTWAIT,
                                       (sockaddr*)&from, &from_len);
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
                sendto(gcs_sock, buf, len, 0,
                       (sockaddr*)&gcs_addr, sizeof(gcs_addr));
            }
        }
    });

    // thread: gcs → drones
    std::jthread gcs_rx([&](std::stop_token st) {
        uint8_t buf[1024];
        while (!st.stop_requested() && running) {
            sockaddr_in from{};
            socklen_t   from_len = sizeof(from);
            ssize_t     len = recvfrom(gcs_sock, buf, sizeof(buf),
                                       MSG_DONTWAIT,
                                       (sockaddr*)&from, &from_len);
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

            if (proxy.should_drop()) {
                std::cout << "event=packet_drop dir=gcs_to_drone\n";
                continue;
            }

            if (!proxy.within_budget(len)) {
                std::cout << "event=packet_drop dir=gcs_to_drone "
                             "reason=budget\n";
                continue;
            }

            // forward to all known drones
            std::lock_guard lk(endpoints_mutex);
            for (auto& [key, addr] : drone_addrs) {
                sendto(drone_sock, buf, len, 0,
                       (sockaddr*)&addr, sizeof(addr));
            }
        }
    });

    // budget reset every second
    while (running) {
        std::this_thread::sleep_for(seconds(1));
        proxy.reset_budget();
    }

    std::cout << "event=proxy_stop\n";
    close(drone_sock);
    close(gcs_sock);
    return 0;
}
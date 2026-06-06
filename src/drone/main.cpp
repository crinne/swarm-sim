#include "common/net.hpp"
#include "common/types.hpp"
#include "common/mavlink_helpers.hpp"
#include "drone/physics.hpp"
#include "drone/mcu.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

using namespace std::chrono;

static volatile bool running = true;
void handle_signal(int) { running = false; }

int main(int argc, char* argv[]) {
    // parse args: drone --id 1 --proxy 127.0.0.1 --port 14550
    uint8_t     id         = 1;
    std::string proxy_ip   = "127.0.0.1";
    uint16_t    proxy_port = 14550;
    float       start_x    = 0.0f;
    float       start_y    = 0.0f;

    for (int i = 1; i < argc - 1; i++) {
        std::string arg = argv[i];
        if (arg == "--id")    id         = std::stoi(argv[i+1]);
        if (arg == "--proxy") proxy_ip   = argv[i+1];
        if (arg == "--port")  proxy_port = std::stoi(argv[i+1]);
        if (arg == "--x")     start_x    = std::stof(argv[i+1]);
        if (arg == "--y")     start_y    = std::stof(argv[i+1]);
    }

    std::signal(SIGINT,  handle_signal);
    std::signal(SIGTERM, handle_signal);

    // setup UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "event=drone_error msg=socket_failed id="
                  << (int)id << "\n";
        return 1;
    }

    sockaddr_in proxy_addr{};
    proxy_addr.sin_family      = AF_INET;
    proxy_addr.sin_port        = htons(proxy_port);
    if (!resolve_ipv4(proxy_ip, proxy_addr.sin_addr)) {
        std::cerr << "event=resolve_error host=" << proxy_ip << "\n";
        close(sock);
        return 1;
    }

    // bind local port so proxy can send back
    sockaddr_in local_addr{};
    local_addr.sin_family      = AF_INET;
    local_addr.sin_port        = htons(14550 + id);
    local_addr.sin_addr.s_addr = INADDR_ANY;
    bind(sock, (sockaddr*)&local_addr, sizeof(local_addr));

    // init drone
    Physics physics(id, {start_x, start_y, 10.0f});
    physics.set_orbit(20.0f + id * 15.0f, id * 2.0f); // per-id radius/angle so drones don't collide
    MCU     mcu(physics);

    std::cout << "event=drone_start id=" << (int)id
              << " proxy=" << proxy_ip
              << " port=" << proxy_port << "\n";

    // fixed step loop — sleep_until not sleep_for
    constexpr float    DT      = 0.01f; // 100Hz
    constexpr auto     TICK    = milliseconds(10);
    auto               next    = steady_clock::now() + TICK;

    uint8_t  rx_buf[1024];
    uint8_t  tx_buf[1024];

    while (running) {
        // 1. receive incoming commands (non-blocking)
        sockaddr_in from{};
        socklen_t   from_len = sizeof(from);
        ssize_t     rx_len   = recvfrom(sock, rx_buf, sizeof(rx_buf),
                                        MSG_DONTWAIT,
                                        (sockaddr*)&from, &from_len);
        if (rx_len > 0) {
            mcu.process(rx_buf, static_cast<uint16_t>(rx_len));
        }

        // 2. physics tick
        physics.step(DT);

        // 3. send telemetry
        uint16_t tx_len = mcu.send_telemetry(tx_buf, sizeof(tx_buf));
        sendto(sock, tx_buf, tx_len, 0,
               (sockaddr*)&proxy_addr, sizeof(proxy_addr));

        if (physics.finished()) {
            running = false;
        }

        // 4. fixed step — sleep until next tick
        std::this_thread::sleep_until(next);
        next += TICK;
    }

    std::cout << "event=drone_stop id=" << (int)id << "\n";
    close(sock);
    return 0;
}

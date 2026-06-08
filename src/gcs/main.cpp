#include "common/net.hpp"
#include "common/udp_socket.hpp"
#include "gcs/gcs_engine.hpp"
#include "gcs/websocket.hpp"

#include <iostream>
#include <thread>
#include <csignal>
#include <netinet/in.h>
#include <arpa/inet.h>

static volatile bool running = true;
static SseServer* g_sse = nullptr;
void handle_signal(int)
{
    running = false;
    if (g_sse) g_sse->stop();
}

int main(int argc, char *argv[])
{
    std::string proxy_ip = "127.0.0.1";
    uint16_t proxy_port = 14560;
    uint16_t http_port = 8080;
    std::string allowed_origin = "http://localhost:5173";

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--allowed-origin") {
            if (i + 1 >= argc) {
                std::cerr
                    << "event=gcs_error msg=missing_allowed_origin\n";
                return 1;
            }
            allowed_origin = argv[++i];
            continue;
        }

        if (i + 1 >= argc)
            continue;
        if (arg == "--proxy")
            proxy_ip = argv[i + 1];
        if (arg == "--port")
            proxy_port = std::stoi(argv[i + 1]);
        if (arg == "--http-port")
            http_port = std::stoi(argv[i + 1]);
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    UdpSocket sock;
    if (!sock.open()) {
        std::cerr << "event=gcs_error msg=socket_failed"
                  << " error=\"" << UdpSocket::last_error() << "\"\n";
        return 1;
    }

    sockaddr_in proxy_addr{};
    proxy_addr.sin_family = AF_INET;
    proxy_addr.sin_port = htons(proxy_port);
    if (!resolve_ipv4(proxy_ip, proxy_addr.sin_addr)) {
        std::cerr << "event=resolve_error host=" << proxy_ip << "\n";
        return 1;
    }

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(proxy_port +1);
    local_addr.sin_addr.s_addr = INADDR_ANY;
    if (!sock.bind(local_addr)) {
        std::cerr << "event=gcs_error msg=bind_failed port="
                  << (proxy_port + 1)
                  << " error=\"" << UdpSocket::last_error() << "\"\n";
        return 1;
    }

    // tx buffer for GOTO commands
    uint8_t tx_buf[512];

    // GCS engine
    GcsEngine engine([](const SwarmSnapshot &snap)
                     {
                         // called on every telemetry update
                         // SSE server pulls via snapshot() instead
                     });

    // SSE server
    SseServer sse(engine, [&](uint8_t id, Vec3 target)
                  {
        uint16_t len = engine.pack_goto(id, target,
                                        tx_buf, sizeof(tx_buf));
        if (sock.send_to(tx_buf, len, proxy_addr) < 0) {
            std::cerr << "event=gcs_error msg=goto_send_failed id="
                      << (int)id
                      << " error=\"" << UdpSocket::last_error() << "\"\n";
        }
        std::cout << "event=goto_sent id=" << (int)id
                  << " x=" << target.x
                  << " y=" << target.y
                  << " z=" << target.z << "\n" << std::flush; },
                  [&](uint8_t id)
                  {
        uint16_t len = engine.pack_kill(id, tx_buf, sizeof(tx_buf));
        if (sock.send_to(tx_buf, len, proxy_addr) < 0) {
            std::cerr << "event=gcs_error msg=kill_send_failed id="
                      << (int)id
                      << " error=\"" << UdpSocket::last_error() << "\"\n";
        }
        std::cout << "event=kill_sent id=" << (int)id
                  << "\n" << std::flush; },
                  allowed_origin);
    g_sse = &sse;  // let the signal handler stop the blocking listen()

    // rx thread — receive MAVLink from proxy
    std::jthread rx([&](std::stop_token st)
                    {
        uint8_t buf[1024];
        while (!st.stop_requested() && running) {
            sockaddr_in from{};
            ssize_t     len = sock.recv_from(buf, sizeof(buf),
                                             MSG_DONTWAIT, from);
            if (len > 0) {
                engine.process(buf, static_cast<uint16_t>(len));
            } else {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(1));
            }
        } });

    // Keep the proxy's GCS endpoint fresh. Kubernetes pod IPs can change
    // across rollouts, and the proxy only knows where to forward telemetry
    // after receiving traffic from the GCS side.
    std::jthread registration([&](std::stop_token st)
                              {
        uint8_t hello = 0;
        while (!st.stop_requested() && running) {
            if (sock.send_to(&hello, 1, proxy_addr) < 0) {
                std::cerr << "event=gcs_error msg=registration_send_failed"
                          << " error=\"" << UdpSocket::last_error()
                          << "\"\n";
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    std::cout << "event=gcs_registered proxy=" << proxy_ip << "\n";

std::cout << "event=gcs_start"
          << " proxy=" << proxy_ip
          << " port="  << proxy_port
          << " http="  << http_port << "\n";

    // SSE server blocks on main thread
    if (!sse.listen("0.0.0.0", http_port)) {
        std::cerr << "event=gcs_error msg=http_listen_failed"
                  << " port=" << http_port << "\n";
        return 1;
    }

    running = false;
    std::cout << "event=gcs_stop\n";
    return 0;
}

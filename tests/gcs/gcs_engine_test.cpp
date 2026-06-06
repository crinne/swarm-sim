#include "gcs/gcs_engine.hpp"
#include "gcs/websocket.hpp"
#include "httplib/httplib.h"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class ServerRunner {
public:
    ServerRunner()
        : engine_([](const SwarmSnapshot&) {}),
          server_(engine_, [](uint8_t, Vec3) {},
                  "https://swarmgcs.dev,https://www.swarmgcs.dev")
    {
        port_ = server_.bind_to_any_port("127.0.0.1");
        require(port_ > 0, "failed to bind GCS test server");

        thread_ = std::thread([this] {
            listen_result_ = server_.listen_after_bind();
        });
        server_.wait_until_ready();

        if (!server_.is_running()) {
            stop();
            throw std::runtime_error("GCS test server failed to start");
        }
    }

    ~ServerRunner()
    {
        stop();
    }

    int port() const
    {
        return port_;
    }

    bool stop()
    {
        server_.stop();
        if (thread_.joinable()) {
            thread_.join();
        }
        return listen_result_;
    }

private:
    GcsEngine engine_;
    SseServer server_;
    int port_ = -1;
    bool listen_result_ = false;
    std::thread thread_;
};

void test_http_status_and_cors()
{
    ServerRunner server;
    httplib::Client client("127.0.0.1", server.port());
    client.set_connection_timeout(0, 100000);
    client.set_read_timeout(1, 0);

    auto health = client.Get("/health");
    require(health, "GET /health did not return a response");
    require(health->status == 200, "GET /health did not return 200");
    require(health->body == R"({"status":"ok"})",
            "GET /health returned an unexpected body");
    require(health->get_header_value("Access-Control-Allow-Origin")
                == "https://swarmgcs.dev",
            "GET /health returned an unexpected allowed origin");

    httplib::Headers www_origin = {{"Origin", "https://www.swarmgcs.dev"}};
    auto www_health = client.Get("/health", www_origin);
    require(www_health, "GET /health with www origin did not return a response");
    require(www_health->get_header_value("Access-Control-Allow-Origin")
                == "https://www.swarmgcs.dev",
            "GET /health did not echo the www allowed origin");

    auto ready = client.Get("/ready");
    require(ready, "GET /ready did not return a response");
    require(ready->status == 200, "GET /ready did not return 200");
    require(ready->body == R"({"status":"ready"})",
            "GET /ready returned an unexpected body");
    require(ready->get_header_value("Access-Control-Allow-Origin")
                == "https://swarmgcs.dev",
            "GET /ready returned an unexpected allowed origin");

    auto options = client.Options("/goto");
    require(options, "OPTIONS /goto did not return a response");
    require(options->get_header_value("Access-Control-Allow-Origin")
                == "https://swarmgcs.dev",
            "OPTIONS /goto returned an unexpected allowed origin");

    require(server.stop(), "GCS test server listen loop failed");
}

} // namespace

int main()
{
    try {
        test_http_status_and_cors();
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }

    std::cout << "PASS\n";
    return 0;
}

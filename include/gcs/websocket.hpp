#pragma once

#include "httplib/httplib.h"
#include "gcs/gcs_engine.hpp"
#include <string>
#include <sstream>
#include <mutex>
#include <vector>
#include <functional>
#include <utility>
#include <unordered_set>

class SseServer {
public:
    using GotoCallback = std::function<void(uint8_t, Vec3)>;
    using KillCallback = std::function<void(uint8_t)>;

    explicit SseServer(GcsEngine& engine, GotoCallback on_goto,
                       KillCallback on_kill,
                       std::string allowed_origin)
        : engine_(engine), on_goto_(std::move(on_goto)),
          on_kill_(std::move(on_kill)),
          allowed_origin_(std::move(allowed_origin)) {
        parse_allowed_origins();

        // telemetry stream
        server_.Get("/telemetry", [this](const httplib::Request& req,
                                         httplib::Response& res) {
            add_cors(req, res);
            res.set_header("Cache-Control", "no-cache");
            res.set_chunked_content_provider(
                "text/event-stream",
                [this](size_t, httplib::DataSink& sink) {
                    auto snap = engine_.snapshot();
                    std::string data = snapshot_to_json(snap);
                    std::string msg = "data: " + data + "\n\n";
                    sink.write(msg.c_str(), msg.size());
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(100));
                    return true;
                }
            );
        });

        // GOTO command
        server_.Post("/goto", [this](const httplib::Request& req,
                                     httplib::Response& res) {
            add_cors(req, res);
            // parse simple JSON: {"id":1,"x":10,"y":20,"z":5}
            uint8_t id = 0;
            float x = 0, y = 0, z = 0;
            sscanf(req.body.c_str(),
                   "{\"id\":%hhu,\"x\":%f,\"y\":%f,\"z\":%f}",
                   &id, &x, &y, &z);
            on_goto_(id, {x, y, z});
            res.set_content("{\"ok\":true}", "application/json");
        });

        server_.Post("/kill", [this](const httplib::Request& req,
                                     httplib::Response& res) {
            add_cors(req, res);
            uint8_t id = 0;
            sscanf(req.body.c_str(), "{\"id\":%hhu}", &id);
            on_kill_(id);
            res.set_content("{\"ok\":true}", "application/json");
        });

        server_.Get("/health", [this](const httplib::Request& req,
                                      httplib::Response& res) {
            add_cors(req, res);
            res.set_content("{\"status\":\"ok\"}", "application/json");
        });

        server_.Get("/ready", [this](const httplib::Request& req,
                                     httplib::Response& res) {
            add_cors(req, res);
            res.set_content("{\"status\":\"ready\"}", "application/json");
        });

        // CORS preflight
        server_.Options(".*", [this](const httplib::Request& req,
                                     httplib::Response& res) {
            add_cors(req, res);
            res.set_header("Access-Control-Allow-Methods",
                           "GET, POST, OPTIONS");
            res.set_header("Access-Control-Allow-Headers",
                           "Content-Type");
            res.set_content("", "text/plain");
        });
    }

    bool listen(const std::string& host, int port) {
        return server_.listen(host.c_str(), port);
    }

    int bind_to_any_port(const std::string& host) {
        return server_.bind_to_any_port(host);
    }

    bool listen_after_bind() {
        return server_.listen_after_bind();
    }

    void wait_until_ready() const {
        server_.wait_until_ready();
    }

    bool is_running() const {
        return server_.is_running();
    }

    void stop() { server_.stop(); }

private:
    httplib::Server server_;
    GcsEngine&      engine_;
    GotoCallback    on_goto_;
    KillCallback    on_kill_;
    std::string     allowed_origin_;
    std::vector<std::string> allowed_origins_;
    std::unordered_set<std::string> allowed_origin_set_;

    void parse_allowed_origins() {
        std::stringstream stream(allowed_origin_);
        std::string origin;
        while (std::getline(stream, origin, ',')) {
            auto start = origin.find_first_not_of(" \t\r\n");
            auto end = origin.find_last_not_of(" \t\r\n");
            if (start == std::string::npos) continue;
            origin = origin.substr(start, end - start + 1);
            allowed_origins_.push_back(origin);
            allowed_origin_set_.insert(origin);
        }
        if (allowed_origins_.empty()) {
            allowed_origins_.push_back(allowed_origin_);
            allowed_origin_set_.insert(allowed_origin_);
        }
    }

    void add_cors(const httplib::Request& req, httplib::Response& res) const {
        auto origin = req.get_header_value("Origin");
        if (!origin.empty() && allowed_origin_set_.count(origin) > 0) {
            res.set_header("Access-Control-Allow-Origin", origin);
        } else if (origin.empty()) {
            res.set_header("Access-Control-Allow-Origin", allowed_origins_.front());
        }
        res.set_header("Vary", "Origin");
    }

    std::string snapshot_to_json(const SwarmSnapshot& snap) {
        std::ostringstream os;
        os << "{\"drones\":[";
        bool first = true;
        for (auto& [id, d] : snap.drones) {
            if (!first) os << ",";
            first = false;
            os << "{"
               << "\"id\":"      << (int)d.id       << ","
               << "\"x\":"       << d.position.x    << ","
               << "\"y\":"       << d.position.y    << ","
               << "\"z\":"       << d.position.z    << ","
               << "\"heading\":" << d.heading        << ","
               << "\"battery\":" << d.battery        << ","
               << "\"mode\":"    << (int)d.mode
               << "}";
        }
        os << "]}";
        return os.str();
    }
};

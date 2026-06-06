#pragma once

#include <cstdint>
#include <string>
#include <atomic>
#include <mutex>
#include <random>
#include <vector>

#include <netinet/in.h>

struct Packet {
    std::vector<uint8_t> data;
    sockaddr_in          dest;
};

class Proxy {
public:
    explicit Proxy(uint16_t drone_port, uint16_t gcs_port,
                   float loss_pct = 0.0f,
                   uint32_t bw_limit_bps = 32 * 1024)
        : loss_pct_(loss_pct)
        , bw_limit_bps_(bw_limit_bps)
        , bytes_this_second_(0)
        , rng_(std::random_device{}())
        , dist_(0.0f, 1.0f)
    {}

    bool should_drop() {
        return dist_(rng_) < loss_pct_;
    }

    bool within_budget(uint32_t pkt_len) {
        // check-and-reserve atomically
        uint32_t current = bytes_this_second_.load(
            std::memory_order_relaxed);
        while (true) {
            if (current + pkt_len > bw_limit_bps_) return false;
            if (bytes_this_second_.compare_exchange_weak(
                current, current + pkt_len,
                std::memory_order_acq_rel)) return true;
        }
    }

    void reset_budget() {
        bytes_this_second_.store(0, std::memory_order_relaxed);
    }

    float    loss_pct()     const { return loss_pct_; }
    uint32_t bw_limit_bps() const { return bw_limit_bps_; }

private:
    float                   loss_pct_;
    uint32_t                bw_limit_bps_;
    std::atomic<uint32_t>   bytes_this_second_;
    std::mt19937            rng_;
    std::uniform_real_distribution<float> dist_;
};
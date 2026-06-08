#pragma once

#include <cstdint>
#include <atomic>
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
                   uint32_t bw_limit_bps = 512 * 1024);

    explicit Proxy(uint16_t drone_port, uint16_t gcs_port,
                   float loss_pct,
                   uint32_t bw_limit_bps,
                   uint32_t seed);

    bool should_drop();

    bool within_budget(uint32_t pkt_len);

    void reset_budget();

    float    loss_pct()     const { return loss_pct_; }
    uint32_t bw_limit_bps() const { return bw_limit_bps_; }

private:
    float                   loss_pct_;
    uint32_t                bw_limit_bps_;
    std::atomic<uint32_t>   bytes_this_second_;
    std::mt19937            rng_;
    std::uniform_real_distribution<float> dist_;
};

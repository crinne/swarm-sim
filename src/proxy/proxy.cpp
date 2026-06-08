#include "proxy/proxy.hpp"

Proxy::Proxy(uint16_t drone_port, uint16_t gcs_port,
             float loss_pct, uint32_t bw_limit_bps)
    : Proxy(drone_port, gcs_port, loss_pct, bw_limit_bps,
            std::random_device{}())
{}

Proxy::Proxy(uint16_t drone_port, uint16_t gcs_port,
             float loss_pct, uint32_t bw_limit_bps, uint32_t seed)
    : loss_pct_(loss_pct)
    , bw_limit_bps_(bw_limit_bps)
    , bytes_this_second_(0)
    , rng_(seed)
    , dist_(0.0f, 1.0f)
{
    (void)drone_port;
    (void)gcs_port;
}

bool Proxy::should_drop()
{
    return dist_(rng_) < loss_pct_;
}

bool Proxy::within_budget(uint32_t pkt_len)
{
    uint32_t current = bytes_this_second_.load(std::memory_order_relaxed);
    while (true) {
        if (current + pkt_len > bw_limit_bps_) {
            return false;
        }
        if (bytes_this_second_.compare_exchange_weak(
                current, current + pkt_len, std::memory_order_acq_rel)) {
            return true;
        }
    }
}

void Proxy::reset_budget()
{
    bytes_this_second_.store(0, std::memory_order_relaxed);
}

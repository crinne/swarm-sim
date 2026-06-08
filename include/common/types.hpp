#pragma once

#include <cstdint>
#include <string>

namespace mavmode
{
    constexpr uint8_t PREFLIGHT         = 0x00;
    constexpr uint8_t GUIDED_ARMED      = 0xD8;
    constexpr uint8_t AUTO_ARMED        = 0xBC;
    constexpr uint8_t MANUAL_ARMED      = 0xC0;
}

struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct DroneState
{
    uint8_t id = 0;
    Vec3 position{};
    Vec3 velocity{};
    Vec3 target{};

    float heading = 0.0f;
    float battery = 100.0f;
    uint8_t mode = mavmode::PREFLIGHT;
    bool armed = false;
};

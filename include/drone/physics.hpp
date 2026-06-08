#pragma once

#include "common/types.hpp"

class Physics {
public:
    explicit Physics(uint8_t id, Vec3 start_pos);

    void step(float dt);
    void set_target(Vec3 target);
    void set_orbit(float radius, float start_angle = 0.0f);
    void begin_failure();

    const DroneState& state() const { return state_; }
    DroneState& state() { return state_; }
    bool battery_depleted() const;
    bool finished() const;

private:
    DroneState state_;

    static constexpr float MAX_SPEED    = 10.0f; // m/s
    static constexpr float FAILURE_DRAIN_PER_SECOND = 35.0f;
    static constexpr float FAILURE_FALL_SPEED = 3.0f;
    float orbit_radius_ = 0.0f;
    float orbit_angle_  = 0.0f;
    float orbit_speed_  = 0.3f; // radians per second
    bool  orbiting_     = false;
    bool  failing_      = false;

    void fail(float dt);
    void update_orbit(float dt);
    void move_toward_target(float dt);
    void update_heading(float dt);
};

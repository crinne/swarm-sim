#pragma once

#include "common/types.hpp"
#include <cmath>

class Physics {
public:
    explicit Physics(uint8_t id, Vec3 start_pos) {
        state_.id       = id;
        state_.position = start_pos;
        state_.mode     = mavmode::GUIDED_ARMED;
        state_.armed    = true;
    }

    // called every tick — dt is fixed 0.01f (100Hz)
    void step(float dt) {
        update_orbit(dt);
        move_toward_target(dt);
        update_heading(dt);
        drain_battery(dt);
    }

    void set_target(Vec3 target) {
        state_.target = target;
        orbiting_ = false;
    }

    void set_orbit(float radius, float start_angle = 0.0f) {
        orbit_radius_ = radius;
        orbit_angle_  = start_angle;
        orbiting_     = true;
    }
    const DroneState& state() const { return state_; }
    DroneState& state() { return state_; }

private:
    DroneState state_;

    static constexpr float MAX_SPEED    = 10.0f; // m/s
    static constexpr float BATTERY_DRAIN = 0.001f; // % per tick

    float orbit_radius_ = 0.0f;
    float orbit_angle_  = 0.0f;
    float orbit_speed_  = 0.3f; // radians per second
    bool  orbiting_     = false;

    void update_orbit(float dt) {
        if (!orbiting_) return;

        orbit_angle_ += orbit_speed_ * dt;
        if (orbit_angle_ > 2.0f * M_PI) orbit_angle_ -= 2.0f * M_PI;

        state_.target.x = orbit_radius_ * std::cos(orbit_angle_);
        state_.target.y = orbit_radius_ * std::sin(orbit_angle_);
        state_.target.z = state_.position.z; // maintain altitude
    }

    void move_toward_target(float dt) {
        Vec3& pos = state_.position;
        Vec3& vel = state_.velocity;
        const Vec3& tgt = state_.target;

        // direction toward target
        float dx = tgt.x - pos.x;
        float dy = tgt.y - pos.y;
        float dz = tgt.z - pos.z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < 0.1f) {
            // close enough — stop
            vel = {};
            return;
        }

        // normalize and scale to max speed
        float scale = std::min(MAX_SPEED, dist) / dist;
        vel.x = dx * scale;
        vel.y = dy * scale;
        vel.z = dz * scale;

        pos.x += vel.x * dt;
        pos.y += vel.y * dt;
        pos.z += vel.z * dt;
    }

    void update_heading(float dt) {
        // heading points toward velocity direction
        if (std::abs(state_.velocity.x) < 0.01f &&
            std::abs(state_.velocity.y) < 0.01f) return;

        float target_heading = std::atan2(state_.velocity.x,
                                          state_.velocity.y)
                               * 180.0f / M_PI;
        if (target_heading < 0) target_heading += 360.0f;

        // smooth rotation toward target heading
        float diff = target_heading - state_.heading;
        if (diff > 180.0f)  diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;

        state_.heading += diff * 5.0f * dt;
        if (state_.heading < 0)    state_.heading += 360.0f;
        if (state_.heading > 360.0f) state_.heading -= 360.0f;
    }

    void drain_battery(float dt) {
        state_.battery = std::max(0.0f,
                         state_.battery - BATTERY_DRAIN);
    }
};

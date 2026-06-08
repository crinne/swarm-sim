#include "drone/physics.hpp"

#include <algorithm>
#include <cmath>

Physics::Physics(uint8_t id, Vec3 start_pos)
{
    state_.id = id;
    state_.position = start_pos;
    state_.mode = mavmode::GUIDED_ARMED;
    state_.armed = true;
}

void Physics::step(float dt)
{
    if (failing_) {
        fail(dt);
        return;
    }

    update_orbit(dt);
    move_toward_target(dt);
    update_heading(dt);
}

void Physics::set_target(Vec3 target)
{
    state_.target = target;
    orbiting_ = false;
}

void Physics::set_orbit(float radius, float start_angle)
{
    orbit_radius_ = radius;
    orbit_angle_ = start_angle;
    orbiting_ = true;
}

void Physics::begin_failure()
{
    failing_ = true;
    orbiting_ = false;
    state_.target = state_.position;
}

bool Physics::battery_depleted() const
{
    return state_.battery <= 0.0f;
}

bool Physics::finished() const
{
    return failing_ && battery_depleted() && state_.position.z <= 0.0f;
}

void Physics::fail(float dt)
{
    state_.battery = std::max(
        0.0f,
        state_.battery - FAILURE_DRAIN_PER_SECOND * dt
    );

    if (!battery_depleted()) {
        state_.velocity = {};
        return;
    }

    state_.armed = false;
    state_.mode = mavmode::PREFLIGHT;
    state_.velocity = {0.0f, 0.0f, -FAILURE_FALL_SPEED};
    state_.position.z = std::max(
        0.0f,
        state_.position.z - FAILURE_FALL_SPEED * dt
    );
    if (state_.position.z <= 0.0f) {
        state_.velocity = {};
    }
}

void Physics::update_orbit(float dt)
{
    if (!orbiting_) {
        return;
    }

    orbit_angle_ += orbit_speed_ * dt;
    if (orbit_angle_ > 2.0f * M_PI) {
        orbit_angle_ -= 2.0f * M_PI;
    }

    state_.target.x = orbit_radius_ * std::cos(orbit_angle_);
    state_.target.y = orbit_radius_ * std::sin(orbit_angle_);
    state_.target.z = state_.position.z;
}

void Physics::move_toward_target(float dt)
{
    Vec3& pos = state_.position;
    Vec3& vel = state_.velocity;
    const Vec3& tgt = state_.target;

    float dx = tgt.x - pos.x;
    float dy = tgt.y - pos.y;
    float dz = tgt.z - pos.z;
    float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < 0.1f) {
        vel = {};
        return;
    }

    float scale = std::min(MAX_SPEED, dist) / dist;
    vel.x = dx * scale;
    vel.y = dy * scale;
    vel.z = dz * scale;

    pos.x += vel.x * dt;
    pos.y += vel.y * dt;
    pos.z += vel.z * dt;
}

void Physics::update_heading(float dt)
{
    if (std::abs(state_.velocity.x) < 0.01f &&
        std::abs(state_.velocity.y) < 0.01f) {
        return;
    }

    float target_heading = std::atan2(state_.velocity.x, state_.velocity.y)
                           * 180.0f / M_PI;
    if (target_heading < 0) {
        target_heading += 360.0f;
    }

    float diff = target_heading - state_.heading;
    if (diff > 180.0f) {
        diff -= 360.0f;
    }
    if (diff < -180.0f) {
        diff += 360.0f;
    }

    state_.heading += diff * 5.0f * dt;
    if (state_.heading < 0) {
        state_.heading += 360.0f;
    }
    if (state_.heading > 360.0f) {
        state_.heading -= 360.0f;
    }
}

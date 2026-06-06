#include "drone/physics.hpp"
#include "drone/mcu.hpp"

#include <cassert>
#include <cmath>

static uint16_t pack_goto(uint8_t target_system, Vec3 target,
                          uint8_t* buffer) {
    mavlink_reset_channel_status(MAVLINK_COMM_1);

    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_pack_chan(
        255, 0, MAVLINK_COMM_1, &message,
        0, target_system, 0, MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        target.x, target.y, target.z,
        0, 0, 0, 0, 0, 0, 0, 0
    );

    return mavlink_msg_to_send_buffer(buffer, &message);
}

int main() {
    {
        Physics physics(1, {0.0f, 0.0f, 10.0f});
        physics.set_orbit(35.0f, 2.0f);

        physics.set_target({300.0f, -120.0f, 10.0f});
        physics.step(0.01f);

        const DroneState& state = physics.state();
        assert(state.velocity.x > 0.0f);
        assert(state.velocity.y < 0.0f);
    }

    {
        Physics physics(1, {0.0f, 0.0f, 10.0f});
        MCU mcu(physics);
        uint8_t buffer[128];
        uint16_t length = pack_goto(1, {300.0f, -120.0f, 10.0f},
                                    buffer);

        mcu.process(buffer, length);
        physics.step(0.01f);

        const DroneState& state = physics.state();
        assert(state.velocity.x > 0.0f);
        assert(state.velocity.y < 0.0f);
    }

    {
        Physics physics(2, {0.0f, 0.0f, 10.0f});
        physics.set_orbit(35.0f, 2.0f);
        MCU mcu(physics);
        uint8_t buffer[128];
        uint16_t length = pack_goto(1, {300.0f, -120.0f, 10.0f},
                                    buffer);

        mcu.process(buffer, length);
        physics.step(0.01f);

        const DroneState& state = physics.state();
        assert(state.velocity.x < 0.0f);
        assert(state.velocity.y > 0.0f);
    }

    {
        Physics physics(1, {0.0f, 0.0f, 10.0f});
        physics.set_target({0.0f, 100.0f, 10.0f});
        physics.step(0.2f);

        assert(std::abs(physics.state().heading) < 0.01f);
    }

    {
        Physics physics(1, {0.0f, 0.0f, 10.0f});
        physics.set_target({1.0f, 0.0f, 10.0f});
        for (int i = 0; i < 300; ++i) {
            physics.step(0.01f);
        }

        const DroneState& state = physics.state();
        assert(std::abs(state.position.x - 1.0f) < 0.1f);
        assert(std::abs(state.velocity.x) < 0.01f);
        assert(std::abs(state.velocity.y) < 0.01f);
    }

    return 0;
}

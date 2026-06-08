#include "proxy/proxy.hpp"
#include "drone/mcu.hpp"
#include "drone/physics.hpp"

#include <cassert>
#include <cstdint>
#include <vector>

namespace {

uint16_t pack_goto(uint8_t target_system, Vec3 target, uint8_t* buffer)
{
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

} // namespace

int main()
{
    {
        Proxy proxy(14550, 14560, 0.0f, 100);

        assert(proxy.within_budget(40));
        assert(proxy.within_budget(60));
        assert(!proxy.within_budget(1));

        proxy.reset_budget();
        assert(proxy.within_budget(100));
        assert(!proxy.within_budget(1));
    }

    {
        Proxy first(14550, 14560, 0.75f, 32 * 1024, 1234);
        Proxy second(14550, 14560, 0.75f, 32 * 1024, 1234);
        std::vector<bool> first_drops;
        std::vector<bool> second_drops;

        for (int i = 0; i < 200; ++i) {
            first_drops.push_back(first.should_drop());
            second_drops.push_back(second.should_drop());
        }

        assert(first_drops == second_drops);
    }

    {
        Proxy lossy_link(14550, 14560, 0.75f, 32 * 1024, 77);
        Physics physics(1, {0.0f, 0.0f, 10.0f});
        MCU mcu(physics);

        uint8_t buffer[128];
        uint16_t length = pack_goto(1, {30.0f, 0.0f, 10.0f}, buffer);
        int delivered = 0;

        for (int tick = 0; tick < 600; ++tick) {
            if (!lossy_link.should_drop()) {
                mcu.process(buffer, length);
                ++delivered;
            }
            physics.step(0.01f);
        }

        assert(delivered > 0);
        assert(physics.state().position.x > 25.0f);
        assert(physics.state().position.x < 30.5f);
    }

    return 0;
}

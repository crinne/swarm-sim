#pragma once

#include "mavlink/common/mavlink.h"
#include "types.hpp"
#include <cstdint>

namespace mavhelper
{

    // each done get its own channel to avoid the Tsan races
    // the previous canditate hiot from sharing channel ids
    inline mavlink_channel_t channel_for(uint8_t drone_id)
    {
        return static_cast<mavlink_channel_t>(drone_id % MAVLINK_COMM_NUM_BUFFERS);
    }

    // pack a heartbeat from drone state
    inline uint16_t pack_heartbeat(uint8_t drone_id, const DroneState &state, uint8_t *buf, uint16_t buflen)
{

    mavlink_message_t msg;
    mavlink_channel_t ch = channel_for(drone_id);

    mavlink_msg_heartbeat_pack_chan(
        drone_id, // system id
        0,        // component id
        ch,
        &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        state.mode,
        0,
        MAV_STATE_ACTIVE);

    return mavlink_msg_to_send_buffer(buf, &msg);
    }
}
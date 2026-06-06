#pragma once

#include "common/types.hpp"
#include "common/mavlink_helpers.hpp"
#include "drone/physics.hpp"
#include "mavlink/common/mavlink.h"
#include <cstdint>
#include <array>

class MCU {
public:
    explicit MCU(Physics& physics) : physics_(physics) {}

    // process incoming raw UDP bytes
    void process(const uint8_t* buf, uint16_t len) {
        mavlink_channel_t ch = mavhelper::channel_for(
            physics_.state().id
        );

        for (uint16_t i = 0; i < len; i++) {
            mavlink_message_t msg;
            mavlink_status_t  status;

            if (mavlink_parse_char(ch, buf[i], &msg, &status)) {
                handle_message(msg);
            }
        }
    }

    // pack and write telemetry into buf, return bytes written
    uint16_t send_telemetry(uint8_t* buf, uint16_t buflen) {
        const DroneState& s = physics_.state();
        uint8_t id = s.id;
        mavlink_channel_t ch = mavhelper::channel_for(id);
        mavlink_message_t msg;
        uint16_t total = 0;

        // heartbeat
        total += mavhelper::pack_heartbeat(id, s, buf + total,
                                           buflen - total);

        // position
        mavlink_msg_local_position_ned_pack_chan(
            id, 0, ch, &msg,
            0,          // time_boot_ms
            s.position.x,
            s.position.y,
            s.position.z,
            s.velocity.x,
            s.velocity.y,
            s.velocity.z
        );
        total += mavlink_msg_to_send_buffer(buf + total, &msg);

        // heading + airspeed (VFR_HUD — what previous candidate forgot)
        float speed = std::sqrt(
            s.velocity.x * s.velocity.x +
            s.velocity.y * s.velocity.y
        );
        mavlink_msg_vfr_hud_pack_chan(
            id, 0, ch, &msg,
            speed,        // airspeed
            speed,        // groundspeed
            s.heading,    // heading — shown in GCS
            0,            // throttle
            s.position.z, // alt
            s.velocity.z  // climb
        );
        total += mavlink_msg_to_send_buffer(buf + total, &msg);

        return total;
    }

private:
    Physics&  physics_;
    void handle_message(const mavlink_message_t& msg) {
        switch (msg.msgid) {
            case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                handle_goto(msg);
                break;
            default:
                break;
        }
    }

    void handle_goto(const mavlink_message_t& msg) {
        mavlink_set_position_target_local_ned_t cmd;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &cmd);
        if (cmd.target_system != physics_.state().id) return;

        physics_.set_target({cmd.x, cmd.y, cmd.z});
    }
};

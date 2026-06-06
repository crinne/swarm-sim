#pragma once

#include "common/types.hpp"
#include "common/mavlink_helpers.hpp"
#include "mavlink/common/mavlink.h"
#include <map>
#include <mutex>
#include <functional>
#include <cmath>

// snapshot of all known drones — safe to read from websocket thread
struct SwarmSnapshot {
    std::map<uint8_t, DroneState> drones;
};

class GcsEngine {
public:
    using SnapshotCallback = std::function<void(const SwarmSnapshot&)>;

    explicit GcsEngine(SnapshotCallback on_update)
        : on_update_(on_update) {}

    // called from rx thread with raw UDP bytes
    void process(const uint8_t* buf, uint16_t len) {
        mavlink_message_t msg;
        mavlink_status_t  status;

        for (uint16_t i = 0; i < len; i++) {
            // GCS uses channel 0 — receives only, no pack_chan conflict
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i],
                                   &msg, &status)) {
                handle_message(msg);
            }
        }
    }

    // pack a GOTO command for drone_id, return bytes written
    uint16_t pack_goto(uint8_t drone_id, Vec3 target,
                       uint8_t* buf, uint16_t buflen) {
        mavlink_message_t msg;
        mavlink_channel_t ch = mavhelper::channel_for(drone_id);

        mavlink_msg_set_position_target_local_ned_pack_chan(
            255,        // GCS system id
            0,
            ch,
            &msg,
            0,          // time_boot_ms
            drone_id,   // target system
            0,          // target component
            MAV_FRAME_LOCAL_NED,
            0b0000111111111000, // position only
            target.x, target.y, target.z,
            0, 0, 0,    // velocity
            0, 0, 0,    // acceleration
            0, 0        // yaw
        );

        return mavlink_msg_to_send_buffer(buf, &msg);
    }

    SwarmSnapshot snapshot() {
        std::lock_guard lk(mutex_);
        return SwarmSnapshot{drones_};
    }

private:
    SnapshotCallback              on_update_;
    std::mutex                    mutex_;
    std::map<uint8_t, DroneState> drones_;

    void handle_message(const mavlink_message_t& msg) {
        uint8_t id = msg.sysid;

        switch (msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT: {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
                std::lock_guard lk(mutex_);
                drones_[id].id   = id;
                drones_[id].mode = hb.base_mode;
                break;
            }
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                mavlink_local_position_ned_t pos;
                mavlink_msg_local_position_ned_decode(&msg, &pos);
                std::lock_guard lk(mutex_);
                drones_[id].position = {pos.x, pos.y, pos.z};
                drones_[id].velocity = {pos.vx, pos.vy, pos.vz};
                break;
            }
            case MAVLINK_MSG_ID_VFR_HUD: {
                mavlink_vfr_hud_t hud;
                mavlink_msg_vfr_hud_decode(&msg, &hud);
                std::lock_guard lk(mutex_);
                drones_[id].heading = hud.heading;
                break;
            }
            default:
                break;
        }

        // notify websocket thread
        on_update_(SwarmSnapshot{drones_});
    }
};
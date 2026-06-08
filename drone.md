# Drone Notes

The drone process is intentionally small. It owns one physics object and one
MCU object, reads commands from UDP, steps physics, and sends telemetry back to
the proxy.

Run example:

```bash
./build/drone --id 1 --proxy 127.0.0.1 --port 14550
```

The drone binds to `14550 + id`. That keeps multiple local drones from fighting
for the same source port.

## Main Loop

The assignment said the simulation should run on the main startup thread, so I
kept physics there. The loop is:

1. non-blocking read from UDP
2. parse any complete MAVLink messages in the MCU
3. step physics with `dt = 0.01`
4. send telemetry
5. sleep until the next 10 ms tick

There is no physics worker thread. If the drone is killed and finishes falling,
the loop exits and the process returns `0`.

## Commands

The MCU accepts two commands:

- `SET_POSITION_TARGET_LOCAL_NED` for GOTO
- `COMMAND_LONG / MAV_CMD_DO_FLIGHTTERMINATION` for crash/kill

Both commands check `target_system`, so a broadcast from the proxy only affects
the drone it was meant for.

## Telemetry

The drone sends:

- `HEARTBEAT`
- `LOCAL_POSITION_NED`
- `VFR_HUD`
- `BATTERY_STATUS`

I used real MAVLink mode bitmasks for the heartbeat instead of fake enum values,
because otherwise a real MAVLink consumer would decode nonsense.

## Physics

Normal movement is simple: point toward target and move up to `10 m/s`. When no
manual target is set, the drone can orbit so multiple drones are visible without
stacking on the same point.

Crash behavior:

1. freeze the target at the current position
2. drain battery quickly
3. at zero battery, mark the drone unarmed
4. fall until altitude reaches zero
5. finish the process

That last part is important for the Kubernetes demo: intentional crash exits
with code `0`, and the spawned pods use `OnFailure`, so Kubernetes does not
restart a drone that was deliberately killed.

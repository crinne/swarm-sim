# Proxy Notes

The proxy is the simulated radio link. It is deliberately separate from the
drone and GCS so packet loss can be tested without changing either endpoint.

Run example:

```bash
./build/proxy --drone-port 14550 --gcs-port 14560 --loss 0.75
```

## How It Routes

The proxy learns endpoints from traffic:

- packets on the drone-facing socket register drone addresses
- packets on the GCS-facing socket register the GCS address

When the GCS sends a command, the proxy forwards it to all known drones. That
sounds broad, but the MAVLink packet still has a `target_system`, and each MCU
ignores commands not meant for its own ID. This kept the proxy simple.

## Loss and Bandwidth

`--loss` is a per-packet drop probability. For normal runtime the random
generator is seeded from `std::random_device`.

For tests, the proxy can also take an explicit seed. That makes the `75%` loss
test repeatable instead of flaky.

Drone-to-GCS traffic has a byte budget. The reservation is atomic, so two
threads cannot both see spare budget and overbook it.

## Why the Test Is Written This Way

The important question is not “can I drop 75% of packets?” It is “does the
drone still respond to control when most packets are gone?”

The proxy test sends real MAVLink GOTO packets through the seeded loss model.
Only packets that survive are passed into the MCU. Physics then runs normally.
At the end the test checks that the drone got close to the target. That is the
behavior the assignment seemed to care about.

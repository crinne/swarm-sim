# Swarm Simulator

This is the C++ part of the swarm simulator. I kept it as three separate
programs because it made the radio-link part easier to reason about and test:

- `drone` simulates one vehicle.
- `proxy` sits between drones and the GCS and can drop packets.
- `gcs` receives telemetry and exposes a small HTTP/SSE API for the web UI.

The web UI, Kubernetes manifests, and controller live in other repos. This repo
is the simulator itself.

## Build and Test

I used CMake so the normal path is:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Requirements are just a C++20 compiler, CMake 3.20+, and POSIX sockets. The
MAVLink headers and cpp-httplib are vendored under `libs/`, so there is no
package manager step for the simulator.

For thread-sanitizer:

```bash
cmake -S . -B build-tsan -DENABLE_TSAN=ON
cmake --build build-tsan
ctest --test-dir build-tsan --output-on-failure
```

## Running It By Hand

Start the proxy first:

```bash
./build/proxy --drone-port 14550 --gcs-port 14560 --loss 0
```

Then the GCS:

```bash
./build/gcs --proxy 127.0.0.1 --port 14560 --http-port 8080
```

Then one or more drones:

```bash
./build/drone --id 1 --proxy 127.0.0.1 --port 14550
./build/drone --id 2 --proxy 127.0.0.1 --port 14550 --x 20 --y 20
```

The GCS API is:

- `GET /health`
- `GET /ready`
- `GET /telemetry`
- `POST /goto` with `{"id":1,"x":10,"y":5,"z":20}`
- `POST /kill` with `{"id":1}`

## What I Was Optimizing For

The assignment was mainly about whether the simulator is understandable and
whether control still works when the link is bad. So the design is deliberately
plain:

- one process per drone, so each drone has a clean system ID and lifecycle
- fixed-step physics in the drone process, on the main thread
- a separate proxy process for packet loss instead of baking loss into the
  drone or GCS
- CTest tests for the parts that are easy to lie to yourself about

The `75%` packet loss test sends real MAVLink GOTO packets through a seeded
loss model, delivers the surviving packets into the MCU, advances physics, and
checks that the drone actually moves toward the target. That matters more than
only testing that packets can be forwarded.

## Layout

```text
include/common/   shared types, MAVLink helpers, socket wrapper
include/drone/    drone interfaces
include/gcs/      GCS engine and HTTP/SSE server
include/proxy/    proxy interface
src/drone/        drone executable and physics implementation
src/gcs/          GCS executable
src/proxy/        proxy executable and proxy implementation
tests/            CTest tests
libs/             vendored headers
```

## Known Tradeoffs

This is still a small simulator, not a full autopilot. The proxy broadcasts GCS
commands to known drones and lets the MAVLink target system decide who accepts
the command. That is simple and good enough for this assignment.

Some code is still header-based, especially the MCU and GCS engine. I moved the
larger physics and proxy logic into `.cpp` files because those were the parts
where implementation details were starting to crowd the interfaces.

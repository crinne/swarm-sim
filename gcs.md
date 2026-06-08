# GCS Notes

The GCS is the bridge between MAVLink UDP and the browser UI. It does not try
to be a full ground-control station. It keeps the latest state per drone and
offers a small HTTP API.

Run example:

```bash
./build/gcs --proxy 127.0.0.1 --port 14560 --http-port 8080
```

## Threads

The HTTP server blocks on the main thread. MAVLink receive runs in a background
thread because the server needs to keep accepting clients while UDP telemetry is
arriving.

There is also a small registration loop that sends one byte to the proxy every
second. This is mostly for Kubernetes: if the GCS pod is recreated, the proxy
needs to learn the new source endpoint.

## State

`GcsEngine` parses these MAVLink messages:

- `HEARTBEAT` for mode
- `LOCAL_POSITION_NED` for position and velocity
- `VFR_HUD` for heading
- `BATTERY_STATUS` for battery percentage

If a drone stops sending telemetry for two seconds, it is removed from the
snapshot. That is how a killed drone disappears from the UI.

## API

- `GET /health`
- `GET /ready`
- `GET /telemetry`
- `POST /goto`
- `POST /kill`

`/goto` packs `SET_POSITION_TARGET_LOCAL_NED`.

`/kill` packs `MAV_CMD_DO_FLIGHTTERMINATION`.

The production deployment uses CORS because the UI and API are on different
hosts. I kept that in the C++ server instead of relying on the browser ignoring
it, because Firefox caught this immediately during testing.

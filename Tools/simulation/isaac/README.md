# Isaac Sim Boat SITL

This directory contains the PX4-side helper for connecting the `px4_sitl_boat`
target to an NVIDIA Isaac Sim boat scene.

## PX4 side

Build the boat SITL target:

```sh
make px4_sitl_boat
```

Run the Isaac boat airframe:

```sh
PX4_SIM_MODEL=isaac_boat PX4_SIMULATOR=isaac ./build/px4_sitl_boat/bin/px4
```

By default PX4 starts `simulator_mavlink` using the same simulator endpoint
policy as the existing SITL targets:

```text
TCP simulator port = 4560 + px4_instance
```

For instance 0 this is TCP port `4560` on localhost. PX4 connects to this TCP
port, so the Isaac bridge should listen as a TCP server unless
`PX4_SIM_HOSTNAME` or `PX4_SIM_HOST_ADDR` is used to point PX4 somewhere else.
The standard SITL MAVLink links are unchanged:

```text
GCS local UDP       18570 + px4_instance
Offboard local UDP  14580 + px4_instance
Offboard remote UDP 14540 + px4_instance
```

## Bridge contract

The Isaac bridge connects to PX4's simulator MAVLink endpoint and exchanges:

- Isaac Sim to PX4: `HEARTBEAT`, `HIL_SENSOR`, `HIL_GPS`
- PX4 to Isaac Sim: `HIL_ACTUATOR_CONTROLS`

The first two actuator controls are treated as the boat's two main propulsion
outputs. The exact left/right mapping must match the Isaac boat model's
thruster layout.

## Standalone bridge smoke test

The Python bridge can run without Isaac Sim and publish static sensor data. This
is useful for checking that PX4 accepts a simulator connection before wiring the
script into an Isaac physics callback:

```sh
python3 Tools/simulation/isaac/bridge/isaac_boat_mavlink_bridge.py --host 127.0.0.1 --port 4560
```

Isaac integration should replace the static `BoatState` sample with values read
from the USD stage and apply received `BoatActuatorCommand` values to the boat
thrusters.

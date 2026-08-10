# PX4 custom Gazebo assets

This directory contains PX4-Autopilot-owned Gazebo assets that should not be
committed inside the `Tools/simulation/gz` submodule.

## Boat SITL

`worlds/boat.sdf` and `models/boat/model.sdf` provide the first Gazebo Harmonic
boat SITL environment for `make px4_sitl gz_boat`.

The temporary boat model intentionally uses SDF primitives. Replace the hull
visual and collision blocks with STP-derived meshes when the production boat
geometry is ready.

The current world is a lightweight local water test scene. For higher-fidelity
maritime simulation, the next extension points are:

- OSRF VRX / WAM-V-style ocean worlds and tasks.
- `asv_wave_sim`-style wave and surface-vessel dynamics plugins.
- Calibrated hydrodynamic coefficients for the final hull and rudder geometry.

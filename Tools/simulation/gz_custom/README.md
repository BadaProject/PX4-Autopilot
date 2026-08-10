# PX4 custom Gazebo assets

This directory contains PX4-Autopilot-owned Gazebo assets that should not be
committed inside the `Tools/simulation/gz` submodule.

## Boat SITL

`worlds/boat.sdf` and `models/boat/model.sdf` provide the first Gazebo Harmonic
boat SITL environment for `make px4_sitl gz_boat`.

The boat model is now parameterized from the BadaSim Aura configuration. The
visual hull uses `models/boat/meshes/Aura_low.obj`, extracted from the BadaSim
`Aura_low.usdc` asset that was generated from `~/projects/AuraHull.stp`.

The current model is the stage-1 stability baseline from
`gazebo_fix_plan.md`: the visual is AuraHull, but the physics hull is a single
rectangular reference box. This deliberately avoids detailed hull-shape effects
while checking buoyancy, axes, thrust direction, and PX4 actuator mapping.
Propulsion is two rear propellers driven from the same signed thrust command for
the first straight-motion test; the rudder and hydrodynamics plugins are
disabled for this baseline.

The stage-1 world intentionally uses `basic_buoyancy` instead of graded
waterline slicing. This removes the strong pitch torque from a partially
submerged box and lets the lowered center of mass stabilize roll and pitch
first. Graded buoyancy should be reintroduced only after the baseline no longer
flips at rest.

The wheel output range is `0..200` with disarmed output `100`. The Gazebo wheel
bridge subtracts `100` before publishing `/model/boat_0/command/motor_speed`,
so the propeller plugins receive `0` at rest, positive values for forward
thrust, and negative values for reverse thrust.

Stage-1 smoke checks:

1. Start `make px4_sitl gz_boat`.
2. Leave the boat uncommanded for at least 60 seconds and check that it does not
   sink or develop unbounded roll/pitch.
3. Arm in Manual mode and raise the virtual joystick throttle above center.
4. Check that both propellers receive the same nonzero motor speed and the boat
   moves straight.
5. Reintroduce differential thrust or a rudder only after the straight-thrust
   baseline is confirmed.

The current world is a lightweight local water test scene. For higher-fidelity
maritime simulation, the next extension points are:

- OSRF VRX / WAM-V-style ocean worlds and tasks.
- `asv_wave_sim`-style wave and surface-vessel dynamics plugins.
- Calibrated hydrodynamic coefficients for the final hull and rudder geometry.

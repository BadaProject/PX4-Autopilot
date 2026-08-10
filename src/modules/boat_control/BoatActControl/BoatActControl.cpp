/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "BoatActControl.hpp"

BoatActControl::BoatActControl(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
}

void BoatActControl::updateParams()
{
	ModuleParams::updateParams();
}

void BoatActControl::updateActControl()
{
	_timestamp = hrt_absolute_time();

	if (_rover_throttle_setpoint_sub.updated()) {
		rover_throttle_setpoint_s rover_throttle_setpoint{};
		_rover_throttle_setpoint_sub.copy(&rover_throttle_setpoint);
		_signed_thrust_setpoint = rover_throttle_setpoint.throttle_body_x;
	}

	if (PX4_ISFINITE(_signed_thrust_setpoint)) {
		actuator_motors_s actuator_motors{};
		actuator_motors.reversible_flags = _param_r_rev.get();
		actuator_motors.control[0] = slewSignedThrust(math::constrain(_signed_thrust_setpoint, -1.f, 1.f));
		actuator_motors.timestamp = _timestamp;
		_actuator_motors_pub.publish(actuator_motors);
	}

	if (_rover_steering_setpoint_sub.updated()) {
		rover_steering_setpoint_s rover_steering_setpoint{};
		_rover_steering_setpoint_sub.copy(&rover_steering_setpoint);
		_steering_setpoint = rover_steering_setpoint.normalized_steering_setpoint;
	}

	if (PX4_ISFINITE(_steering_setpoint)) {
		actuator_servos_s actuator_servos{};
		actuator_servos.control[0] = math::constrain(_steering_setpoint, -1.f, 1.f);
		actuator_servos.timestamp = _timestamp;
		_actuator_servos_pub.publish(actuator_servos);
	}
}

void BoatActControl::stopVehicle()
{
	_timestamp = hrt_absolute_time();

	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	actuator_motors.control[0] = 0.f;
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);

	actuator_servos_s actuator_servos{};
	actuator_servos.control[0] = 0.f;
	actuator_servos.timestamp = _timestamp;
	_actuator_servos_pub.publish(actuator_servos);

	reset();
}

void BoatActControl::reset()
{
	_signed_thrust_setpoint = NAN;
	_steering_setpoint = NAN;
	_last_signed_thrust_setpoint = 0.f;
	_last_signed_thrust_update = 0;
}

float BoatActControl::slewSignedThrust(float signed_thrust_setpoint)
{
	const float dt = _last_signed_thrust_update > 0 ?
			 math::constrain((_timestamp - _last_signed_thrust_update) * 1e-6f, 0.001f, 0.1f) : 0.01f;
	const bool increasing_magnitude = fabsf(signed_thrust_setpoint) > fabsf(_last_signed_thrust_setpoint);
	const float rate_limit = math::max(increasing_magnitude ? _param_boat_eng_ramp_up.get() :
					  _param_boat_eng_ramp_dn.get(), 0.f);

	if (rate_limit > FLT_EPSILON) {
		_last_signed_thrust_setpoint += math::constrain(signed_thrust_setpoint - _last_signed_thrust_setpoint,
						    -rate_limit * dt, rate_limit * dt);

	} else {
		_last_signed_thrust_setpoint = signed_thrust_setpoint;
	}

	_last_signed_thrust_update = _timestamp;
	return math::constrain(_last_signed_thrust_setpoint, -1.f, 1.f);
}

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
		_throttle_setpoint = rover_throttle_setpoint.throttle_body_x;
	}

	if (PX4_ISFINITE(_throttle_setpoint)) {
		actuator_motors_s actuator_motors{};
		actuator_motors.reversible_flags = _param_r_rev.get();
		actuator_motors.control[0] = math::constrain(_throttle_setpoint, 0.f, 1.f);
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

	_throttle_setpoint = NAN;
	_steering_setpoint = NAN;
}

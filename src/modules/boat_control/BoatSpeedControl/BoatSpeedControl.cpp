/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "BoatSpeedControl.hpp"

BoatSpeedControl::BoatSpeedControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_throttle_setpoint_pub.advertise();
	_rover_speed_status_pub.advertise();
	updateParams();
}

void BoatSpeedControl::updateParams()
{
	ModuleParams::updateParams();
}

void BoatSpeedControl::updateSpeedControl()
{
	updateSubscriptions();

	const hrt_abstime timestamp = hrt_absolute_time();

	if (PX4_ISFINITE(_speed_setpoint)) {
		const float speed_setpoint = math::constrain(_speed_setpoint, 0.f, _param_boat_speed_lim.get());
		const float proposed_thrust = _param_boat_thr_ff.get() * speed_setpoint;
		const float max_change = _param_boat_thr_rate.get();
		float thrust_change = proposed_thrust - _last_thrust;

		if (max_change > FLT_EPSILON) {
			thrust_change = math::constrain(thrust_change, -max_change, max_change);
		}

		const float thrust = _last_thrust + thrust_change;
		_last_thrust = math::max(thrust, 0.f);

		float remapped_thrust = 0.f;

		if (_last_thrust > FLT_EPSILON && _param_boat_thr_scale.get() > FLT_EPSILON) {
			remapped_thrust = sqrtf(_last_thrust / _param_boat_thr_scale.get());
		}

		remapped_thrust = math::constrain(remapped_thrust, 0.f, _param_boat_thr_max.get());
		const float normalized_throttle = _param_boat_thr_max.get() > FLT_EPSILON ?
						  remapped_thrust / _param_boat_thr_max.get() : 0.f;

		rover_throttle_setpoint_s rover_throttle_setpoint{};
		rover_throttle_setpoint.timestamp = timestamp;
		rover_throttle_setpoint.throttle_body_x = math::constrain(normalized_throttle, 0.f, 1.f);
		rover_throttle_setpoint.throttle_body_y = NAN;
		_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

		_adjusted_speed_setpoint = speed_setpoint;
	}

	rover_speed_status_s rover_speed_status{};
	rover_speed_status.timestamp = timestamp;
	rover_speed_status.measured_speed_body_x = _vehicle_speed;
	rover_speed_status.adjusted_speed_body_x_setpoint = _adjusted_speed_setpoint;
	rover_speed_status.pid_throttle_body_x_integral = NAN;
	rover_speed_status.measured_speed_body_y = NAN;
	rover_speed_status.adjusted_speed_body_y_setpoint = NAN;
	rover_speed_status.pid_throttle_body_y_integral = NAN;
	_rover_speed_status_pub.publish(rover_speed_status);
}

void BoatSpeedControl::updateSubscriptions()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = Quatf(vehicle_attitude.q);
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		const Vector3f velocity_ned(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		const Vector3f velocity_body = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_ned);
		_vehicle_speed = velocity_body(0);
	}

	if (_rover_speed_setpoint_sub.updated()) {
		rover_speed_setpoint_s rover_speed_setpoint{};
		_rover_speed_setpoint_sub.copy(&rover_speed_setpoint);
		_speed_setpoint = rover_speed_setpoint.speed_body_x;
	}
}

bool BoatSpeedControl::runSanityChecks()
{
	bool ret = true;

	if (_param_boat_speed_lim.get() < FLT_EPSILON) {
		ret = false;
		events::send<float>(events::ID("boat_speed_control_invalid_speed_lim"), events::Log::Error,
				     "Invalid configuration of parameter BOAT_SPEED_LIM", _param_boat_speed_lim.get());
	}

	if (_param_boat_thr_max.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_boat_thr_scale.get() < FLT_EPSILON) {
		ret = false;
	}

	return ret;
}

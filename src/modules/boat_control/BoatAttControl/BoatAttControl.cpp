/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "BoatAttControl.hpp"

using namespace time_literals;

BoatAttControl::BoatAttControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_steering_setpoint_pub.advertise();
	_rover_attitude_status_pub.advertise();
	updateParams();
}

void BoatAttControl::updateParams()
{
	ModuleParams::updateParams();
}

void BoatAttControl::updateAttControl()
{
	updateSubscriptions();

	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 100_ms) * 1e-6f;

	float normalized_steering = NAN;

	if (PX4_ISFINITE(_yaw_setpoint)) {
		const float error = matrix::wrap_pi(_yaw_setpoint - _vehicle_yaw);
		normalized_steering = normalizedSteering(steeringFromYawError(error, dt));

	} else if (PX4_ISFINITE(_yaw_rate_setpoint)) {
		const float max_yaw_rate = _param_boat_y_rate_lim.get() * M_DEG_TO_RAD_F;
		normalized_steering = max_yaw_rate > FLT_EPSILON ?
				      math::constrain(_yaw_rate_setpoint / max_yaw_rate, -1.f, 1.f) : 0.f;
		_last_error = 0.f;
	}

	if (PX4_ISFINITE(normalized_steering)) {
		rover_steering_setpoint_s rover_steering_setpoint{};
		rover_steering_setpoint.timestamp = _timestamp;
		rover_steering_setpoint.normalized_steering_setpoint = normalized_steering;
		_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
	}

	rover_attitude_status_s rover_attitude_status{};
	rover_attitude_status.timestamp = _timestamp;
	rover_attitude_status.measured_yaw = _vehicle_yaw;
	rover_attitude_status.adjusted_yaw_setpoint = PX4_ISFINITE(_yaw_setpoint) ? _yaw_setpoint : NAN;
	_rover_attitude_status_pub.publish(rover_attitude_status);
}

void BoatAttControl::updateSubscriptions()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_yaw = Eulerf(Quatf(vehicle_attitude.q)).psi();
	}

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		_vehicle_yaw_rate = vehicle_angular_velocity.xyz[2];
	}

	if (_rover_attitude_setpoint_sub.updated()) {
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		_rover_attitude_setpoint_sub.copy(&rover_attitude_setpoint);
		_yaw_setpoint = rover_attitude_setpoint.yaw_setpoint;

		if (PX4_ISFINITE(_yaw_setpoint)) {
			_yaw_rate_setpoint = NAN;
		}
	}

	if (_rover_rate_setpoint_sub.updated()) {
		rover_rate_setpoint_s rover_rate_setpoint{};
		_rover_rate_setpoint_sub.copy(&rover_rate_setpoint);
		_yaw_rate_setpoint = rover_rate_setpoint.yaw_rate_setpoint;

		if (PX4_ISFINITE(_yaw_rate_setpoint)) {
			_yaw_setpoint = NAN;
		}
	}
}

float BoatAttControl::steeringFromYawError(float error, float dt)
{
	const float derivative = matrix::wrap_pi(error - _last_error) * 0.1f / math::max(dt, 1e-3f);
	float steering = -_param_boat_str_p.get() * error - _param_boat_str_d.get() * derivative;
	steering = math::constrain(steering, -_param_boat_str_max.get(), _param_boat_str_max.get());

	const float max_change = _param_boat_str_rate.get();

	if (max_change > FLT_EPSILON) {
		const float steering_change = math::constrain(steering - _last_steering, -max_change, max_change);
		steering = _last_steering + steering_change;
	}

	_last_error = error;
	_last_steering = steering;
	return steering;
}

float BoatAttControl::normalizedSteering(float steering) const
{
	return _param_boat_str_max.get() > FLT_EPSILON ?
	       math::constrain(steering / _param_boat_str_max.get(), -1.f, 1.f) : 0.f;
}

bool BoatAttControl::runSanityChecks()
{
	bool ret = true;

	if (_param_boat_str_max.get() < FLT_EPSILON) {
		ret = false;
		events::send<float>(events::ID("boat_att_control_invalid_str_max"), events::Log::Error,
				     "Invalid configuration of parameter BOAT_STR_MAX", _param_boat_str_max.get());
	}

	if (_param_boat_y_rate_lim.get() < FLT_EPSILON) {
		ret = false;
	}

	return ret;
}

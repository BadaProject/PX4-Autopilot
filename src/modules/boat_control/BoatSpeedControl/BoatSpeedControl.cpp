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

	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = timestamp_prev > 0 ? math::constrain((_timestamp - timestamp_prev) * 1e-6f, 0.001f, 0.1f) : 0.01f;

	if (PX4_ISFINITE(_speed_setpoint)) {
		const float speed_setpoint = math::constrain(_speed_setpoint, -_param_boat_speed_lim.get(),
					     _param_boat_speed_lim.get());
		const float adjusted_speed_setpoint = updateAdjustedSpeedSetpoint(speed_setpoint, dt, _timestamp);
		const float signed_thrust = calculateSignedThrust(adjusted_speed_setpoint, dt);

		rover_throttle_setpoint_s rover_throttle_setpoint{};
		rover_throttle_setpoint.timestamp = _timestamp;
		rover_throttle_setpoint.throttle_body_x = signed_thrust;
		rover_throttle_setpoint.throttle_body_y = NAN;
		_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
	}

	rover_speed_status_s rover_speed_status{};
	rover_speed_status.timestamp = _timestamp;
	rover_speed_status.measured_speed_body_x = _vehicle_speed;
	rover_speed_status.adjusted_speed_body_x_setpoint = _adjusted_speed_setpoint;
	rover_speed_status.pid_throttle_body_x_integral = _param_boat_spd_en.get() != 0 ? _speed_error_integral : NAN;
	rover_speed_status.measured_speed_body_y = NAN;
	rover_speed_status.adjusted_speed_body_y_setpoint = NAN;
	rover_speed_status.pid_throttle_body_y_integral = NAN;
	_rover_speed_status_pub.publish(rover_speed_status);
}

float BoatSpeedControl::updateAdjustedSpeedSetpoint(float requested_speed_setpoint, float dt, hrt_abstime timestamp)
{
	const float requested_speed_abs = fabsf(requested_speed_setpoint);

	if (requested_speed_abs < FLT_EPSILON) {
		_adjusted_speed_setpoint = 0.f;
		_last_requested_speed_setpoint = 0.f;
		_startup_ramp_active = false;
		_startup_ramp_reached_time = 0;
		_last_thrust = 0.f;
		_speed_error_integral = 0.f;
		_previous_speed_error = NAN;
		return _adjusted_speed_setpoint;
	}

	const bool new_start = fabsf(_last_requested_speed_setpoint) < FLT_EPSILON
			       || matrix::sign(_last_requested_speed_setpoint) != matrix::sign(requested_speed_setpoint);
	const float ramp_speed = math::max(_param_boat_ramp_spd.get(), 0.f);

	if (new_start && ramp_speed > FLT_EPSILON && requested_speed_abs > ramp_speed) {
		_startup_ramp_active = true;
		_startup_ramp_reached_time = 0;
	}

	float target_speed_abs = requested_speed_abs;

	if (_startup_ramp_active) {
		target_speed_abs = math::min(requested_speed_abs, ramp_speed);

		if (fabsf(_adjusted_speed_setpoint) >= target_speed_abs - 0.05f) {
			if (_startup_ramp_reached_time == 0) {
				_startup_ramp_reached_time = timestamp;
			}

			const hrt_abstime hold_time_us = (hrt_abstime)(math::max(_param_boat_ramp_hold.get(), 0.f) * 1000000.f);

			if (timestamp - _startup_ramp_reached_time >= hold_time_us) {
				_startup_ramp_active = false;
				_startup_ramp_reached_time = 0;
				target_speed_abs = requested_speed_abs;
			}

		} else {
			_startup_ramp_reached_time = 0;
		}
	}

	const float target_speed_setpoint = matrix::sign(requested_speed_setpoint) * target_speed_abs;
	const bool increasing_magnitude = fabsf(target_speed_setpoint) > fabsf(_adjusted_speed_setpoint);
	const float rate_limit = math::max(increasing_magnitude ? _param_boat_spd_acc_up.get() : _param_boat_spd_acc_dn.get(),
					  0.f);

	if (rate_limit > FLT_EPSILON) {
		_adjusted_speed_setpoint += math::constrain(target_speed_setpoint - _adjusted_speed_setpoint,
					     -rate_limit * dt, rate_limit * dt);

	} else {
		_adjusted_speed_setpoint = target_speed_setpoint;
	}

	_last_requested_speed_setpoint = requested_speed_setpoint;
	return _adjusted_speed_setpoint;
}

float BoatSpeedControl::calculateSignedThrust(float adjusted_speed_setpoint, float dt)
{
	if (fabsf(adjusted_speed_setpoint) < FLT_EPSILON) {
		_last_thrust = 0.f;
		_speed_error_integral = 0.f;
		_previous_speed_error = NAN;
		return 0.f;
	}

	float raw_thrust = _param_boat_thr_ff.get() * adjusted_speed_setpoint;

	if (_param_boat_spd_en.get() != 0 && PX4_ISFINITE(_vehicle_speed)) {
		const float speed_error = adjusted_speed_setpoint - _vehicle_speed;
		const float previous_integral = _speed_error_integral;
		const float speed_error_derivative = PX4_ISFINITE(_previous_speed_error) ?
						     (speed_error - _previous_speed_error) / dt : 0.f;

		_speed_error_integral = math::constrain(_speed_error_integral + speed_error * dt,
							-_param_boat_spd_imax.get(), _param_boat_spd_imax.get());

		float feedback_thrust = _param_boat_spd_p.get() * speed_error
					+ _param_boat_spd_i.get() * _speed_error_integral
					+ _param_boat_spd_d.get() * speed_error_derivative;
		raw_thrust += feedback_thrust;

		const float signed_thrust_preview = remapThrustToSignedSetpoint(raw_thrust);

		if (fabsf(signed_thrust_preview) >= 1.f - FLT_EPSILON
		    && matrix::sign(speed_error) == matrix::sign(signed_thrust_preview)) {
			_speed_error_integral = previous_integral;
			feedback_thrust = _param_boat_spd_p.get() * speed_error
					  + _param_boat_spd_i.get() * _speed_error_integral
					  + _param_boat_spd_d.get() * speed_error_derivative;
			raw_thrust = _param_boat_thr_ff.get() * adjusted_speed_setpoint + feedback_thrust;
		}

		_previous_speed_error = speed_error;

	} else {
		_speed_error_integral = 0.f;
		_previous_speed_error = NAN;
	}

	const float max_change = _param_boat_thr_rate.get();
	float thrust_change = raw_thrust - _last_thrust;

	if (max_change > FLT_EPSILON) {
		thrust_change = math::constrain(thrust_change, -max_change, max_change);
	}

	_last_thrust += thrust_change;

	return remapThrustToSignedSetpoint(_last_thrust);
}

float BoatSpeedControl::remapThrustToSignedSetpoint(float thrust) const
{
	if (fabsf(thrust) < FLT_EPSILON || _param_boat_thr_scale.get() < FLT_EPSILON
	    || _param_boat_thr_max.get() < FLT_EPSILON) {
		return 0.f;
	}

	const float remapped_thrust = sqrtf(fabsf(thrust) / _param_boat_thr_scale.get());
	return matrix::sign(thrust) * math::constrain(remapped_thrust / _param_boat_thr_max.get(), 0.f, 1.f);
}

void BoatSpeedControl::reset()
{
	_speed_setpoint = NAN;
	_last_thrust = 0.f;
	_last_requested_speed_setpoint = 0.f;
	_adjusted_speed_setpoint = 0.f;
	_speed_error_integral = 0.f;
	_previous_speed_error = NAN;
	_startup_ramp_active = false;
	_startup_ramp_reached_time = 0;
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

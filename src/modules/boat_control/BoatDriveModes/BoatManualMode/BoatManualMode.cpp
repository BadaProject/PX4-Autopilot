/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "BoatManualMode.hpp"

BoatManualMode::BoatManualMode(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_throttle_setpoint_pub.advertise();
	_rover_steering_setpoint_pub.advertise();
	_rover_rate_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	_rover_speed_setpoint_pub.advertise();
	_rover_position_setpoint_pub.advertise();
}

void BoatManualMode::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_boat_y_rate_lim.get() * M_DEG_TO_RAD_F;
}

void BoatManualMode::manual()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	const float signed_thrust_setpoint = math::constrain(manual_control_setpoint.throttle, -1.f, 1.f);

	rover_steering_setpoint_s rover_steering_setpoint{};
	rover_steering_setpoint.timestamp = hrt_absolute_time();
	rover_steering_setpoint.normalized_steering_setpoint = manual_control_setpoint.roll;
	_rover_steering_setpoint_pub.publish(rover_steering_setpoint);

	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = signed_thrust_setpoint;
	rover_throttle_setpoint.throttle_body_y = NAN;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
}

void BoatManualMode::acro()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	const float signed_thrust_setpoint = math::constrain(manual_control_setpoint.throttle, -1.f, 1.f);

	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = signed_thrust_setpoint;
	rover_throttle_setpoint.throttle_body_y = NAN;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

	rover_rate_setpoint_s rover_rate_setpoint{};
	rover_rate_setpoint.timestamp = hrt_absolute_time();
	rover_rate_setpoint.yaw_rate_setpoint = yawRateFromStick(manual_control_setpoint.roll,
						signed_thrust_setpoint);
	_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
}

void BoatManualMode::stab()
{
	updateAttitude();

	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	const float signed_thrust_setpoint = math::constrain(manual_control_setpoint.throttle, -1.f, 1.f);

	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = signed_thrust_setpoint;
	rover_throttle_setpoint.throttle_body_y = NAN;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

	if (fabsf(manual_control_setpoint.roll) > FLT_EPSILON
	    || fabsf(rover_throttle_setpoint.throttle_body_x) < FLT_EPSILON) {
		_stab_yaw_setpoint = NAN;

		rover_rate_setpoint_s rover_rate_setpoint{};
		rover_rate_setpoint.timestamp = hrt_absolute_time();
		rover_rate_setpoint.yaw_rate_setpoint = yawRateFromStick(manual_control_setpoint.roll,
							signed_thrust_setpoint);
		_rover_rate_setpoint_pub.publish(rover_rate_setpoint);

		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = NAN;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

	} else {
		if (!PX4_ISFINITE(_stab_yaw_setpoint)) {
			_stab_yaw_setpoint = _vehicle_yaw;
		}

		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = _stab_yaw_setpoint;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
	}
}

void BoatManualMode::position()
{
	updateAttitude();
	updateLocalPosition();

	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	const float signed_thrust_setpoint = math::constrain(manual_control_setpoint.throttle, -1.f, 1.f);

	const float speed_setpoint = math::interpolate<float>(signed_thrust_setpoint,
				     -1.f, 1.f, -_param_boat_speed_lim.get(), _param_boat_speed_lim.get());

	if (fabsf(manual_control_setpoint.roll) > FLT_EPSILON || fabsf(speed_setpoint) < FLT_EPSILON) {
		_pos_ctl_course_direction = Vector2f(NAN, NAN);

		rover_speed_setpoint_s rover_speed_setpoint{};
		rover_speed_setpoint.timestamp = hrt_absolute_time();
		rover_speed_setpoint.speed_body_x = speed_setpoint;
		_rover_speed_setpoint_pub.publish(rover_speed_setpoint);

		rover_rate_setpoint_s rover_rate_setpoint{};
		rover_rate_setpoint.timestamp = hrt_absolute_time();
		rover_rate_setpoint.yaw_rate_setpoint = yawRateFromStick(manual_control_setpoint.roll,
							signed_thrust_setpoint);
		_rover_rate_setpoint_pub.publish(rover_rate_setpoint);

		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = NAN;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = NAN;
		rover_position_setpoint.position_ned[1] = NAN;
		rover_position_setpoint.start_ned[0] = NAN;
		rover_position_setpoint.start_ned[1] = NAN;
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.cruising_speed = NAN;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);

	} else {
		if (!_pos_ctl_course_direction.isAllFinite()) {
			_pos_ctl_course_direction = Vector2f(cosf(_vehicle_yaw), sinf(_vehicle_yaw));
			_pos_ctl_start_position_ned = _curr_pos_ned;
		}

		const Vector2f start_to_curr_pos = _curr_pos_ned - _pos_ctl_start_position_ned;
		const float vector_scaling = fabsf(start_to_curr_pos * _pos_ctl_course_direction) + _param_boat_lookahead.get();
		const Vector2f target_waypoint_ned = _pos_ctl_start_position_ned + matrix::sign(speed_setpoint) * vector_scaling *
						     _pos_ctl_course_direction;

		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = target_waypoint_ned(0);
		rover_position_setpoint.position_ned[1] = target_waypoint_ned(1);
		rover_position_setpoint.start_ned[0] = _pos_ctl_start_position_ned(0);
		rover_position_setpoint.start_ned[1] = _pos_ctl_start_position_ned(1);
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.cruising_speed = speed_setpoint;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);
	}
}

void BoatManualMode::reset()
{
	_stab_yaw_setpoint = NAN;
	_pos_ctl_course_direction = Vector2f(NAN, NAN);
	_pos_ctl_start_position_ned = Vector2f(NAN, NAN);
	_curr_pos_ned = Vector2f(NAN, NAN);
}

float BoatManualMode::yawRateFromStick(float roll, float throttle) const
{
	return matrix::sign(throttle) * _max_yaw_rate *
	       math::superexpo<float>(math::deadzone(roll, _param_boat_y_stick_dz.get()),
				      _param_boat_y_expo.get(), _param_boat_y_supexpo.get());
}

void BoatManualMode::updateAttitude()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = Quatf(vehicle_attitude.q);
		_vehicle_yaw = Eulerf(_vehicle_attitude_quaternion).psi();
	}
}

void BoatManualMode::updateLocalPosition()
{
	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
	}
}

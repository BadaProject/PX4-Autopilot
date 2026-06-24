/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "BoatPosControl.hpp"

BoatPosControl::BoatPosControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_speed_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	updateParams();
}

void BoatPosControl::updateParams()
{
	ModuleParams::updateParams();
	_acceptance_radius = _param_boat_acc_rad.get();
}

void BoatPosControl::updatePosControl()
{
	updateSubscriptions();

	const hrt_abstime timestamp = hrt_absolute_time();

	if (!_target_waypoint_ned.isAllFinite() || !_curr_pos_ned.isAllFinite()) {
		return;
	}

	const float distance_to_target = (_target_waypoint_ned - _curr_pos_ned).norm();

	if (distance_to_target <= _acceptance_radius && _arrival_speed <= FLT_EPSILON) {
		rover_speed_setpoint_s rover_speed_setpoint{};
		rover_speed_setpoint.timestamp = timestamp;
		rover_speed_setpoint.speed_body_x = 0.f;
		_rover_speed_setpoint_pub.publish(rover_speed_setpoint);

		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = timestamp;
		rover_attitude_setpoint.yaw_setpoint = _vehicle_yaw;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

		if (!_stopped && fabsf(_vehicle_speed) < FLT_EPSILON) {
			_stopped = true;
			_target_waypoint_ned = _curr_pos_ned;
		}

		if (_stopped && _updated_reset_counter != _reset_counter) {
			_target_waypoint_ned = _curr_pos_ned;
			_reset_counter = _updated_reset_counter;
		}

		return;
	}

	Vector2f start_ned = _start_ned.isAllFinite() ? _start_ned : _curr_pos_ned;
	const Vector2f line = _target_waypoint_ned - start_ned;
	const float line_len_sq = line.norm_squared();
	float bearing_setpoint = atan2f(_target_waypoint_ned(1) - _curr_pos_ned(1),
					_target_waypoint_ned(0) - _curr_pos_ned(0));

	if (line_len_sq > 1e-3f) {
		const float t = math::constrain((_curr_pos_ned - start_ned) * line / line_len_sq, 0.f, 1.f);
		const Vector2f projection = start_ned + t * line;
		const Vector2f direction = line.normalized();
		const Vector2f carrot = projection + _param_boat_lookahead.get() * direction;
		bearing_setpoint = atan2f(carrot(1) - _curr_pos_ned(1), carrot(0) - _curr_pos_ned(0));
	}

	const float speed_setpoint = math::constrain(PX4_ISFINITE(_cruising_speed) ? _cruising_speed :
				     _param_boat_speed_lim.get(), -_param_boat_speed_lim.get(), _param_boat_speed_lim.get());

	rover_speed_setpoint_s rover_speed_setpoint{};
	rover_speed_setpoint.timestamp = timestamp;
	rover_speed_setpoint.speed_body_x = speed_setpoint;
	_rover_speed_setpoint_pub.publish(rover_speed_setpoint);

	rover_attitude_setpoint_s rover_attitude_setpoint{};
	rover_attitude_setpoint.timestamp = timestamp;
	rover_attitude_setpoint.yaw_setpoint = speed_setpoint > -FLT_EPSILON ? bearing_setpoint :
					       matrix::wrap_pi(bearing_setpoint + M_PI_F);
	_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
}

void BoatPosControl::updateSubscriptions()
{
	if (_position_controller_status_sub.updated()) {
		position_controller_status_s position_controller_status{};
		_position_controller_status_sub.copy(&position_controller_status);
		_acceptance_radius = PX4_ISFINITE(position_controller_status.acceptance_radius) ?
				     position_controller_status.acceptance_radius : _param_boat_acc_rad.get();
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = Quatf(vehicle_attitude.q);
		_vehicle_yaw = Eulerf(_vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		_updated_reset_counter = vehicle_local_position.xy_reset_counter;
		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
		const Vector3f velocity_ned(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		const Vector3f velocity_body = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_ned);
		_vehicle_speed = velocity_body(0);
	}

	if (_rover_position_setpoint_sub.updated()) {
		rover_position_setpoint_s rover_position_setpoint{};
		_rover_position_setpoint_sub.copy(&rover_position_setpoint);
		_start_ned = Vector2f(rover_position_setpoint.start_ned[0], rover_position_setpoint.start_ned[1]);
		_start_ned = _start_ned.isAllFinite() ? _start_ned : _curr_pos_ned;
		_target_waypoint_ned = Vector2f(rover_position_setpoint.position_ned[0], rover_position_setpoint.position_ned[1]);
		_arrival_speed = PX4_ISFINITE(rover_position_setpoint.arrival_speed) ? rover_position_setpoint.arrival_speed : 0.f;
		_cruising_speed = PX4_ISFINITE(rover_position_setpoint.cruising_speed) ? rover_position_setpoint.cruising_speed :
				  _param_boat_speed_lim.get();
		_stopped = false;
	}
}

bool BoatPosControl::runSanityChecks()
{
	bool ret = true;

	if (_param_boat_speed_lim.get() < FLT_EPSILON) {
		ret = false;
		events::send<float>(events::ID("boat_pos_control_invalid_speed_lim"), events::Log::Error,
				     "Invalid configuration of parameter BOAT_SPEED_LIM", _param_boat_speed_lim.get());
	}

	if (_param_boat_lookahead.get() < FLT_EPSILON) {
		ret = false;
	}

	return ret;
}

void BoatPosControl::reset()
{
	_start_ned = Vector2f{NAN, NAN};
	_target_waypoint_ned = Vector2f{NAN, NAN};
	_arrival_speed = 0.f;
	_cruising_speed = _param_boat_speed_lim.get();
	_stopped = false;
}

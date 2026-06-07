/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "BoatAutoMode.hpp"

BoatAutoMode::BoatAutoMode(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_position_setpoint_pub.advertise();
	_position_controller_status_pub.advertise();
}

void BoatAutoMode::updateParams()
{
	ModuleParams::updateParams();
	_acceptance_radius = _param_boat_acc_rad.get();
}

void BoatAutoMode::autoControl()
{
	if (_position_setpoint_triplet_sub.updated()) {
		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position{};
			_vehicle_local_position_sub.copy(&vehicle_local_position);

			if (!_global_ned_proj_ref.isInitialized()
			    || (_global_ned_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.ref_timestamp)) {
				_global_ned_proj_ref.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon,
								   vehicle_local_position.ref_timestamp);
			}

			_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
		}

		updateWaypointsAndAcceptanceRadius();

		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = _curr_wp_ned(0);
		rover_position_setpoint.position_ned[1] = _curr_wp_ned(1);
		rover_position_setpoint.start_ned[0] = _prev_wp_ned(0);
		rover_position_setpoint.start_ned[1] = _prev_wp_ned(1);
		rover_position_setpoint.arrival_speed = (_next_wp_ned.isAllFinite()
				&& _curr_wp_type != position_setpoint_s::SETPOINT_TYPE_LAND
				&& _curr_wp_type != position_setpoint_s::SETPOINT_TYPE_IDLE) ? _cruising_speed : 0.f;
		rover_position_setpoint.cruising_speed = _cruising_speed;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);
	}
}

void BoatAutoMode::updateWaypointsAndAcceptanceRadius()
{
	position_setpoint_triplet_s position_setpoint_triplet{};
	_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);
	_curr_wp_type = position_setpoint_triplet.current.type;
	const bool previous_waypoint_valid = position_setpoint_triplet.previous.valid
					     && PX4_ISFINITE(position_setpoint_triplet.previous.lat)
					     && PX4_ISFINITE(position_setpoint_triplet.previous.lon);

	RoverControl::globalToLocalSetpointTriplet(_curr_wp_ned, _prev_wp_ned, _next_wp_ned, position_setpoint_triplet,
			_curr_pos_ned, _global_ned_proj_ref);

	if (!previous_waypoint_valid && _curr_wp_ned.isAllFinite() && _curr_pos_ned.isAllFinite()) {
		const Vector2f current_to_target = _curr_wp_ned - _curr_pos_ned;

		if (current_to_target.norm() > FLT_EPSILON && _param_boat_start_dist.get() > FLT_EPSILON) {
			_prev_wp_ned = _curr_pos_ned - _param_boat_start_dist.get() * current_to_target.normalized();

		} else {
			_prev_wp_ned = _curr_pos_ned;
		}
	}

	_cruising_speed = position_setpoint_triplet.current.cruising_speed > 0.f ?
			  math::constrain(position_setpoint_triplet.current.cruising_speed, 0.f, _param_boat_speed_lim.get()) :
			  _param_boat_speed_lim.get();

	_acceptance_radius = _param_boat_acc_rad.get();

	position_controller_status_s position_controller_status{};
	position_controller_status.acceptance_radius = _acceptance_radius;
	position_controller_status.timestamp = hrt_absolute_time();
	_position_controller_status_pub.publish(position_controller_status);
}

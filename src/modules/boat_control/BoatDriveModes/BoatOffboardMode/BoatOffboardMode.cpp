/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "BoatOffboardMode.hpp"

BoatOffboardMode::BoatOffboardMode(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_speed_setpoint_pub.advertise();
	_rover_position_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
}

void BoatOffboardMode::updateParams()
{
	ModuleParams::updateParams();
}

void BoatOffboardMode::offboardControl()
{
	offboard_control_mode_s offboard_control_mode{};
	_offboard_control_mode_sub.copy(&offboard_control_mode);

	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	if (offboard_control_mode.position) {
		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = trajectory_setpoint.position[0];
		rover_position_setpoint.position_ned[1] = trajectory_setpoint.position[1];
		rover_position_setpoint.start_ned[0] = NAN;
		rover_position_setpoint.start_ned[1] = NAN;
		rover_position_setpoint.cruising_speed = NAN;
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);

	} else if (offboard_control_mode.velocity) {
		const Vector2f velocity_ned(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);
		rover_speed_setpoint_s rover_speed_setpoint{};
		rover_speed_setpoint.timestamp = hrt_absolute_time();
		rover_speed_setpoint.speed_body_x = velocity_ned.norm();
		_rover_speed_setpoint_pub.publish(rover_speed_setpoint);

		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = atan2f(velocity_ned(1), velocity_ned(0));
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
	}
}

/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module_params.h>

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <math.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/topics/rover_position_setpoint.h>
#include <uORB/topics/rover_rate_setpoint.h>
#include <uORB/topics/rover_speed_setpoint.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;

class BoatManualMode : public ModuleParams
{
public:
	BoatManualMode(ModuleParams *parent);
	~BoatManualMode() = default;

	void manual();
	void acro();
	void stab();
	void position();
	void reset();

protected:
	void updateParams() override;

private:
	float yawRateFromStick(float roll, float throttle) const;
	void updateAttitude();
	void updateLocalPosition();

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	uORB::Publication<rover_throttle_setpoint_s> _rover_throttle_setpoint_pub{ORB_ID(rover_throttle_setpoint)};
	uORB::Publication<rover_steering_setpoint_s> _rover_steering_setpoint_pub{ORB_ID(rover_steering_setpoint)};
	uORB::Publication<rover_rate_setpoint_s> _rover_rate_setpoint_pub{ORB_ID(rover_rate_setpoint)};
	uORB::Publication<rover_attitude_setpoint_s> _rover_attitude_setpoint_pub{ORB_ID(rover_attitude_setpoint)};
	uORB::Publication<rover_speed_setpoint_s> _rover_speed_setpoint_pub{ORB_ID(rover_speed_setpoint)};
	uORB::Publication<rover_position_setpoint_s> _rover_position_setpoint_pub{ORB_ID(rover_position_setpoint)};

	Quatf _vehicle_attitude_quaternion{};
	Vector2f _pos_ctl_course_direction{NAN, NAN};
	Vector2f _pos_ctl_start_position_ned{NAN, NAN};
	Vector2f _curr_pos_ned{NAN, NAN};
	float _stab_yaw_setpoint{NAN};
	float _vehicle_yaw{NAN};
	float _max_yaw_rate{NAN};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BOAT_Y_RATE_LIM>) _param_boat_y_rate_lim,
		(ParamFloat<px4::params::BOAT_Y_STICK_DZ>) _param_boat_y_stick_dz,
		(ParamFloat<px4::params::BOAT_Y_EXPO>)     _param_boat_y_expo,
		(ParamFloat<px4::params::BOAT_Y_SUPEXPO>)  _param_boat_y_supexpo,
		(ParamFloat<px4::params::BOAT_LOOKAHD>)    _param_boat_lookahead,
		(ParamFloat<px4::params::BOAT_SPEED_LIM>)  _param_boat_speed_lim
	)
};

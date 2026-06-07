/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/events.h>
#include <px4_platform_common/module_params.h>

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <math.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rover_speed_setpoint.h>
#include <uORB/topics/rover_speed_status.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;

class BoatSpeedControl : public ModuleParams
{
public:
	BoatSpeedControl(ModuleParams *parent);
	~BoatSpeedControl() = default;

	void updateSpeedControl();
	bool runSanityChecks();
	void reset() {_speed_setpoint = NAN; _last_thrust = 0.f; _adjusted_speed_setpoint = 0.f;};

protected:
	void updateParams() override;

private:
	void updateSubscriptions();

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _rover_speed_setpoint_sub{ORB_ID(rover_speed_setpoint)};

	uORB::Publication<rover_throttle_setpoint_s> _rover_throttle_setpoint_pub{ORB_ID(rover_throttle_setpoint)};
	uORB::Publication<rover_speed_status_s> _rover_speed_status_pub{ORB_ID(rover_speed_status)};

	Quatf _vehicle_attitude_quaternion{};
	float _vehicle_speed{0.f};
	float _speed_setpoint{NAN};
	float _last_thrust{0.f};
	float _adjusted_speed_setpoint{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BOAT_SPEED_LIM>) _param_boat_speed_lim,
		(ParamFloat<px4::params::BOAT_THR_MAX>)   _param_boat_thr_max,
		(ParamFloat<px4::params::BOAT_THR_RATE>)  _param_boat_thr_rate,
		(ParamFloat<px4::params::BOAT_THR_FF>)    _param_boat_thr_ff,
		(ParamFloat<px4::params::BOAT_THR_SCALE>) _param_boat_thr_scale
	)
};

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
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/topics/rover_attitude_status.h>
#include <uORB/topics/rover_rate_setpoint.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>

using namespace matrix;

class BoatAttControl : public ModuleParams
{
public:
	BoatAttControl(ModuleParams *parent);
	~BoatAttControl() = default;

	void updateAttControl();
	bool runSanityChecks();
	void reset() {_yaw_setpoint = NAN; _yaw_rate_setpoint = NAN; _last_error = 0.f; _last_steering = 0.f; _timestamp = 0;};

protected:
	void updateParams() override;

private:
	void updateSubscriptions();
	float steeringFromYawError(float error, float dt);
	float normalizedSteering(float steering) const;

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _rover_attitude_setpoint_sub{ORB_ID(rover_attitude_setpoint)};
	uORB::Subscription _rover_rate_setpoint_sub{ORB_ID(rover_rate_setpoint)};

	uORB::Publication<rover_steering_setpoint_s> _rover_steering_setpoint_pub{ORB_ID(rover_steering_setpoint)};
	uORB::Publication<rover_attitude_status_s> _rover_attitude_status_pub{ORB_ID(rover_attitude_status)};

	hrt_abstime _timestamp{0};
	float _vehicle_yaw{0.f};
	float _vehicle_yaw_rate{0.f};
	float _yaw_setpoint{NAN};
	float _yaw_rate_setpoint{NAN};
	float _last_error{0.f};
	float _last_steering{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BOAT_STR_P>)      _param_boat_str_p,
		(ParamFloat<px4::params::BOAT_STR_D>)      _param_boat_str_d,
		(ParamFloat<px4::params::BOAT_STR_MAX>)    _param_boat_str_max,
		(ParamFloat<px4::params::BOAT_STR_RATE>)   _param_boat_str_rate,
		(ParamFloat<px4::params::BOAT_Y_RATE_LIM>) _param_boat_y_rate_lim
	)
};

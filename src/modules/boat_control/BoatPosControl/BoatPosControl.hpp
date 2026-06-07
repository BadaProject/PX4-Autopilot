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
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/topics/rover_position_setpoint.h>
#include <uORB/topics/rover_speed_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;

class BoatPosControl : public ModuleParams
{
public:
	BoatPosControl(ModuleParams *parent);
	~BoatPosControl() = default;

	void updatePosControl();
	bool runSanityChecks();
	void reset();

protected:
	void updateParams() override;

private:
	void updateSubscriptions();

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _rover_position_setpoint_sub{ORB_ID(rover_position_setpoint)};
	uORB::Subscription _position_controller_status_sub{ORB_ID(position_controller_status)};

	uORB::Publication<rover_speed_setpoint_s> _rover_speed_setpoint_pub{ORB_ID(rover_speed_setpoint)};
	uORB::Publication<rover_attitude_setpoint_s> _rover_attitude_setpoint_pub{ORB_ID(rover_attitude_setpoint)};

	Quatf _vehicle_attitude_quaternion{};
	Vector2f _curr_pos_ned{NAN, NAN};
	Vector2f _start_ned{NAN, NAN};
	Vector2f _target_waypoint_ned{NAN, NAN};
	float _arrival_speed{0.f};
	float _cruising_speed{NAN};
	float _vehicle_yaw{NAN};
	float _vehicle_speed{0.f};
	float _acceptance_radius{0.f};
	bool _stopped{false};
	uint8_t _reset_counter{0};
	uint8_t _updated_reset_counter{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BOAT_ACC_RAD>)   _param_boat_acc_rad,
		(ParamFloat<px4::params::BOAT_SPEED_LIM>) _param_boat_speed_lim,
		(ParamFloat<px4::params::BOAT_LOOKAHD>)   _param_boat_lookahead
	)
};

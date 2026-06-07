/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module_params.h>

#include <lib/rover_control/RoverControl.hpp>
#include <mathlib/mathlib.h>
#include <math.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/rover_position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;

class BoatAutoMode : public ModuleParams
{
public:
	BoatAutoMode(ModuleParams *parent);
	~BoatAutoMode() = default;

	void autoControl();

protected:
	void updateParams() override;

private:
	void updateWaypointsAndAcceptanceRadius();

	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};

	uORB::Publication<rover_position_setpoint_s> _rover_position_setpoint_pub{ORB_ID(rover_position_setpoint)};
	uORB::Publication<position_controller_status_s> _position_controller_status_pub{ORB_ID(position_controller_status)};

	MapProjection _global_ned_proj_ref{};
	Vector2f _curr_wp_ned{NAN, NAN};
	Vector2f _prev_wp_ned{NAN, NAN};
	Vector2f _next_wp_ned{NAN, NAN};
	Vector2f _curr_pos_ned{NAN, NAN};
	float _acceptance_radius{0.f};
	float _cruising_speed{0.f};
	int _curr_wp_type{position_setpoint_s::SETPOINT_TYPE_IDLE};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BOAT_ACC_RAD>)   _param_boat_acc_rad,
		(ParamFloat<px4::params::BOAT_SPEED_LIM>) _param_boat_speed_lim,
		(ParamFloat<px4::params::BOAT_START_DIST>) _param_boat_start_dist
	)
};

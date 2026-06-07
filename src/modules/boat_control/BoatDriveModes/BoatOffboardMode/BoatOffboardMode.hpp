/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module_params.h>

#include <matrix/matrix/math.hpp>
#include <math.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/topics/rover_position_setpoint.h>
#include <uORB/topics/rover_speed_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>

using namespace matrix;

class BoatOffboardMode : public ModuleParams
{
public:
	BoatOffboardMode(ModuleParams *parent);
	~BoatOffboardMode() = default;

	void offboardControl();

protected:
	void updateParams() override;

private:
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};

	uORB::Publication<rover_speed_setpoint_s> _rover_speed_setpoint_pub{ORB_ID(rover_speed_setpoint)};
	uORB::Publication<rover_position_setpoint_s> _rover_position_setpoint_pub{ORB_ID(rover_position_setpoint)};
	uORB::Publication<rover_attitude_setpoint_s> _rover_attitude_setpoint_pub{ORB_ID(rover_attitude_setpoint)};
};

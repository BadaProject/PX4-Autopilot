/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module_params.h>

#include <mathlib/mathlib.h>
#include <math.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_throttle_setpoint.h>

class BoatActControl : public ModuleParams
{
public:
	BoatActControl(ModuleParams *parent);
	~BoatActControl() = default;

	void updateActControl();
	void stopVehicle();

protected:
	void updateParams() override;

private:
	uORB::Subscription _rover_steering_setpoint_sub{ORB_ID(rover_steering_setpoint)};
	uORB::Subscription _rover_throttle_setpoint_sub{ORB_ID(rover_throttle_setpoint)};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};

	hrt_abstime _timestamp{0};
	float _throttle_setpoint{NAN};
	float _steering_setpoint{NAN};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev
	)
};

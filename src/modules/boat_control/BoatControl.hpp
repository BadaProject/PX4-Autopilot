/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

#include "BoatActControl/BoatActControl.hpp"
#include "BoatAttControl/BoatAttControl.hpp"
#include "BoatSpeedControl/BoatSpeedControl.hpp"
#include "BoatPosControl/BoatPosControl.hpp"
#include "BoatDriveModes/BoatAutoMode/BoatAutoMode.hpp"
#include "BoatDriveModes/BoatManualMode/BoatManualMode.hpp"
#include "BoatDriveModes/BoatOffboardMode/BoatOffboardMode.hpp"

class BoatControl : public ModuleBase<BoatControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	BoatControl();
	~BoatControl() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

protected:
	void updateParams() override;

private:
	void Run() override;
	void generateSetpoints();
	void updateControllers();
	void runSanityChecks();
	void reset();

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	vehicle_control_mode_s _vehicle_control_mode{};

	BoatActControl _boat_act_control{this};
	BoatAttControl _boat_att_control{this};
	BoatSpeedControl _boat_speed_control{this};
	BoatPosControl _boat_pos_control{this};
	BoatAutoMode _auto_mode{this};
	BoatManualMode _manual_mode{this};
	BoatOffboardMode _offboard_mode{this};

	bool _sanity_checks_passed{true};
	bool _was_armed{false};
};

/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <version/version.h>
#include <parameters/param.h>
#include <px4_platform_common/printload.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>


typedef struct _PlcToVcc
{
    uint16_t mr_mtr_sta; // right main motor status
    uint16_t mr_flt_msg_err1; // right main motor fault message error 1
    uint16_t mr_flt_msg_err2; // right main motor fault message error 2
    uint16_t mr_flt_msg_warn1; // right main motor fault message warning 1
    uint16_t mr_flt_msg_warn2; // right main motor fault message warning 2
    uint16_t mr_mtr_curr_real; // right main motor current real
    uint16_t mr_temp; // right main motor temperature
    uint16_t mr_mtr_rpm_real; // right main motor RPM real
    uint16_t mr_mtr_rot_real; // right main motor rotation real
    uint16_t rsv3509;

    uint16_t ml_mtr_sta; // left main motor status
    uint16_t ml_flt_msg_err1; // left main motor fault message error 1
    uint16_t ml_flt_msg_err2; // left main motor fault message error 2
    uint16_t ml_flt_msg_warn1; // left main motor fault message warning 1
    uint16_t ml_flt_msg_warn2; // left main motor fault message warning 2
    uint16_t ml_mtr_curr_real; // left main motor current real
    uint16_t ml_temp; // left main motor temperature
    uint16_t ml_mtr_rpm_real; // left main motor RPM real
    uint16_t ml_mtr_rot_real; // left main motor rotation real
    uint16_t rsv3519;

    uint16_t br_mtr_sta; // right bow motor status
    uint16_t br_flt_msg; // right bow motor fault message
    uint16_t br_mtr_curr_real; // right bow motor current real
    uint16_t br_temp; // right bow motor temperature
    uint16_t br_mtr_rpm; // right bow motor RPM
    uint16_t br_mtr_rot_sta; // right bow motor rotation status
    uint16_t rsv3526;
    uint16_t rsv3527;
    uint16_t rsv3528;
    uint16_t rsv3529;

    uint16_t bl_mtr_sta; // left bow motor status
    uint16_t bl_flt_msg; // left bow motor fault message
    uint16_t bl_mtr_curr_real; // left bow motor current real
    uint16_t bl_temp; // left bow motor temperature
    uint16_t bl_mtr_rpm; // left bow motor RPM
    uint16_t bl_mtr_rot_sta; // left bow motor rotation status
    uint16_t rsv3536;
    uint16_t rsv3537;
    uint16_t rsv3538;
    uint16_t rsv3539;

    uint16_t sr_str_rpm; // right steering servo RPM
    int16_t sr_str_ang; // right steering servo angle
    uint16_t sl_str_rpm; // left steering servo RPM
    int16_t sl_str_ang; // left steering servo angle
    uint16_t batt400vdc; // main motor battery 400V
    uint16_t batt24vdc_1; // ANC PC & PLC
    uint16_t batt24vdc_2; // bilge pump, always-on power
} PlcToVcc;


typedef struct _VccToPlc
{
    uint16_t mr_mtr_run; // right main motor running
    uint16_t mr_mtr_rpm; // right main motor RPM
    uint16_t mr_mtr_trq; // right main motor torque
    uint16_t rsv3003;
    uint16_t rsv3004;
    uint16_t rsv3005;
    uint16_t rsv3006;
    uint16_t rsv3007;
    uint16_t rsv3008;
    uint16_t rsv3009;

    uint16_t ml_mtr_run; // left main motor running
    uint16_t ml_mtr_rpm; // left main motor RPM
    uint16_t ml_mtr_trq; // left main motor torque
    uint16_t rsv3013;
    uint16_t rsv3014;
    uint16_t rsv3015;
    uint16_t rsv3016;
    uint16_t rsv3017;
    uint16_t rsv3018;
    uint16_t rsv3019;

    uint16_t br_mtr_rot; // right bow motor rotation
    uint16_t br_mtr_thr_per; // right bow motor thrust percentage
    uint16_t spare3022;
    uint16_t spare3023;
    uint16_t spare3024;
    uint16_t spare3025;
    uint16_t spare3026;
    uint16_t spare3027;
    uint16_t spare3028;
    uint16_t spare3029;

    uint16_t bl_mtr_rot; // left bow motor rotation
    uint16_t bl_mtr_thr_per; // left bow motor thrust percentage
    uint16_t spare3032;
    uint16_t spare3033;
    uint16_t spare3034;
    uint16_t spare3035;
    uint16_t spare3036;
    uint16_t spare3037;
    uint16_t spare3038;
    uint16_t spare3039;

    uint16_t sr_str_srv_on; // right steering servo on/off
    uint16_t st_str_srv_home; // right steering servo home
    uint16_t sr_str_rpm; // right steering servo RPM
    uint16_t sr_str_ang; // right steering servo angle
    uint16_t sr_rfrg; // right steering servo refrigerator
    uint16_t spare3045;
    uint16_t spare3046;
    uint16_t spare3047;
    uint16_t spare3048;
    uint16_t spare3049;

    uint16_t sl_str_srv_on; // left steering servo on/off
    uint16_t sl_str_srv_home; // left steering servo home
    uint16_t sl_str_rpm; // left steering servo RPM
    uint16_t sl_str_ang; // left steering servo angle
    uint16_t sl_rfrg; // left steering servo refrigerator
    uint16_t spare3055;
    uint16_t spare3056;
    uint16_t spare3057;
    uint16_t spare3058;
    uint16_t spare3059;

    uint16_t emer_stop; // emergency stop
    uint16_t nav_mode; // navigation mode
    uint16_t nav_light1; // navigation light 1
    uint16_t nav_light2; // navigation light 2
    uint16_t br_s1; // right bilge pump s1
    uint16_t br_s2; // right bilge pump s2
    uint16_t br_s3; // right bilge pump s3
    uint16_t br_s4; // right bilge pump s4
    uint16_t pr_s1; // right fan s1
    uint16_t pr_s2; // right fan s2

    uint16_t bl_s1; // left bilge pump s1
    uint16_t bl_s2; // left bilge pump s2
    uint16_t bl_s3; // left bilge pump s3
    uint16_t bl_s4; // left bilge pump s4
    uint16_t pl_s1; // left fan s1
    uint16_t pl_s2; // left fan s2
} VccToPlc;

extern "C" __EXPORT int plclink_main(int argc, char *argv[]);

using namespace time_literals;

static constexpr hrt_abstime TRY_SUBSCRIBE_INTERVAL{20_ms};	// interval in microseconds at which we try to subscribe to a topic
// if we haven't succeeded before

namespace px4
{
namespace plclink
{

static constexpr uint8_t MSG_ID_INVALID = UINT8_MAX;


class Plclink : public ModuleBase<Plclink>, public ModuleParams
{
public:


	Plclink(LogWriter::Backend backend, size_t buffer_size, uint32_t log_interval, const char *poll_topic_name,
	       LogMode log_mode, bool log_name_timestamp, float rate_factor);

	~Plclink();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Logger *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/**
	 * request the logger thread to stop (this method does not block).
	 * @return true if the logger is stopped, false if (still) running
	 */
	static bool request_stop_static();


	/**
	 * @brief Updates and checks for updated uORB parameters.
	 */
	void update_params();

};

} //namespace logger
} //namespace px4

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

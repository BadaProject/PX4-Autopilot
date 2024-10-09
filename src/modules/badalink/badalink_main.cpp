/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file sensors.cpp
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/actuator_outputs.h>


using namespace time_literals;

class BadaLink : public ModuleBase<BadaLink>, public ModuleParams, public px4::ScheduledWorkItem
{
	public:
		BadaLink();
		~BadaLink() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	private:
		void Run() override;
		uORB::SubscriptionInterval _acturator_update_sub{ORB_ID(parameter_update), 1_s};

		uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};				/**< notification of parameter updates */
		uORB::SubscriptionCallbackWorkItem _adc_report_sub{this, ORB_ID(adc_report)};

		perf_counter_t	_loop_perf;			/**< loop performance counter */

		/**
		 * Check for changes in parameters.
		 */
		void 		parameter_update_poll(bool forced = false);

		/**
		 * Poll the ADC and update readings to suit.
		 *
		 * @param raw			Combined sensor data structure into which
		 *				data should be returned.
		*/
};

BadaLink::BadaLink() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME))
{
	updateParams();
}

BadaLink::~BadaLink()
{
	ScheduleClear();
}

void
BadaLink::parameter_update_poll(bool forced)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || forced) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}


void
BadaLink::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check parameters for updates */
	parameter_update_poll();

	perf_end(_loop_perf);
}

int
BadaLink::task_spawn(int argc, char *argv[])
{
	BadaLink *instance = new BadaLink();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool
BadaLink::init()
{
	return true; // return _adc_report_sub.registerCallback();
}

int BadaLink::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BadaLink::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

The provided functionality includes:
- Read the output from the ADC driver (via ioctl interface) and publish `battery_status`.


### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("badalink", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int badalink_main(int argc, char *argv[])
{
	return BadaLink::main(argc, argv);
}

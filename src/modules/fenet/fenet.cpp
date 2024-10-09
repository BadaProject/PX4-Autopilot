/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "fenet.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/airspeed.h>


int Fenet::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

void Fenet::find_broadcast_address()
{

}

void Fenet::init_udp()
{
	PX4_DEBUG("Setting up UDP with port %hu", _network_port);

	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_network_port);

	if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed: %s", strerror(errno));
		return;
	}

	if (bind(_socket_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
		PX4_WARN("bind failed: %s", strerror(errno));
		return;
	}

	/* set default target address, but not for onboard mode (will be set on first received packet) */
	if (!_src_addr_initialized) {
		_src_addr.sin_family = AF_INET;
		inet_aton("192.168.1.100", &_src_addr.sin_addr);
	}

	_src_addr.sin_port = htons(5000);
}

int Fenet::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int Fenet::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("fenet",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				       2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Fenet *Fenet::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	Fenet *instance = new Fenet(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Fenet::Fenet(int example_param, bool example_flag)
	: ModuleParams(nullptr),
	_receiver(this)
{
	init_udp();

}

void Fenet::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(airspeed));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);
	_receiver.start();

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here
			char msg[] = "Hello Pixhawk";
			sendto(_socket_fd, msg, sizeof(msg), 0, (struct sockaddr *)&_src_addr, sizeof(_src_addr));

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			PX4_ERR("poll error %lf", (double)sensor_combined.accelerometer_m_s2[0]);
			// TODO: do something with the data...

			sendto(_socket_fd, _buf, _buf_fill, 0, (struct sockaddr *)&_src_addr, sizeof(_src_addr));

		}

		parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);
}

void Fenet::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int Fenet::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int fenet_main(int argc, char *argv[])
{
	return Fenet::main(argc, argv);
}

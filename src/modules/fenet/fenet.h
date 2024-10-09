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

#pragma once

#include <nuttx/fs/fs.h>
#include <net/if.h>
#include <netinet/in.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netutils/netlib.h>

#include "fenet_receiver.h"

# define DEFAULT_REMOTE_PORT_UDP 14550 ///< GCS port per MAVLink spec

#define MAX_DATA_RATE                  10000000        ///< max data rate in bytes/s
#define MAIN_LOOP_DELAY                10000           ///< 100 Hz @ 1000 bytes/s data rate

// static pthread_mutex_t fenet_module_mutex = PTHREAD_MUTEX_INITIALIZER;

using namespace time_literals;

extern "C" __EXPORT int fenet_main(int argc, char *argv[]);


class Fenet : public ModuleBase<Fenet>, public ModuleParams
{
public:
	enum BROADCAST_MODE {
		BROADCAST_MODE_OFF = 0,
		BROADCAST_MODE_ON,
		BROADCAST_MODE_MULTICAST
	};

	Fenet(int example_param, bool example_flag);

	virtual ~Fenet() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Fenet *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	BROADCAST_MODE		_mav_broadcast {BROADCAST_MODE_OFF};

	sockaddr_in		_myaddr {};
	sockaddr_in		_src_addr {};
	sockaddr_in		_bcast_addr {};

	bool			_src_addr_initialized{false};
	bool			_broadcast_address_found{false};
	bool			_broadcast_address_not_found_warned{false};
	bool			_broadcast_failed_warned{false};

	unsigned short		_network_port{14556};
	unsigned short		_remote_port{DEFAULT_REMOTE_PORT_UDP};


	void find_broadcast_address();
	void init_udp();

	unsigned short		get_network_port() { return _network_port; }
	unsigned short		get_remote_port() { return _remote_port; }
	const in_addr		query_netmask_addr(const int socket_fd, const ifreq &ifreq);
	const in_addr		compute_broadcast_addr(const in_addr &host_addr, const in_addr &netmask_addr);
	struct sockaddr_in 	&get_client_source_address() { return _src_addr; }
	void			set_client_source_initialized() { _src_addr_initialized = true; }
	bool			get_client_source_initialized() { return _src_addr_initialized; }

	uint8_t			_buf[1000] {};
	unsigned		_buf_fill{0};

	bool			_tx_buffer_low{false};

	int			_socket_fd{-1};
private:
	FenetReceiver		_receiver;
	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

};


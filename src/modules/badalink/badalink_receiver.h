
/**
 * @file badalink_receiver.h
 *
 * badalink receiver thread that converts the received BadaLink messages to the appropriate
 * uORB topic publications, to decouple the uORB message and BadaLink message.
 *
 * @author Jeyong Shin <jeyong@subak.io>
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>


class BadaLink;

class BadaLinkReceiver : public ModuleParams
{
	public:
		BadaLinkReceiver(BadaLink &parent);
		~BadaLinkReceiver() override;

		void start();
		void stop();

	private:
		static void *start_trampoline(void *context);
		void run();

		void handle_message(uint8_t *msg);

		px4::atomic_bool _should_exit{false};
		pthread_t	_thread {};

		BadaLink &_badalink;

		uORB::Subscription	_vehicle_global_position_sub{ORB_ID(vehicle_global_position)};

	// DEFINE_PARAMETERS(
	// 	(ParamFloat<px4::params::BAT_CRIT_THR>)     _param_bat_crit_thr,
	// 	(ParamFloat<px4::params::BAT_EMERGEN_THR>)  _param_bat_emergen_thr,
	// 	(ParamFloat<px4::params::BAT_LOW_THR>)      _param_bat_low_thr
	// );

	// Disallow copy construction and move assignment.
	BadaLinkReceiver(const MavlinkReceiver &) = delete;
	BadaLinkReceiver operator=(const BadaLinkReceiver &) = delete;

};


/**
 * @file badalink_receiver.cpp
 * BadaLink protocol message receive and dispatch
 *
 * @author Jeyong Shin <jeyong@subak.io>
 */

#include <math.h>
#include <poll.h>

#ifdef CONFIG_NET
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#ifndef __PX4_POSIX
#include <termios.h>
#endif

#include "badalink_main.h"
#include "badalink_receiver.h"


#ifdef CONFIG_NET
#define MAVLINK_RECEIVER_NET_ADDED_STACK 1360
#else
#define MAVLINK_RECEIVER_NET_ADDED_STACK 0
#endif

BadaLinkReceiver::~BadaLinkReceiver()
{

}


BadaLinkReceiver::BadaLinkReceiver(BadaLink &parent) :
	ModuleParams(nullptr),
	_badalink(parent)
{
}



void
BadaLinkReceiver::handle_message(uint8_t *msg)
{

}


void
BadaLinkReceiver::run()
{
	/* set thread name */
	{
		char thread_name[17];
		snprintf(thread_name, sizeof(thread_name), "mavlink_rcv_if%d", _mavlink.get_instance_id());
		px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
	}

	#if defined(CONFIG_NET)
		/* 1500 is the Wifi MTU, so we make sure to fit a full packet */
		uint8_t buf[1000];
	#else
		uint8_t buf[1000];

	struct pollfd fds[1] = {};

	struct sockaddr_in srcaddr = {};
	socklen_t addrlen = sizeof(srcaddr);

	fds[0].fd = _badalink.get_socket_fd();
	fds[0].events = POLLIN;

	ssize_t nread = 0;
	hrt_abstime last_send_update = 0;

	while(!_badalink.should_exit()){
		int ret = poll(&fds[0], 1, timeout);

		if(ret > 0 ){
			if(fds[0].revents & POLLIN) {
				nread = recvfrom(_badalink.get_socket_fd(), buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
			}
			struct sockaddr_in &srcaddr_last = _mavlink.get_client_source_address();

			int localhost = (127 << 24) + 1;

			if(nread > 0) {
				
			}
		}
	}
}

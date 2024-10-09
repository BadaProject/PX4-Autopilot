#include <math.h>
#include <poll.h>

#ifdef CONFIG_NET
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#include <lib/drivers/device/Device.hpp> // For DeviceId union

#ifdef CONFIG_NET
#define MAVLINK_RECEIVER_NET_ADDED_STACK 1360
#else
#define MAVLINK_RECEIVER_NET_ADDED_STACK 0
#endif

FenetReceiver::FenetReceiver(Fenet *parent) :
    ModuleParams(nullptr),
    _parent(parent)
{
}

FenetReceiver::~FenetReceiver()
{
    stop();
}
void FenetReceiver::start()
{
    pthread_attr_t receiveloop_attr;
    pthread_attr_init(&receiveloop_attr);

    struct sched_param param;
    (void)pthread_attr_getschedparam(&receiveloop_attr, &param);
    param.sched_priority = SCHED_PRIORITY_MAX - 80;
    (void)pthread_attr_setschedparam(&receiveloop_attr, &param);

    pthread_attr_setstacksize(&receiveloop_attr, PX4_STACK_ADJUSTED(sizeof(FenetReceiver) + 2840 + MAVLINK_RECEIVER_NET_ADDED_STACK));
    pthread_create(&_thread, &receiveloop_attr, FenetReceiver::start_trampoline, this);

    pthread_attr_destroy(&receiveloop_attr);
}

void *FenetReceiver::start_trampoline(void *context)
{
	FenetReceiver *self = reinterpret_cast<FenetReceiver *>(context);
	self->run();
	return nullptr;
}

void FenetReceiver::run()
{
    uint8_t buf[1000];

    struct pollfd fds[1] = {};

    struct sockaddr_in srcaddr = {};
    socklen_t addrlen = sizeof(srcaddr);
    fds[0].fd = 1;
    fds[0].events = POLLIN;

    ssize_t nread = 0;
    hrt_abstime last_send_update = 0;

    while(!_parent->should_exit()){
        int ret = poll(&fds[0], 1, timeout);

        if(ret > 0){
            if(fds[0].revents & POLLIN){
                nread = recvfrom(_parent->_socket_fd, buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
            }
        }
    }
}
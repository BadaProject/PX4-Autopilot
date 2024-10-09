/**
 * @file fenet_receiver.h
 *
 * fenet receiver thread that converts the received MAVLink messages to the appropriate
 * uORB topic publications, to decouple the uORB message and MAVLink message.
 *
 * @author Jeyong Shin <jeyong@subak.io>
 */

#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

class Fenet;

class FenetReceiver : public ModuleParams 
{
    public:
        FenetReceiver(Fenet *parent);
        ~FenetReceiver();

        int start();
        void stop();

        void print_status();

    private:
        void run();
        static void *start_trampoline(void *context);
    
        px4::atomic_bool 	_should_exit{false};
        pthread_t		_thread {};

        Fenet *_parent;
        // Disallow copy construction and move assignment.
        MavlinkReceiver(const MavlinkReceiver &) = delete;
        MavlinkReceiver operator=(const MavlinkReceiver &) = delete;
}
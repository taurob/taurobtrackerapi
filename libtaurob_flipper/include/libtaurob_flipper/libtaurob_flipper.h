#ifndef LIBTAUROB_FLIPPER_H_
#define LIBTAUROB_FLIPPER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/tuple/tuple.hpp>

#include <libtaurob_tools/SocketUDPBinary.h>
#include <libtaurob_arm/libtaurob_arm.h>

#define FLIPPER_SETPOINT_FACTOR 1.2f

typedef boost::posix_time::ptime tTime;
typedef boost::posix_time::time_duration tTimeDuration;

using namespace boost::tuples;
using namespace boost;

// note that this is (or should be) IDENTICAL to Arm_config)
// it is included so that modules using this lib don't have to 
// explicitly reference libtaurob_arm.
typedef struct Flipper_config_
{
	std::vector<std::string> joint_names;
	std::vector<std::string> joint_ips;
	std::vector<int> joint_ports;
	std::vector<int> joint_channels;
	
	void Add_segment(std::string name, std::string ip, int port, int channel)
	{
		joint_names.push_back(name);
		joint_ips.push_back(ip);
		joint_ports.push_back(port);
		joint_channels.push_back(channel);
	}
} Flipper_config;


class Flipper : IReceiveCallbackRelay, IAbstract_taurob_segment
{
	public:	
		static const int MAX_ANGLE_VALUE = 4095;
		static const int UI_SERVER_BASE_PORT = 9051;

		Flipper(Flipper_config configuration, bool control_enabled_initially);
		~Flipper();
		
		// IAbstract_taurob_flipper		
		void Run();
		void Stop();		
		bool Is_uptodate(); 	// needed to decide whether or not get values are current
		bool Watchdog_ok(); 	// reports if a watchdog timeout has occurred		   
		void Feed_watchdog();
		void Set_on_received_callback(void (*callback)(int segment_nr));
		void Set_watchdog_enabled(bool state);
		void Set_pause_sending(bool pause);		
		int Get_segment_count();		
		std::string Get_segment_name(int segment);
		float Get_position(int segment);
		float Get_current(int segment);
		float Get_temperature(int segment);
		void Set_position(int segment, float pos);
		
		// class-specific
		unsigned char Get_error_code(int segment);
		unsigned char Get_bitfield(int segment);
	
		void On_segment_receive(int segment_nr);
		void On_friction_clutch_slipped(int segment_nr);
	
	private:
		std::vector<Arm_segment*> segments;
		void (*on_receive_callback)(int segment_nr);
};

#endif

/**************************************************************************//**
 *
 * @file libtaurob_arm.h
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Library for communication with taurob arm, header file
 *
 *
 *  Copyright (c) 2016 taurob GmbH. All rights reserved.
 *  Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ******************************************************************************/
 
#ifndef LIBTAUROB_ARM_H_
#define LIBTAUROB_ARM_H_

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

#include "Arm_frames.h"

typedef boost::posix_time::ptime tTime;
typedef boost::posix_time::time_duration tTimeDuration;

using namespace boost;


typedef struct Arm_config_
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
} Arm_config;

class IReceiveCallbackRelay
{
	public:
		virtual void On_segment_receive(int segment_nr) = 0;
		virtual void On_friction_clutch_slipped(int segment_nr) = 0;
};

class Arm_segment : IUdpReceiverBinary
{
	public:
		static const int DEFAULT_MAX_INPUT_VALUE = 4095;
		static const int DEFAULT_UI_SERVER_BASE_PORT = 9041;
		static const int MAX_TIME_WITHOUT_DRIVE_CMD_MS = 500; 	// ms
		static const int INVALID_ANGLE_VALUE = 5000;

		Arm_segment(std::string name, 
					std::string host_ip, 
					int host_port, 
					int segment_nr, 
					bool start_sending_initially,
					int max_input_value, 
					int ui_server_base_port);
		Arm_segment(std::string name, 
					std::string host_ip, 
					int host_port, 
					int segment_nr, 
					bool start_sending_initially);
		~Arm_segment();
		
		void On_data_received(unsigned char* data, int len, char* from_remote_ip, int from_remote_port, int from_local_port);
		
		void Run();
		void Stop();
		void Feed_watchdog();
		void Set_watchdog_enabled(bool state);
		void Set_on_received_callback(IReceiveCallbackRelay* relay);		
		void Set_pause_sending(bool pause);
		
		bool Is_uptodate();
		bool Watchdog_ok();
		void Reset_friction_clutch();
		void Reset_connection();
		
		std::string Get_name();
		float Get_position();
		float Get_current();
		float Get_temperature();
		unsigned char Get_error_code();
		unsigned char Get_bitfield();
		
		void Set_position(float pos);

		void Print_timestamp_debug();
		
	private:
		std::string segment_name;
		std::string host_ip;
		int host_port;
		int segment_nr;
		int max_input_value;
		int ui_server_base_port;
		int reset_friction_clutch_counter;

		static const int WATCHDOG_MAX_TIME = 400;	// ms
		static const int SEGMENT_STATIC_TOLERANCE = 1;  // degrees * (4096/360)
		static const int SEGMENT_STATIC_COUNTER_MAX = 20; 	// should equal appox. 800ms if we send every 80ms
		static const int MAX_DRIVE_TIME = 40000; 	// ms -- must be so long because e.g. argos flipper do take long to do 180deg.
		static const int MAX_BLIND_TIME = 400; 		// ms
		static const int RESET_ECU_DURATION = 400; 	// ms
		
		SocketUDPBinary* ecu_socket;
		SocketUDPBinary* ui_socket;

		bool receive_pause;
		bool pause_sending;
		bool reset_ecu;
		tTime reset_ecu_timestamp;
		
		IReceiveCallbackRelay* relay;
		
		tTime latest_watchdog_time;
		bool watchdog_enabled;
		tTime latest_rx_frame_time;
		tTime latest_drive_command_time;
		int segment_static_counter;
		
		bool first_frame_received;

		void Sending_thread();
		thread sending_thread;
		bool sending_thread_running;
		
		void Init_frames(); 	// to initialize internal frames to safe values
		void Check_if_allowed_to_drive(); 	// decides if motor enable bit is set or not
		void Set_motor_enable(bool enable);
					
		Arm_command_frame current_set_values;
		Arm_status_frame current_get_values;
		mutex current_set_values_locker;
		mutex current_get_values_locker;
		int current_tx_seqno;
};

class Arm : IReceiveCallbackRelay
{
	public:	
		Arm(Arm_config configuration, bool control_enabled_initially);
		~Arm();
		
		void Run();
		void Stop();
		   
		void Feed_watchdog();
		void Set_on_received_callback(void (*callback)(int segment_nr));
		void Set_on_friction_clutch_slipped_callback(void (*callback)(int segment_nr));
		void Set_watchdog_enabled(bool state);
		void Set_pause_sending(bool pause);
		
		bool Is_uptodate();
		bool Watchdog_ok();
		
		int Get_segment_count();
		//tTime Get_last_frame_timestamp();
		
		std::string Get_segment_name(int segment);
		float Get_position(int segment);
		float Get_current(int segment);
		float Get_temperature(int segment);
		unsigned char Get_error_code(int segment);
		unsigned char Get_bitfield(int segment);
	
		void Set_position(int segment, float pos);
		void Reset_friction_clutch(); 	// resets friction clutch for entire arm, because we don't want just one segment stopping
		
		// IReceiveCallbackRelay members		
		void On_segment_receive(int segment_nr);
		void On_friction_clutch_slipped(int segment_nr);
	
	private:
		static const int MIN_UPTODATE_WARNING_INTERVAL = 3000; // milliseconds

		tTime last_notuptodate_notification; 	// if a segment is not uptodate, the user should be notified but not spammed down. so use this timestamp to show a warning every x sec.

		bool config_successful;
		std::vector<Arm_segment*> segments;
		void (*on_receive_callback)(int segment_nr);
		void (*on_friction_clutch_slipped_callback)(int segment_nr);
};

#endif

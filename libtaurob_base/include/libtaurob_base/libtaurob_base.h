/**************************************************************************//**
 *
 * @file libtaurob_base.h
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Library for communication with taurob base ECU, header file
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
 
#ifndef LIBTAUROB_BASE_H_
#define LIBTAUROB_BASE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>


#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/tuple/tuple.hpp>

#include <libtaurob_tools/Debug.h>
#include <libtaurob_tools/SocketUDP.h>
#include "Base_frames.h"

#define null 0

typedef boost::posix_time::ptime tTime;
typedef boost::posix_time::time_duration tTimeDuration;

using namespace boost::tuples;
using namespace boost;

class Taurob_base : public IUdpReceiver
{
	public:
		static const float WHEELS_PLUS_TRACKS_DIAMETER = 265; 	// mm
		static const int MAX_TIME_WITHOUT_DRIVE_CMD_MS = 500; 	// ms
		
		Taurob_base(std::string host_ip, int host_port, int protocol_version, bool control_enabled_initially);
		~Taurob_base();

		void Run();
		void Stop();

		// communication
		virtual void On_string_received(std::string msg_data, char* from_remote_ip, int from_remote_port, int from_local_port);
		void Feed_watchdog();
		void Set_on_received_callback(void (*callback)());
		void Set_on_error_callback(void (*callback)(int));
		void Set_watchdog_enabled(bool state);
		void Set_pause_sending(bool pause);
		
		// calibration
		void Tilt_calibrate_zero();
		void Start_compass_calibration();
		void Stop_compass_calibration();
		
		// setters
		void Set_drive_command(tuple<float, float> drive_command);
		void Set_gripper_angle(float degrees);
		void Set_light(bool state);
		void Set_bluelight(bool state);
		
		// do not use
		void Set_manipulator_base_angle(float degrees);
		void Set_overturn_angles(tuple<float, float, float, float> angles); 	// tuple order: front, back, left, right
		void Set_overturn_thresholds(tuple<float, float> thresholds);
		
		// getters
		tuple<float, float> Get_wheel_speeds(); 				// U/min at wheels (Tuple order: left, right)
		tuple<float, float> Get_motor_currents_raw(); 				// raw value, needs multiplication by a factor to become Ampere (Tuple order: left, right)
		tuple<float, float> Get_motor_distance(); 				// Meters, do not use
		float Get_gripper_angle(); 									// degrees
		double Get_average_total_current(); 	// Ampere
		float Get_manipulator_base_angle(); 						// degrees, deprecated
		float Get_supply_voltage(); 								// Volts
		tuple<float, float, float, float> Get_temperatures(); 	// degrees Celsius
		tuple<float, float, float> Get_orientation(); 			// degrees, tuple order: pitch (x), yaw (y), roll (z)
		tuple<float, float, float> Get_gyro_data();
		tuple<float, float, float> Get_acc_data();
		tuple<float, float, float> Get_mag_data();
		unsigned char Get_error_code();
		unsigned char Get_AUX_bits();
		tTime Get_last_frame_timestamp();

	private:
		static const double gear_ratio = 30.0;
		static const double wheel_diameter = 0.303;
		static const double track_width = 0.55;
		static const double turning_geometry_factor = 1.476; // by experimentation
		static const int FLIPPER_CENTER_POS = 6000; // internal representation of 0 degrees
		static const double FLIPPER_DEGREES_TO_PULSES_FACTOR = 50; // factor to convert an angle in degrees to hall signal pulses
		static const int FLIPPER_CALIBRATION_MODE_OFFSET = 30000;
		static const uint INVALID_ANGLE_VALUE = 5000;
		static const uint UI_SERVER_PORT = 9002;
		static const int CURRENT_AVERAGE_ELEMENTS = 3;
		static const double MAX_TOTAL_MOTOR_CURRENT = 25.0;

		bool motor_fault;
		double avg_total_motor_current;
		uint backup_gripper_pos;
	
		std::string ECU_host_ip;
		int ECU_host_port;
		int protocol_version;

		static const int WATCHDOG_MAX_TIME = 150;	//ms
		SocketUDP* ecu_socket;
		SocketUDP* ui_socket;
		
		void (*on_receive_callback)();
		void (*on_error_callback)(int);
		
		std::string latest_command_frame;
		tTime latest_watchdog_time;
		bool watchdog_enabled;
		tTime latest_rx_frame_time;
		tTime latest_drive_command_time;
		
		bool pause_sending; 	// reflects if user wants to pause sending
		bool receive_pause; 	// reflects if we didn't receive frames for some time
		bool first_frame_sent;

		void Sending_thread();
		thread sending_thread;
		bool sending_thread_running;
		
		void Init_frames(); 	// to initialize internal frames to safe values
		void Check_for_errors();
		bool Watchdog_ok();
					
		Base_command_frame current_set_values;
		Base_status_frame current_get_values;
		mutex current_set_values_locker;
		mutex current_get_values_locker;
		int current_tx_seqno;
};

#endif /* LIBTAUROB_TRACKER_BASE_H_ */

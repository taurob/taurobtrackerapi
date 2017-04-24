/*
 * Copyright (c) 2017 taurob GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
#ifndef LIBTAUROB_BASE_H_
#define LIBTAUROB_BASE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>

#include <libtaurob_tools/SocketUDP.h>
#include "Base_frames.h"

using namespace boost;

typedef struct _DRIVETRAIN_STATUS
{
   float distance_travelled_forward;  //!< in meters since last update
   float angle_turned;                //!< in radian since last update
   float driving_speed;               //!< in m/sec
   float turning_speed;               //!< in radian/sec
   float left_motor_current;          //!< in ampere
   float right_motor_current;         //!< in ampere
   bool cooldown_active;             //!< raised if motors have been disabed for a cool-down period
   float temperature;                 //!< measured temperature in °C at the power FETs
   float voltage;                     //!< measured supply voltage
   bool dig_in_1_state;               //!< level of digital input 1 (if used)
   bool dig_in_2_state;               //!< level of digital input 2 (if used)
   float air_pressure;                //!< air pressure in bar (if used)
} DRIVETRAIN_STATUS;

/** Represents the robot's drivetrain
 */
class Taurob_base : public IUdpReceiver
{
	public:		
        Taurob_base(std::string host_ip,
                    int host_port,
                    double gear_ratio,
                    double wheel_diameter,
                    double track_width,
                    double turning_geometry_factor);
		~Taurob_base();

        /** implementation for IUdpReceiverBinary */
        virtual void On_string_received(std::string msg_data, char* from_remote_ip, int from_remote_port, int from_local_port);

        /** Starts communicating with the drivetrain u-ecu via UDP*/
        void Run();

        /** Stops communicating with the drivetrain u-ecu via UDP*/
		void Stop();

        /** Safety feature: has to be called every 250 ms or the drivetrain will stop */
        void Feed_watchdog();

        /** Tells the robot where to go
          *
          * @param forward_speed in m/sec
          * @param turning_speed in radian/sec
          */
        void Set_drive_command(float forward_speed, float turning_speed);

        /** Called if a status frame was received */
        void Set_on_received_callback(void (*callback)(DRIVETRAIN_STATUS status));

	private:
        static const int WATCHDOG_MAX_TIME = 250;	//ms
		static const int CURRENT_AVERAGE_ELEMENTS = 3;
        static const int AIR_PRESSURE_ELEMENTS = 100;
		static const double MAX_TOTAL_MOTOR_CURRENT = 25.0;
        static const double MAX_TEMPERATURE = 90.0;
        static const int COOLDOWN_PERIOD_MS = 10000;

        std::string ECU_host_ip;
        int ECU_host_port;
        void (*on_receive_callback)(DRIVETRAIN_STATUS status);
		double avg_total_motor_current;
        double avg_air_pressure;
        bool cooldown_active;
        boost::posix_time::ptime cooldown_time;
        boost::posix_time::ptime latest_watchdog_time;
        bool sending_thread_running;
        boost::thread sending_thread;
        double gear_ratio;
        double wheel_diameter;
        double track_width;
        double turning_geometry_factor; // by experimentation

		SocketUDP* ecu_socket;

        Base_command_frame current_set_values;
        boost::mutex current_set_values_locker;

		void Sending_thread();
        void Init_frames();
		bool Watchdog_ok();
};

#endif /* LIBTAUROB_TRACKER_BASE_H_ */

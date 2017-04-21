/**************************************************************************//**
 *
 * @file libtaurob_arm.h
 * @author Lukas Silberbauer, taurob GmbH
 * @date 2 Feb 2017
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
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <diagnostic_updater/publisher.h>

#include "Arm_frames.h"
#include "libtaurob_tools/SocketUDPBinary.h"

typedef struct Arm_config_
{
	std::vector<std::string> joint_names;
	std::vector<std::string> joint_ips;
	std::vector<int> joint_ports;
    std::vector<int> joint_position_tolerance_mrad;
	
    void Add_segment(std::string name, std::string ip, int port, int position_tolerance_mrad)
	{
		joint_names.push_back(name);
		joint_ips.push_back(ip);
		joint_ports.push_back(port);
        joint_position_tolerance_mrad.push_back(position_tolerance_mrad);
	}
} Arm_config;

typedef enum
{
    STOP = 0,                  //!< the arm joint is stopped
    POSITION_CONTROL = 1,      //!< the arm drives to a set position
    SPEED_CONTROL = 2          //!< the arm joint rotates with a set speed
} ARM_SEGMENT_STATE;

typedef struct _ARM_SEGMENT_COMMAND
{
    ARM_SEGMENT_STATE state;
    float set_position;         //!< absolute position in rad
    float max_speed;            //!< max speed to reach position in rad/sec, or 0 if default max speed is used
                                //!< only used when state == POSITION_CONTROL
    float set_speed;            //!< speed in rad/sec
    bool reset_friction_clutch; //!< resets the friction clutch warning and re-enables arm movement
    bool control_aux_port;      //!< if true the aux_values are used - can only be used by one task per U-ECU!
    bool enable_five_volt;      //!< if control_aux_port == true, 5V aux port is set / cleared
    bool enable_twelve_volt;    //!< if control_aux_port == true, 12V aux port is set / cleared
    bool enable_dig_out_1;      //!< if control_aux_port == true, digital output 1 is set / cleared
    bool enable_dig_out_2;      //!< if control_aux_port == true, digital output 1 is set / cleared
    bool reset_u_ecu;           //!< allows to trigger a U-ECU reset - only possible once each minute, use with caution!
} ARM_SEGMENT_COMMAND;

#define SEGMENT_STATUS_NO_ERROR                    0x00
#define SEGMENT_STATUS_CONNECTION_LOST             0x01
#define SEGMENT_STATUS_FRICTION_CLUTCH_SLIPPED     0x02
#define SEGMENT_STATUS_ANGLE_NOT_VALID             0x04

typedef struct _ARM_SEGMENT_STATUS
{
   unsigned char status_bitfield;   //!< see SEGMENT_STATUS bits above
   long timestamp_sec;              //!< if RTC has been set, seconds since 1.1.1970, else 0
   long timestamp_ns;               //!< if RTC has been set, the remainder of seconds in nanoseconds, else 0
   float position;                  //!< absolute position in rad
   float speed;                     //!< current speed in rad/sec
   float effort;                    //!< current effort in Nm (note: not very accurate due to snail drives)
   float current;                   //!< measured current in ampere
   float temperature;               //!< measured temperature in °C at the power FETs
   float voltage;                   //!< measured supply voltage
   bool dig_in_1_state;             //!< level of digital input 1
   bool dig_in_2_state;             //!< level of digital input 2
} ARM_SEGMENT_STATUS;

/** Used for processing arm segment status updates
 */
class IReceiveCallback
{
    public:
        virtual void On_status_update_received(int segment_number, ARM_SEGMENT_STATUS status) = 0;
};

/** Represents a single arm joint, communicates with a task of a u-ecu (motor
 *  controller)
 */
class Arm_segment : IUdpReceiverBinary
{
	public:
        static const int DEFAULT_MAX_INPUT_VALUE = 4095;

        Arm_segment(std::string segment_name,
					std::string host_ip, 
					int host_port, 
                    int segment_nr,
                    int position_tolerance_mrad,
                    IReceiveCallback *status_update_callback);
		~Arm_segment();
		
        /** implementation for IUdpReceiverBinary */
		void On_data_received(unsigned char* data, int len, char* from_remote_ip, int from_remote_port, int from_local_port);
		
        /** Starts communicating with the arm joint via UDP */
		void Run();

        /** Stops communicating with the arm joint via UDP  */
		void Stop();

        /** Control the arm joint */
        void Set(ARM_SEGMENT_COMMAND command);

        /** returns the segment's name */
        std::string Get_name();

        void produce_connection_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

	private:        
        static const int MAX_CONNECTION_LOSS_TIME_MS = 250;
        static const int SENDING_THREAD_PERIOD_MS = 20;
        static const int RESET_FRICTION_CLUTCH_FRAMES = 10;

		std::string segment_name;
		std::string host_ip;
		int host_port;
        int segment_nr;
		int reset_friction_clutch_counter;
		SocketUDPBinary* ecu_socket;	        
        boost::posix_time::ptime latest_rx_frame_time;
        bool sending_thread_running;
        IReceiveCallback *parent_callback;
        Arm_frames* arm_frames;
        MOD_ARM_COMMAND_FRAME current_set_values;
        ARM_SEGMENT_STATUS current_status;
        boost::mutex current_set_values_locker;
        boost::mutex current_status_locker;
        boost::thread sending_thread;
        int position_tolerance_mrad;
        bool position_reached;
        int16_t last_converted_position_value;

        void Init_UDP_frames();
		void Sending_thread();
};

class Arm : IReceiveCallback
{
    public:
        Arm(Arm_config configuration);
        ~Arm();

        /** Starts communicating with all arm joints via UDP */
        void Run();

         /** Stops communicating with all arm joints via UDP  */
        void Stop();

        /** Safety feature: has to be called every 250 ms or the arm will stop */
        void Feed_watchdog();

        /** Control an arm joint */
        void Set(int segment_number, ARM_SEGMENT_COMMAND command);

        /** Get arm joint data */
        ARM_SEGMENT_STATUS Get(int segment_number);

        /** get number of segments */
        int Get_segment_count(void);

        /** returns the segment's name */
        std::string Get_segment_name(int segment_nr);

        /** Returns false if an arm_segment has detected an error */
        bool Is_ok(void);

        /** IReceiveCallback: Set call back for status update events */
        void On_status_update_received(int segment_number, ARM_SEGMENT_STATUS current_status);

        void produce_connection_diagnostics(int segment_number, diagnostic_updater::DiagnosticStatusWrapper& stat);
        void produce_temperature_diagnostics(int segment_number, diagnostic_updater::DiagnosticStatusWrapper& stat);
        void produce_clutch_diagnostics(int segment_number, diagnostic_updater::DiagnosticStatusWrapper& stat);

    private:        
        static const int WATCHDOG_MAX_TIME_MS = 250;
        static const int WATCHDOG_CHECKER_THREAD_PERIOD_MS = 5;

        bool config_successful;
        std::vector<Arm_segment*> segments;
        std::vector<ARM_SEGMENT_STATUS> segment_status;
        boost::mutex segment_status_locker;
        boost::posix_time::ptime latest_watchdog_time;
        boost::thread watchdog_checker_thread;
        bool watchdog_checker_thread_running;
        bool watchdog_has_expired;

        /** Stops the movement but not the communication to the arm joints */
        void Halt_entire_arm(void);

        /** Checks if the watchdog has been fed well */
        void Watchdog_checker(void);
};

#endif

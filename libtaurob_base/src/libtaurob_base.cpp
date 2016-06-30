/**************************************************************************//**
 *
 * @file libtaurob_base.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Library for communication with taurob base ECU
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

#include "libtaurob_base/libtaurob_base.h"
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost::tuples;
using namespace boost;


bool debug_output_enabled = false;

Taurob_base::Taurob_base(std::string host_ip, int host_port, int protocol_version, bool start_sending_initially) : 
	protocol_version(protocol_version),
	ECU_host_ip(host_ip), 
	ECU_host_port(host_port), 
	pause_sending(!start_sending_initially),
	on_receive_callback(0),
	motor_fault(false),
	avg_total_motor_current(0),
	watchdog_enabled(false),
	first_frame_sent(false)
{ 
	Set_pause_sending(pause_sending);
}

Taurob_base::~Taurob_base() 
{
	sending_thread_running = false;
}

// ################# COMMUNICATION ####################

bool Taurob_base::Watchdog_ok()
{
        bool ret = false;

        tTimeDuration dt = boost::posix_time::microsec_clock::local_time() - latest_watchdog_time;
        if (dt.total_milliseconds() <= WATCHDOG_MAX_TIME) ret = true;

        return ret;
}


void Taurob_base::Sending_thread()
{
	static bool command_rx_timeout_message = false;
	
	DEBUG("starting sending thread\n");
	receive_pause = false;
	
	while (sending_thread_running)
	{
		// if watchdog is disabled, it is always fed.
		if (watchdog_enabled == false)
		{
			Feed_watchdog();
		}
		
		// only consider sending a command frame if control is enabled in the first place
		// this should be safe even without stopping the robot explicitly, since it will 
		// stop anyway if it doesn't receive command frames any more.
		if (pause_sending == false)
		{
			// lock the locker from early on to avoid other threads messing up our safe state (->watchdog)
			current_set_values_locker.lock();

			if (Watchdog_ok() == false)
			{
				// if watchdog timed out, force-stop the robot
				current_set_values.vector_x = 0;
				current_set_values.vector_y = 0;
				if (receive_pause == false)
				{
					printf("[Base ECU] No command received since more than 110ms (Watchdog timeout) - set speed to 0\n");
					receive_pause = true;
				}
			}
			else if (receive_pause == true)
			{
				receive_pause = false;
				printf("Command frame was received again, allowing to drive again.\n");
			}
				
			current_tx_seqno = (current_tx_seqno + 1) % 255;
			current_set_values.sequence_number = current_tx_seqno;
		
			// Actual sending of the frame - unless we have a motor fault, in which case we cease communication alltogether
			if (motor_fault == false)
			{
				if (first_frame_sent == false)
				{
					// calibrate flippers to zero with first frame
					current_set_values.gripper_pos_set = FLIPPER_CENTER_POS + FLIPPER_CALIBRATION_MODE_OFFSET;
				}
				ecu_socket->send(Base_frames::Command_to_string(current_set_values, protocol_version));
				if (first_frame_sent == false)
				{
					current_set_values.gripper_pos_set = FLIPPER_CENTER_POS;
					first_frame_sent = true;
				}
			}
			current_set_values_locker.unlock();
			
			// see when we had the last drive command, stop robot if it's been too long
			// this is the alternative to watchdog, so if watchdog isn't active, this is.
			if (watchdog_enabled == false)
			{
				tTimeDuration dt_drive_cmd = boost::posix_time::microsec_clock::local_time() - latest_drive_command_time;
				if (dt_drive_cmd.total_milliseconds() >= MAX_TIME_WITHOUT_DRIVE_CMD_MS)
				{
					if (command_rx_timeout_message == false)
					{
						printf("[Base ECU] No drive command since >%dms. Stopping the robot.\n", MAX_TIME_WITHOUT_DRIVE_CMD_MS);
						command_rx_timeout_message = true;
					}
					current_set_values_locker.lock();
					current_set_values.vector_x = 0;
					current_set_values.vector_y = 0;
					current_set_values_locker.unlock();
				}
				else if (command_rx_timeout_message == true)
				{
					printf("[Base ECU] Received drive command again.\n");
					command_rx_timeout_message = false;
				}
			}
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(40));
	}
}

void Taurob_base::Run() 
{
	Init_frames();
	sending_thread = boost::thread(boost::bind(&Taurob_base::Sending_thread, this));

	sending_thread_running = true;
	
	DEBUG("[Base ECU] Opening ECU socket to %s:%d\n", ECU_host_ip.c_str(), ECU_host_port);
	
	ecu_socket = new SocketUDP(ECU_host_ip, ECU_host_port); // To Send & Receive
	ecu_socket->start_listen(this);

	// start ui server socket for frame relaying
	ui_socket = new SocketUDP(UI_SERVER_PORT);
	ui_socket->start_listen(this);
}

void Taurob_base::Stop()
{
	sending_thread_running = false;
	
	DEBUG("[Base ECU] Stopping - Waiting for threads to join...\n");
	sending_thread.join();
	DEBUG("[Base ECU] ECUClient::run() completed\n");

	ecu_socket->stop_listen();
	delete ecu_socket;
	ecu_socket = null;
	
	ui_socket->stop_listen();
	delete ui_socket;
	ui_socket = null;
}

void Taurob_base::Init_frames()
{
	// initialize internal frames to safe values - they are all set to 0 by default
	current_set_values_locker.lock();
	current_set_values.vector_x = 0;
	current_set_values.vector_y = 0;
	current_set_values.ot_threshold_stage1 = 255;
	current_set_values.ot_threshold_stage2 = 255;
	current_set_values.manipulator_pos_set = INVALID_ANGLE_VALUE;
	current_set_values.gripper_pos_set = FLIPPER_CENTER_POS;
	current_set_values_locker.unlock();
	
	current_get_values_locker.lock();
	current_get_values.sequence_number = 0;
	current_get_values.BLDC1_speed_get = 0;
	current_get_values.BLDC2_speed_get = 0;
	current_get_values.BLDC1_current = 0;
	current_get_values.BLDC2_current = 0;
	current_get_values.distance_travelled_1 = 0;
	current_get_values.distance_travelled_2 = 0;
	current_get_values.gripper_pos_get = 0;
	current_get_values.manipulator_angle = 0;
	current_get_values.voltage = 0;
	current_get_values.temp1 = 40;
	current_get_values.temp2 = 0;
	current_get_values.temp3 = 0;
	current_get_values.temp4 = 0;
	current_get_values.pitch = 0;
	current_get_values.roll = 0;
	current_get_values.yaw = 0;
	current_get_values.acc_x = 0;
	current_get_values.acc_y = 0;
	current_get_values.acc_z = 0;
	current_get_values.gyro_x = 0;
	current_get_values.gyro_y = 0;
	current_get_values.gyro_z = 0;
	current_get_values.mag_x = 0;
	current_get_values.mag_y = 0;
	current_get_values.mag_z = 0;
	current_get_values.pos_x = 0;
	current_get_values.pos_y = 0;
	current_get_values.pos_z = 0;
	current_get_values.error_code = 0;
	current_get_values.aux_in = 0;
	current_get_values_locker.unlock();
}

// attention: this not only sets watchdog, but also command_rx_timeout_enabled!
void Taurob_base::Set_watchdog_enabled(bool state)
{
	watchdog_enabled = state;
}

void Taurob_base::Set_pause_sending(bool pause)
{
	pause_sending = pause;
	
	// if we lose control of the robot, all internally cached command values must 
	// be reset to a safe state, so that the robot won't do unpredictable things 
	// when we re-gain control
	Init_frames();
	if (pause_sending == true)
	{
		DEBUG("[Base ECU] control was disabled -- starting relay server\n");
	}
	else
	{
		DEBUG("[Base ECU] control was enabled -- stopping relay server\n");
	}
}

// receive from ecu
void Taurob_base::On_string_received(std::string msg_data, char* from_remote_ip, int from_remote_port, int from_local_port)
{
	if (pause_sending == true && from_local_port == UI_SERVER_PORT) 	// note: ip check would be possible here too, but it makes simulation/debugging tricky (if the sim runs on the same machine as the UI)
	{
		//DEBUG("Received UI command frame: %s\n", msg_data.c_str());
		//DEBUG("control is not enabled, relaying received command frame to ECU...\n");
		
		// relay frame received from ui to ecu
		ecu_socket->send(msg_data);
	}
	else if (from_local_port != UI_SERVER_PORT)
	{
		//DEBUG("Publishing ECU Output Message: %s\n", msg_data.c_str());

		current_get_values_locker.lock();
		latest_rx_frame_time = boost::posix_time::microsec_clock::universal_time();
		current_get_values_locker.unlock();
		
		Base_status_frame received_values;
		if (Base_frames::String_to_status(msg_data, protocol_version, &received_values))
		{
			current_get_values_locker.lock();
			current_get_values.sequence_number = received_values.sequence_number;
			current_get_values.BLDC1_speed_get = received_values.BLDC1_speed_get;
			current_get_values.BLDC2_speed_get = received_values.BLDC2_speed_get;
			current_get_values.BLDC1_current = received_values.BLDC1_current;
			current_get_values.BLDC2_current = received_values.BLDC2_current;
			current_get_values.distance_travelled_1 = received_values.distance_travelled_1;
			current_get_values.distance_travelled_2 = received_values.distance_travelled_2;
			current_get_values.gripper_pos_get = received_values.gripper_pos_get;
			current_get_values.manipulator_angle = received_values.manipulator_angle;
			current_get_values.voltage = received_values.voltage;
			current_get_values.temp1 = received_values.temp1;
			current_get_values.temp2 = received_values.temp2;
			current_get_values.temp3 = received_values.temp3;
			current_get_values.temp4 = received_values.temp4;
			current_get_values.pitch = received_values.pitch;
			current_get_values.roll = received_values.roll;
			current_get_values.yaw = received_values.yaw;
			current_get_values.acc_x = received_values.acc_x;
			current_get_values.acc_y = received_values.acc_y;
			current_get_values.acc_z = received_values.acc_z;
			current_get_values.gyro_x = received_values.gyro_x;
			current_get_values.gyro_y = received_values.gyro_y;
			current_get_values.gyro_z = received_values.gyro_z;
			current_get_values.mag_x = received_values.mag_x;
			current_get_values.mag_y = received_values.mag_y;
			current_get_values.mag_z = received_values.mag_z;
			current_get_values.pos_x = received_values.pos_x;
			current_get_values.pos_y = received_values.pos_y;
			current_get_values.pos_z = received_values.pos_z;
			current_get_values.error_code = received_values.error_code;
			current_get_values.aux_in = received_values.aux_in;
			current_get_values_locker.unlock();

			Check_for_errors();
			
			if (on_receive_callback != 0)
			{
				on_receive_callback();
			}
			if (ui_socket != null)
			{
				// relay frame from ecu to ui
				ui_socket->send(msg_data);
			}
		}
		else
		{
			DEBUG("[Base ECU] Status frame has invalid format -- doing nothing.\n");
		}			
	}
}

void Taurob_base::Check_for_errors()
{
	current_get_values_locker.lock();
	double current_total_motor_current = (current_get_values.BLDC1_current + current_get_values.BLDC2_current) / (0.101 * 25 * 0.95); 	// constants are from firmware
	avg_total_motor_current -= avg_total_motor_current / CURRENT_AVERAGE_ELEMENTS;
	avg_total_motor_current += current_total_motor_current;

	double current = avg_total_motor_current / CURRENT_AVERAGE_ELEMENTS;
	if (current > MAX_TOTAL_MOTOR_CURRENT)
	{
		motor_fault = true;
		if (on_error_callback != 0)
		{
			on_error_callback(0x40);
		}
	}

	if (current_get_values.error_code != 0)
	{
		if (current_get_values.error_code & 0x08)
		{
			motor_fault = true;
		}
		if (on_error_callback != 0)
		{
			on_error_callback(current_get_values.error_code);
		}
	}
}

void Taurob_base::Feed_watchdog()
{
	/*struct timeval  tv;
	gettimeofday(&tv, NULL);
	double time_in_mill = (tv.tv_usec) / 1000;

	printf("[%d.%d] Feeding watchdog...\n", tv.tv_sec, (int)time_in_mill);*/
	latest_watchdog_time = boost::posix_time::microsec_clock::local_time();
}

void Taurob_base::Set_on_received_callback(void (*callback)())
{
	on_receive_callback = callback;
}

void Taurob_base::Set_on_error_callback(void (*callback)(int))
{
	on_error_callback = callback;
}

void Taurob_base::Tilt_calibrate_zero()
{
}

void Taurob_base::Start_compass_calibration()
{
}

void Taurob_base::Stop_compass_calibration()
{
}

// ############ SETTERS ##############

void Taurob_base::Set_drive_command(tuple<float, float> drive_command)
{
	if (!pause_sending && !receive_pause)
	{
		latest_drive_command_time = boost::posix_time::microsec_clock::local_time();
		
		// drive_command<0> is linear
		float vector_y = (float) ((get<0>(drive_command) * gear_ratio * 60.0) / (wheel_diameter * M_PI));
		
		// drive_command<1> is angular
		float vector_x = (float) ((-get<1>(drive_command) * gear_ratio * 60.0 * track_width * turning_geometry_factor)
								/ (wheel_diameter * 2 * M_PI));
					
		current_set_values_locker.lock();
		current_set_values.vector_x = vector_x;
		current_set_values.vector_y = vector_y;
		current_set_values_locker.unlock();
	}
}

void Taurob_base::Set_gripper_angle(float degrees)
{
	if (!pause_sending && !receive_pause)
	{
		// convert from degrees to an ECU-compatible number
		int setpos = (int)(-degrees * FLIPPER_DEGREES_TO_PULSES_FACTOR + FLIPPER_CENTER_POS);
		
		current_set_values_locker.lock();
		current_set_values.gripper_pos_set = setpos;
		current_set_values_locker.unlock();
	}
}

void Taurob_base::Set_manipulator_base_angle(float degrees)
{
	if (!pause_sending && !receive_pause)
	{
		current_set_values_locker.lock();
		current_set_values.manipulator_pos_set = degrees;
		current_set_values_locker.unlock();
	}
}

void Taurob_base::Set_light(bool state)
{
	if (!pause_sending)
	{
		current_set_values_locker.lock();
		if (state == true)
		{
			current_set_values.bitfield |= (1 << 0);
		}
		else
		{
			current_set_values.bitfield &= ~(1 << 0);
		}
		current_set_values_locker.unlock();
	}
}

void Taurob_base::Set_bluelight(bool state)
{
	if (!pause_sending && !receive_pause)
	{
		current_set_values_locker.lock();
		if (state == true)
		{
			current_set_values.bitfield |= (1 << 1);
		}
		else
		{
			current_set_values.bitfield &= ~(1 << 1);
		}
		current_set_values_locker.unlock();
	}
}
	
// do not use
void Taurob_base::Set_overturn_angles(tuple<float, float, float, float> angles)
{
	if (!pause_sending && !receive_pause)
	{
		current_set_values_locker.lock();
		current_set_values.overturn_angle_front = get<0>(angles);
		current_set_values.overturn_angle_back = get<1>(angles);
		current_set_values.overturn_angle_left = get<2>(angles);
		current_set_values.overturn_angle_right = get<3>(angles);
		current_set_values_locker.unlock();
	}
}

void Taurob_base::Set_overturn_thresholds(tuple<float, float> thresholds)
{
	if (!pause_sending && !receive_pause)
	{
		current_set_values_locker.lock();
		current_set_values.ot_threshold_stage1 = get<0>(thresholds);
		current_set_values.ot_threshold_stage2 = get<1>(thresholds);
		current_set_values_locker.unlock();
	}
}

// getters
tuple<float, float> Taurob_base::Get_wheel_speeds()
{
	tuple<float, float> ret;
	current_get_values_locker.lock();
	ret = make_tuple(current_get_values.BLDC1_speed_get, current_get_values.BLDC2_speed_get);
	current_get_values_locker.unlock();
	return ret;
}

tuple<float, float> Taurob_base::Get_motor_currents()
{
	tuple<float, float> ret;
	current_get_values_locker.lock();
	ret = make_tuple(current_get_values.BLDC1_current, current_get_values.BLDC2_current);
	current_get_values_locker.unlock();
	return ret;
}

tuple<float, float> Taurob_base::Get_motor_distance()
{
	tuple<float, float> ret;
	current_get_values_locker.lock();
	ret = make_tuple(current_get_values.distance_travelled_1, current_get_values.distance_travelled_2);
	current_get_values_locker.unlock();
	return ret;
}

float Taurob_base::Get_gripper_angle()
{
	float ret = 0;
	current_get_values_locker.lock();
	ret = current_get_values.gripper_pos_get;
	current_get_values_locker.unlock();
	return ret;
}

float Taurob_base::Get_manipulator_base_angle()
{
	float ret = 0;
	current_get_values_locker.lock();
	ret = current_get_values.manipulator_angle;
	current_get_values_locker.unlock();
	return ret;
}

float Taurob_base::Get_supply_voltage()
{
	float ret = 0;
	current_get_values_locker.lock();
	if (current_get_values.voltage != 0)
		ret = (current_get_values.voltage / 10.0) + 10.0;
	current_get_values_locker.unlock();
	return ret;
}

tuple<float, float, float, float> Taurob_base::Get_temperatures()
{
	tuple<float, float, float, float> ret;
	current_get_values_locker.lock();
	ret = make_tuple(current_get_values.temp1 - 40,
							current_get_values.temp2,
							current_get_values.temp3,
							current_get_values.temp4);
	current_get_values_locker.unlock();
	return ret;
}

tuple<float, float, float> Taurob_base::Get_orientation()
{
	tuple<float, float, float> ret;
	current_get_values_locker.lock();
	ret = make_tuple(current_get_values.pitch,
							current_get_values.roll,
							current_get_values.yaw);
	current_get_values_locker.unlock();
	return ret;
}

tuple<float, float, float> Taurob_base::Get_gyro_data()
{
	tuple<float, float, float> ret;
	current_get_values_locker.lock();
	ret = make_tuple(current_get_values.gyro_x,
							current_get_values.gyro_y,
							current_get_values.gyro_z);
	current_get_values_locker.unlock();
	return ret;
}

tuple<float, float, float> Taurob_base::Get_acc_data()
{
	tuple<float, float, float> ret;
	current_get_values_locker.lock();
	ret = make_tuple(current_get_values.acc_x,
							current_get_values.acc_y,
							current_get_values.acc_z);
	current_get_values_locker.unlock();
	return ret;
}

tuple<float, float, float> Taurob_base::Get_mag_data()
{
	tuple<float, float, float> ret;
	current_get_values_locker.lock();
	ret = make_tuple(current_get_values.mag_x,
							current_get_values.mag_y,
							current_get_values.mag_z);
	current_get_values_locker.unlock();
	return ret;
}


unsigned char Taurob_base::Get_error_code()
{
	unsigned char ret = 0;
	current_get_values_locker.lock();
	ret = current_get_values.error_code;
	current_get_values_locker.unlock();
	return ret;
}

unsigned char Taurob_base::Get_AUX_bits()
{
	unsigned char ret = 0;
	current_get_values_locker.lock();
	ret = current_get_values.aux_in;
	current_get_values_locker.unlock();
	return ret;
}

boost::posix_time::ptime Taurob_base::Get_last_frame_timestamp()
{
	boost::posix_time::ptime ret;
	current_get_values_locker.lock();
	ret = latest_rx_frame_time;
	current_get_values_locker.unlock();
	return ret;
}

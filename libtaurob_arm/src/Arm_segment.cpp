/**************************************************************************//**
 *
 * @file Arm_segment.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Library for communication with taurob arm, class representing one arm segment
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

#include "libtaurob_arm/libtaurob_arm.h"

using namespace boost;

bool debug_output_enabled = false;

Arm_segment::Arm_segment(std::string name, 
						 std::string host_ip, 
						 int host_port, 
						 int segment_nr, 
						 bool start_sending_initially,
						 int max_input_value, 
						 int ui_server_base_port) : 
	host_ip(host_ip), 
	host_port(host_port),
	relay(0),
	segment_nr(segment_nr),
	segment_name(name),
	segment_static_counter(0),
	watchdog_enabled(false),
	reset_ecu(false),
	force_motor_enable(false),
	check_for_static_motor(true),
	reset_friction_clutch_counter(0),
	pause_sending(!start_sending_initially),
	ui_server_base_port(ui_server_base_port),
	max_input_value(max_input_value)
{ 
	Set_pause_sending(pause_sending);
}

Arm_segment::Arm_segment(std::string name, 
						 std::string host_ip, 
						 int host_port, 
						 int segment_nr, 
						 bool start_sending_initially) : 
	host_ip(host_ip), 
	host_port(host_port),
	relay(0),
	segment_nr(segment_nr),
	segment_name(name),
	segment_static_counter(0),
	reset_friction_clutch_counter(0),
	watchdog_enabled(false),
	reset_ecu(false),
	force_motor_enable(false),
	check_for_static_motor(true),
	pause_sending(!start_sending_initially),
	ui_server_base_port(DEFAULT_UI_SERVER_BASE_PORT),
	max_input_value(DEFAULT_MAX_INPUT_VALUE)
{ 
	Set_pause_sending(pause_sending);
}

Arm_segment::~Arm_segment() 
{
	sending_thread_running = false;
}

// ################# COMMUNICATION ####################

bool Arm_segment::Is_uptodate()
{
	bool ret = true;
	
	// we say we are 'uptodate' if the latest status frame isn't older than MAX_BLIND_TIME
	if ((boost::posix_time::microsec_clock::local_time() - latest_rx_frame_time).total_milliseconds() > MAX_BLIND_TIME)
	{
		//printf("[Arm seg. #%d (%d)] Not up to date.\n", segment_nr, host_port);
		ret = false;
	}
	return ret;
}

void Arm_segment::Print_timestamp_debug()
{
	// ptime to string.
	const std::string str_now = to_iso_extended_string(boost::posix_time::microsec_clock::local_time());
	const std::string str_time = to_iso_extended_string(latest_rx_frame_time);
	
	printf("\n\n### Timestamps: \n\tNow: %s\n\tLatest rx frame time: %s\n\n\n", str_now.c_str(), str_time.c_str());
}

bool Arm_segment::Watchdog_ok()
{
	bool ret = false;
	
	tTimeDuration dt = boost::posix_time::microsec_clock::local_time() - latest_watchdog_time;
	if (dt.total_milliseconds() <= WATCHDOG_MAX_TIME) ret = true;
			
	return ret;
}

// suppress sending for RESET_ECU_DURATION (typ. 400) ms to reset modarm ecu
void Arm_segment::Reset_connection()
{
	reset_ecu = true;
	reset_ecu_timestamp = boost::posix_time::microsec_clock::local_time();
}

void Arm_segment::Check_if_allowed_to_drive()
{
	// first of all, see if we want to drive in the first place
	bool want_to_drive = false;
	current_set_values_locker.lock();
	if ((current_set_values.bitfield & (1 << 6)) != 0) want_to_drive = true;
	current_set_values_locker.unlock();
	if (want_to_drive == false)
	{
		DEBUG("[Arm seg. #%d (%d)] no motor enable since we don't want to drive.\n", segment_nr, host_port);
		return;
	}

	// if getpos == setpos, do not set motor enable (there is a firmware bug as of now, but also it's not necessary)
	bool same_pos = false;

	// because we transform int->float and rad->deg and back, it might 
	// happen that "get=set" is off by one due to float rounding. have a tolerance 
	// for that.
	if (check_for_static_motor)
	{
		current_get_values_locker.lock();
		current_set_values_locker.lock();
	
		if (current_set_values.set_position == current_get_values.position ||
			(((current_set_values.set_position + 1) % max_input_value) == current_get_values.position) ||
			(current_set_values.set_position == ((current_get_values.position + 1) % max_input_value)))
		{
			same_pos = true;
		}
		current_set_values_locker.unlock();
		current_get_values_locker.unlock();

		if (same_pos == true)
		{
			Set_motor_enable(false); 	// set motor enable locks set, so we mustn't call it inside a set locker
			DEBUG("[Arm seg. #%d (%d)] Clearing motor enable because setpos=getpos\n", segment_nr, host_port);
			return;
		}

		if (segment_static_counter > SEGMENT_STATIC_COUNTER_MAX)
		{
			DEBUG("[Arm seg. #%d (%d)] Static counter reached max, disabling movement\n", segment_nr, host_port);
			Set_motor_enable(false);
			segment_static_counter = 0;
			return;
		}
	}
		
	if ((boost::posix_time::microsec_clock::local_time() - latest_drive_command_time).total_milliseconds() >= MAX_DRIVE_TIME)
	{
		DEBUG("[Arm seg. #%d (%d)] Drive timeout, disabling movement\n", segment_nr, host_port);
		Set_motor_enable(false);
		return;
	}
	if (reset_friction_clutch_counter > 0)
	{
		printf("[Arm seg. #%d (%d)] resetting friction clutch (counter: %d) - setting motor enable false\n", segment_nr, host_port, reset_friction_clutch_counter);
		Set_motor_enable(false);
		--reset_friction_clutch_counter;
		if (reset_friction_clutch_counter < 0)
		{
			reset_friction_clutch_counter = 0;
		}
		return;
	}

	// see when we received last status frame, stop robot if it's been too long
	// this is in addition to the watchdog
	if (Is_uptodate() == false)
	{
		printf("[Arm seg. #%d (%d)] Receive timeout, disabling movement\n", segment_nr, host_port);
		Set_motor_enable(false);
		return;
	}

	if (Watchdog_ok() == false)
	{
		DEBUG("[Arm seg. #%d (%d)] Watchdog nok - set motor enable false\n", segment_nr, host_port);
		Set_motor_enable(false);
		if (receive_pause == false)
		{
			printf("[Arm seg. #%d (%d)] No command received since more than 110ms (Watchdog timeout) - motors are disabled\n", segment_nr, host_port);
			receive_pause = true;
		}
		return;
	}
	else if (receive_pause == true)
	{
		receive_pause = false;
		printf("[Arm seg. #%d (%d)] Command frame was received again, re-enabling motor movement.\n", segment_nr, host_port);
	}
}

void Arm_segment::Sending_thread()
{
	static bool command_rx_timeout_message = false;
	
	receive_pause = false;
	
	unsigned char sendbuffer[Arm_frames::COMMAND_FRAME_LENGTH];
	
	while (sending_thread_running)
	{
		if (reset_ecu == false)
		{
			// if watchdog is disabled, it is always fed.
			if (watchdog_enabled == false)
			{
				Feed_watchdog();
			}
			
			if (pause_sending == false)
			{
				Check_if_allowed_to_drive();
				if (force_motor_enable)
				{
					Set_motor_enable(true);
					force_motor_enable = false;
				}
					
				current_set_values_locker.lock();
				current_tx_seqno = (current_tx_seqno + 1) % 255;
				current_set_values.sequence_number = current_tx_seqno;
DEBUG("[Arm seg. #%d (%d)] Motor enable: %d\n", segment_nr, host_port, current_set_values.bitfield);
				Arm_frames::Command_to_bytes(current_set_values, sendbuffer);
				current_set_values_locker.unlock();

				ecu_socket->send(sendbuffer, Arm_frames::COMMAND_FRAME_LENGTH);

				//DEBUG("Segment %d (%d) sent: %d\n", segment_nr, host_port, current_set_values.set_position);
			}
		}
		else
		{
			tTimeDuration dt = boost::posix_time::microsec_clock::local_time() - reset_ecu_timestamp;
			if (dt.total_milliseconds() > RESET_ECU_DURATION) 
			{
				reset_ecu = false;
				printf("[Arm seg. #%d (%d)] Connection was reset. Resuming to send commands.\n", segment_nr, host_port);
			}
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(40));
	}
}

void Arm_segment::Run() 
{
	if (max_input_value == 0) max_input_value = DEFAULT_MAX_INPUT_VALUE;
	if (ui_server_base_port == 0) ui_server_base_port = DEFAULT_UI_SERVER_BASE_PORT;
	Init_frames();
	
	sending_thread_running = true;
	ecu_socket = new SocketUDPBinary(host_ip, host_port, host_port, Arm_frames::STATUS_FRAME_LENGTH); // bind to same port as host_port, since ecu will send responses there
	ecu_socket->start_listen(this);

	// start ui server socket for frame relaying
	int bufferlen = (Arm_frames::COMMAND_FRAME_LENGTH > Arm_frames::STATUS_FRAME_LENGTH ? Arm_frames::COMMAND_FRAME_LENGTH : Arm_frames::STATUS_FRAME_LENGTH);
	bufferlen++;  	// just a precaution

	ui_socket = new SocketUDPBinary(ui_server_base_port + segment_nr, bufferlen);
	ui_socket->start_listen(this);

	//DEBUG("starting sending thread...\n");
	sending_thread = boost::thread(boost::bind(&Arm_segment::Sending_thread, this));
	//DEBUG("after thread creation. host ip: %s\n", host_ip.c_str());

	DEBUG("Segment %d opened connection to ip %s, port %d\n", segment_nr, host_ip.c_str(), host_port);
}

void Arm_segment::Stop()
{
	sending_thread_running = false;
	
	DEBUG("[Arm seg. #%d] Stopping - Waiting for threads to join...\n", segment_nr);
	sending_thread.join();
	DEBUG("[Arm seg. #%d] ECUClient::run() completed\n", segment_nr);

	ecu_socket->stop_listen();
	delete ecu_socket;
	ecu_socket = 0;
	
	ui_socket->stop_listen();
	delete ui_socket;
	ui_socket = 0;
}

void Arm_segment::Init_frames()
{
	// initialize internal frames to safe values - they are all set to 0 by default
	current_set_values_locker.lock();
	current_set_values.set_position = INVALID_ANGLE_VALUE;
	current_set_values.bitfield = 0;
	current_set_values_locker.unlock();

	current_get_values_locker.lock();
	current_get_values.position = INVALID_ANGLE_VALUE;
	current_get_values_locker.unlock();

	reset_friction_clutch_counter = 0;
}

// attention: this not only sets watchdog, but also command_rx_timeout_enabled!
void Arm_segment::Set_watchdog_enabled(bool state)
{
	watchdog_enabled = state;
}

void Arm_segment::Set_pause_sending(bool pause)
{
	bool print_msg = (pause_sending != pause);
	pause_sending = pause;
	Init_frames();
	
	if (print_msg)
	{
		if (pause_sending == true)
		{
			printf("[Arm seg. #%d (%d)] control was disabled -- starting relay server\n", segment_nr, host_port);
		}
		else
		{
			printf("[Arm seg. #%d (%d)] control was enabled -- stopping relay server\n", segment_nr, host_port);
		}
	}
}

// receive from ecu or ui server
void Arm_segment::On_data_received(unsigned char* data, int len, char* from_remote_ip, int from_remote_port, int from_local_port)
{
	if (pause_sending == true && from_local_port == ui_server_base_port + segment_nr) 	// note: ip check would be possible here too, but it makes simulation/debugging tricky (if the sim runs on the same machine as the UI)
	{
		ecu_socket->send(data, len);
	}
	else if (from_local_port != ui_server_base_port + segment_nr)
	{
		current_get_values_locker.lock();
		uint16_t old_position = current_get_values.position;
		current_get_values_locker.unlock();
		//printf("[arm seg. #%d (%d)] Got fresh values, updating latest_rx_frame_time\n", segment_nr, host_port);
		latest_rx_frame_time = boost::posix_time::microsec_clock::local_time();
		
		// store decode result in an intermediate status frame, so decode errors won't
		// compromise our current get values
		Arm_status_frame received_values;
		if (Arm_frames::Bytes_to_status(data, &received_values))
		{		
			bool clutch_slipping = false;
				
			current_get_values_locker.lock();
			
			// write received values from intermediate frame to our current_get_values
			current_get_values.sequence_number = received_values.sequence_number;
			current_get_values.position = received_values.position;
			current_get_values.current = received_values.current;
			current_get_values.temperature = received_values.temperature;
			current_get_values.bitfield = received_values.bitfield;
			current_get_values.error_code = received_values.error_code;

			//DEBUG("## Segment %d (%d) received %d\n", segment_nr, host_port, current_get_values.position);
		
			uint16_t dposition = current_get_values.position - old_position;
			
			if ((current_get_values.bitfield & 0x04) != 0)
			{
				clutch_slipping = true;
			}
			
			current_get_values_locker.unlock();
			
			if (first_frame_received == false)
			{
				// get initial set value from first received frame -- safety measure
				first_frame_received = true;
				current_set_values_locker.lock();
				current_get_values_locker.lock();
				DEBUG("Received first frame with position %d\n", current_get_values.position);
				current_set_values.set_position = current_get_values.position;
				current_get_values_locker.unlock();
				current_set_values_locker.unlock();
			}
			
			// check if we are static when we shouldn't be, and if the time to reach 
			// out set point is reasonable -- otherwise stop motors
			if (abs(dposition) <= SEGMENT_STATIC_TOLERANCE)
			{
				//DEBUG("No joint movement detected - counter is %d\n", segment_static_counter);
				segment_static_counter++;
			}
			else
			{
				segment_static_counter = 0;
			}
			
			if (relay != 0)
			{
				relay->On_segment_receive(segment_nr);
				if (clutch_slipping == true)
				{
					relay->On_friction_clutch_slipped(segment_nr);
				}
			}
						
			if (ui_socket != 0)
			{
				// relay frame from ecu to ui
				ui_socket->send(data, len);
			}
		}
		else
		{
			printf("[Arm seg. #%d (%d)] Status frame has invalid format -- doing nothing.\n", segment_nr, host_port);
		}
	}
}

void Arm_segment::Reset_friction_clutch()
{
	printf("[Arm seg. #%d (%d)] Requested to reset friction clutch bit\n", segment_nr, host_port);
	reset_friction_clutch_counter = 10;	// send (at least) 10 frames with motor_enable disengaged so that the friction clutch resets
}

void Arm_segment::Feed_watchdog()
{
	latest_watchdog_time = boost::posix_time::microsec_clock::local_time();
}

void Arm_segment::Set_on_received_callback(IReceiveCallbackRelay* relay)
{
	this->relay = relay;
}

// ############ SETTERS & GETTERS ##############

// expects rad in range [0..2pi]
void Arm_segment::Set_position(float pos)
{
	if (!pause_sending && !receive_pause && pos==pos) 	// the pos==pos expression is a NaN check
	{
		bool changed = false;
		//DEBUG("[Arm seg. #%d] (%d) received arm set pos to %f\n", segment_nr, host_port, pos);	
		pos *= (180.0f/M_PI);  	// convert from rad to deg
		uint16_t new_set_pos = (uint16_t)((fmod((360.0 + pos), 360.0)) * ((double)max_input_value / 360.0));
		//DEBUG("[Arm seg. #%d]  this translates to %d\n", segment_nr, new_set_pos);

		current_set_values_locker.lock();
		
		//DEBUG("[Arm seg. #%d (%d)] Setting position %f, which is %d; current value: %d\n", segment_nr, host_port, pos, new_set_pos, current_set_values.set_position);
		// see if we have a new set value
		if (new_set_pos != current_set_values.set_position)
		{
			changed = true;
			latest_drive_command_time = boost::posix_time::microsec_clock::local_time();
			segment_static_counter = 0; 	// reset static counter so we can move without immediately being stopped again (stall prevention)
		}

		current_set_values.set_position = new_set_pos;
		current_set_values_locker.unlock();
		
		if (changed) 	// only actively enable motor if we have a new set value - the motor 
		{				// might have stalled (SEGMENT_STATIC_COUNTER), while we may constantly send the same value
			Set_motor_enable(true); 	// must call outside of lock since it locks on its own
		}
	}
	else
	{
		if (pos != pos) printf("[Arm seg. #%d (%d)] Received NaN in Set_position!", segment_nr, host_port);
	}
}

void Arm_segment::Set_check_for_static_motor(bool check)
{
	check_for_static_motor = check;
}

void Arm_segment::Set_motor_enable(bool enable)
{
	if (!pause_sending && !receive_pause)
	{
		current_set_values_locker.lock();
		if (enable)
		{
			current_set_values.bitfield |= (1 << 6);
		}
		else
		{
			current_set_values.bitfield &= ~(1 << 6);
		}
		current_set_values_locker.unlock();
	}
}

void Arm_segment::Force_motor_enable_once()
{
	force_motor_enable = true;
}

std::string Arm_segment::Get_name()
{
	return segment_name;
}


// returns rad in range [0..2pi]
float Arm_segment::Get_position()
{
	float ret = 0;
	current_get_values_locker.lock();
	if (current_get_values.position == INVALID_ANGLE_VALUE)
	{
		ret = std::numeric_limits<float>::quiet_NaN();
	}
	else
	{
		ret = (360.0f + (((double)current_get_values.position / max_input_value) * 360.0f));
		while (ret >= 360.0f)
		{
			ret -= 360.0f;
		}
		while (ret < 0)
		{
			ret += 360.0f;
		}
	}
	current_get_values_locker.unlock();

	//DEBUG("[Arm seg. #%d (%d)] requesting position, which is currently %d, that means %f deg\n", segment_nr, host_port, current_get_values.position, ret);
	
	ret *= M_PI/180.0f;
	//printf("#%d## (%d) - that translates to %f rad\n", segment_nr, host_port, ret);
	return ret;
}

float Arm_segment::Get_current()
{
	float ret = 0;
	current_get_values_locker.lock();
	ret = current_get_values.current;
	current_get_values_locker.unlock();
	return ret;
}

float Arm_segment::Get_temperature()
{
	float ret = 0;
	current_get_values_locker.lock();
	ret = current_get_values.temperature;
	ret -= 40; 	// convert to degrees celsius
	current_get_values_locker.unlock();
	return ret;
}

unsigned char Arm_segment::Get_error_code()
{
	unsigned char ret = 0;
	current_get_values_locker.lock();
	ret = current_get_values.error_code;
	current_get_values_locker.unlock();
	return ret;
}

unsigned char Arm_segment::Get_bitfield()
{
	unsigned char ret = 0;
	current_get_values_locker.lock();
	ret = current_get_values.bitfield;
	current_get_values_locker.unlock();
	return ret;
}

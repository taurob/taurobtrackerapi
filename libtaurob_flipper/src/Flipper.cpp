/**************************************************************************//**
 *
 * @file Flipper.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Library for communication with taurob gripper (claw)
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
 
 #include "libtaurob_flipper/libtaurob_flipper.h"

using namespace boost;

bool debug_output_enabled = true;

Flipper::Flipper(std::string host_ip, 
			 int host_port, 
			 bool start_sending_initially,
			 int max_input_value) : 
	host_ip(host_ip), 
	host_port(host_port),
	segment_static_counter(0),
	watchdog_enabled(false),
	reset_ecu(false),
	pause_sending(!start_sending_initially),
	max_input_value(max_input_value)
{ 
	Set_pause_sending(pause_sending);
}

Flipper::Flipper(std::string host_ip, 
			 int host_port, 
			 bool start_sending_initially) : 
	host_ip(host_ip), 
	host_port(host_port),
	segment_static_counter(0),
	watchdog_enabled(false),
	reset_ecu(false),
	pause_sending(!start_sending_initially),
	max_input_value(DEFAULT_MAX_INPUT_VALUE)
{ 
	Set_pause_sending(pause_sending);
}

Flipper::~Flipper() 
{
	sending_thread_running = false;
}

// ################# COMMUNICATION ####################

bool Flipper::Is_uptodate()
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

void Flipper::Print_timestamp_debug()
{
	// ptime to string.
	const std::string str_now = to_iso_extended_string(boost::posix_time::microsec_clock::local_time());
	const std::string str_time = to_iso_extended_string(latest_rx_frame_time);
	
	printf("\n\n### Timestamps: \n\tNow: %s\n\tLatest rx frame time: %s\n\n\n", str_now.c_str(), str_time.c_str());
}

bool Flipper::Watchdog_ok()
{
	bool ret = false;
	
	tTimeDuration dt = boost::posix_time::microsec_clock::local_time() - latest_watchdog_time;
	if (dt.total_milliseconds() <= WATCHDOG_MAX_TIME) ret = true;
			
	return ret;
}

// suppress sending for RESET_ECU_DURATION (typ. 400 ms) to reset modarm ecu
void Flipper::Reset_connection()
{
	reset_ecu = true;
	reset_ecu_timestamp = boost::posix_time::microsec_clock::local_time();
}

void Flipper::Check_if_allowed_to_drive()
{
	// first of all, see if we want to drive in the first place (check for motor enable bit)
	bool want_to_drive = false;
	current_set_values_locker.lock();
	if ((current_set_values.bitfield & 0x40) != 0) want_to_drive = true;
	current_set_values_locker.unlock();
	if (want_to_drive == false) return;

	// TODO: static counter disabled for now as u-ecu is currently unable to report position correctly
	// enable this as soon as valid flipper position is available
	/*
	if (segment_static_counter > SEGMENT_STATIC_COUNTER_MAX)
	{
		DEBUG("[Flipper] Static counter reached max, disabling movement\n");
		Set_motor_enable(false);
		segment_static_counter = 0;
		return;
	}
	*/
	if ((boost::posix_time::microsec_clock::local_time() - latest_drive_command_time).total_milliseconds() >= MAX_DRIVE_TIME)
	{
		DEBUG("[Flipper] Drive command timeout, disabling movement\n");
		Set_motor_enable(false);
		return;
	}

	// see when we received last status frame, stop robot if it's been too long
	// this is in addition to the watchdog
	if (Is_uptodate() == false)
	{
		DEBUG("[Flipper] Timeout receiving status frame from ECU, disabling movement\n");
		Set_motor_enable(false);
		return;
	}

	if (Watchdog_ok() == false)
	{
		DEBUG("[Flipper] Watchdog nok - set motor enable false\n");
		Set_motor_enable(false);
		if (receive_pause == false)
		{
			printf("[Flipper] No command received since more than 110ms (Watchdog timeout) - motors are disabled\n");
			receive_pause = true;
		}
		return;
	}
	else if (receive_pause == true)
	{
		receive_pause = false;
		printf("[Flipper] Command frame was received again, re-enabling motor movement.\n");
	}
}

void Flipper::Sending_thread()
{
	static bool command_rx_timeout_message = false;
	
	receive_pause = false;
	
	unsigned char sendbuffer[Flipper_frames::COMMAND_FRAME_LENGTH];
	
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
					
				current_set_values_locker.lock();
				current_tx_seqno = (current_tx_seqno + 1) % 255;
				current_set_values.sequence_number = current_tx_seqno;
				Flipper_frames::Command_to_bytes(current_set_values, sendbuffer);
				current_set_values_locker.unlock();
				
				ecu_socket->send(sendbuffer, Flipper_frames::COMMAND_FRAME_LENGTH);
					
				//DEBUG("Segment %d (%d) sent: %d\n", segment_nr, host_port, current_set_values.set_position);
			}
		}
		else
		{
			tTimeDuration dt = boost::posix_time::microsec_clock::local_time() - reset_ecu_timestamp;
			if (dt.total_milliseconds() > RESET_ECU_DURATION) 
			{
				reset_ecu = false;
				printf("[Flipper] Connection was reset. Resuming to send commands.\n");
			}
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(40));
	}
}

void Flipper::Run() 
{
	if (max_input_value == 0) max_input_value = DEFAULT_MAX_INPUT_VALUE;
	Init_frames();
	
	sending_thread_running = true;
	ecu_socket = new SocketUDPBinary(host_ip, host_port, host_port, Flipper_frames::STATUS_FRAME_LENGTH); // bind to same port as host_port, since ecu will send responses there
	ecu_socket->start_listen(this);

	//DEBUG("starting sending thread...\n");
	sending_thread = boost::thread(boost::bind(&Flipper::Sending_thread, this));
	//DEBUG("after thread creation. host ip: %s\n", host_ip.c_str());

	DEBUG("Flipper opened connection to ip %s, port %d\n", host_ip.c_str(), host_port);
}

void Flipper::Stop()
{
	sending_thread_running = false;
	
	DEBUG("[Flipper] Stopping - Waiting for threads to join...\n");
	sending_thread.join();
	DEBUG("[Flipper] ECUClient::run() completed\n");

	ecu_socket->stop_listen();
	delete ecu_socket;
	ecu_socket = 0;
}

void Flipper::Init_frames()
{
	// initialize internal frames to safe values - they are all set to 0 by default
	current_set_values_locker.lock();
	current_set_values.set_duty = 0;
	current_set_values.bitfield = 0;
	current_set_values_locker.unlock();

	current_get_values_locker.lock();
	current_get_values.position = INVALID_ANGLE_VALUE;
	current_get_values_locker.unlock();
}

// attention: this not only sets watchdog, but also command_rx_timeout_enabled!
void Flipper::Set_watchdog_enabled(bool state)
{
	watchdog_enabled = state;
}

void Flipper::Set_pause_sending(bool pause)
{
	bool print_msg = (pause_sending != pause);
	pause_sending = pause;
	Init_frames();
	
	if (print_msg)
	{
		if (pause_sending == true)
		{
			printf("[Flipper] control was disabled -- starting relay server\n");
		}
		else
		{
			printf("[Flipper] control was enabled -- stopping relay server\n");
		}
	}
}


void Flipper::Set_on_received_callback(void (*callback)())
{
	on_receive_callback = callback;
}


// receive from ecu
void Flipper::On_data_received(unsigned char* data, int len, char* from_remote_ip, int from_remote_port, int from_local_port)
{
	current_get_values_locker.lock();
	int16_t old_position = current_get_values.position;
	current_get_values_locker.unlock();
	
	latest_rx_frame_time = boost::posix_time::microsec_clock::local_time();
	
	// store decode result in an intermediate status frame, so decode errors won't
	// compromise our current get values
	Flipper_status_frame received_values;
	if (Flipper_frames::Bytes_to_status(data, &received_values))
	{		
		current_get_values_locker.lock();
		
		// write received values from intermediate frame to our current_get_values
		current_get_values.sequence_number = received_values.sequence_number;
		current_get_values.position = received_values.position;
		current_get_values.error_code = received_values.error_code;
	
		int16_t dposition = current_get_values.position - old_position;
				
		current_get_values_locker.unlock();
		
		if (first_frame_received == false)
		{
			// get initial set value from first received frame -- safety measure
			first_frame_received = true;
			current_set_values_locker.lock();
			current_get_values_locker.lock();
			DEBUG("Received first frame with position %d\n", current_get_values.position);
			current_set_values.set_duty = 0;
			current_get_values_locker.unlock();
			current_set_values_locker.unlock();
		}
		
		// check if we are static when we shouldn't be, and if the time to reach 
		// our set point is reasonable -- otherwise stop motors
		if (abs(dposition) <= SEGMENT_STATIC_TOLERANCE)
		{
			//DEBUG("No joint movement detected - counter is %d\n", segment_static_counter);
			segment_static_counter++;
		}
		else
		{
			segment_static_counter = 0;
		}
		
		if (on_receive_callback != 0)
		{
			on_receive_callback();
		}
	}
	else
	{
		printf("[Flipper] Status frame has invalid format -- doing nothing.\n");
	}
}


void Flipper::Feed_watchdog()
{
	latest_watchdog_time = boost::posix_time::microsec_clock::local_time();
}


// ############ SETTERS & GETTERS ##############

// expects fraction in range [-1.0..+1.0]
void Flipper::Set_duty(float duty)
{
	if (!pause_sending && !receive_pause && duty==duty) 	// the duty==duty expression is a NaN check
	{
		bool changed = false;
		DEBUG("[Flipper] received Flipper set duty %f\n", duty);	
		
		if (duty > 1.0f) duty = 1.0f;
		if (duty < -1.0f) duty = -1.0f;
		duty *= FLIPPER_MAX_DUTY;  	// convert from rad to deg
		
		//DEBUG("[Flipper]  this translates to %d\n", new_set_pos);

		current_set_values_locker.lock();
		
		//DEBUG("[Flipper] Setting position %f, which is %d; current value: %d\n", pos, new_set_pos, current_set_values.set_position);
		// see if we have a new set value
		if (duty != current_set_values.set_duty)
		{
			changed = true;
			latest_drive_command_time = boost::posix_time::microsec_clock::local_time();
			segment_static_counter = 0; 	// reset static counter so we can move without immediately being stopped again (stall prevention)
		}

		current_set_values.set_duty = duty;
		current_set_values_locker.unlock();
		
		if (changed) 	// only actively enable motor if we have a new set value - the motor 
		{				// might have stalled (SEGMENT_STATIC_COUNTER), while we may constantly send the same value
			Set_motor_enable(true); 	// must call outside of lock since it locks on its own
		}
	}
	else
	{
		if (duty != duty) printf("[Flipper] Received NaN in Set_duty!\n");
	}
}

void Flipper::Set_motor_enable(bool enable)
{
	//if (enable) printf("Set motor enable TRUE\n");
	//else printf("Set motor enable FALSE\n");
	
	if (!pause_sending && !receive_pause)
	{
		current_set_values_locker.lock();
		if (enable)
		{
			current_set_values.bitfield |= 0x40; 	// bit 0: motor enable, bit 1: rotation absolute mode, bit 2: grip absolute mode
		}
		else
		{
			current_set_values.bitfield &= ~(0x40);
		}
		current_set_values_locker.unlock();
	}
}

// returns rad in range [0..2pi]
float Flipper::Get_position()
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

	//DEBUG("[Flipper] requesting position, which is currently %d, that means %f deg\n", current_get_values.position, ret);
	
	ret *= M_PI/180.0f;
	//printf(" - that translates to %f rad\n", ret);
	return ret;
}


unsigned char Flipper::Get_error_code()
{
	unsigned char ret = 0;
	current_get_values_locker.lock();
	ret = current_get_values.error_code;
	current_get_values_locker.unlock();
	return ret;
}

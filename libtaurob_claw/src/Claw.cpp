/**************************************************************************//**
 *
 * @file Claw.cpp
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
 
 #include "libtaurob_claw/libtaurob_claw.h"

using namespace boost;

bool debug_output_enabled = false;

Claw::Claw(std::string host_ip, 
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

Claw::Claw(std::string host_ip, 
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

Claw::~Claw() 
{
	sending_thread_running = false;
}

// ################# COMMUNICATION ####################

bool Claw::Is_uptodate()
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

void Claw::Print_timestamp_debug()
{
	// ptime to string.
	const std::string str_now = to_iso_extended_string(boost::posix_time::microsec_clock::local_time());
	const std::string str_time = to_iso_extended_string(latest_rx_frame_time);
	
	printf("\n\n### Timestamps: \n\tNow: %s\n\tLatest rx frame time: %s\n\n\n", str_now.c_str(), str_time.c_str());
}

bool Claw::Watchdog_ok()
{
	bool ret = false;
	
	tTimeDuration dt = boost::posix_time::microsec_clock::local_time() - latest_watchdog_time;
	if (dt.total_milliseconds() <= WATCHDOG_MAX_TIME) ret = true;
			
	return ret;
}

// suppress sending for RESET_ECU_DURATION (typ. 400 ms) to reset modarm ecu
void Claw::Reset_connection()
{
	reset_ecu = true;
	reset_ecu_timestamp = boost::posix_time::microsec_clock::local_time();
}

void Claw::Check_if_allowed_to_drive()
{
	// first of all, see if we want to drive in the first place (check for motor enable bit)
	bool want_to_drive = false;
	current_set_values_locker.lock();
	if ((current_set_values.bitfield & 0x01) != 0) want_to_drive = true;
	current_set_values_locker.unlock();
	if (want_to_drive == false) return;

	if (segment_static_counter > SEGMENT_STATIC_COUNTER_MAX)
	{
		DEBUG("[Claw] Static counter reached max, disabling movement\n");
		Set_motor_enable(false);
		segment_static_counter = 0;
		return;
	}
	if ((boost::posix_time::microsec_clock::local_time() - latest_drive_command_time).total_milliseconds() >= MAX_DRIVE_TIME)
	{
		DEBUG("[Claw] Drive timeout, disabling movement\n");
		Set_motor_enable(false);
		return;
	}

	// see when we received last status frame, stop robot if it's been too long
	// this is in addition to the watchdog
	if (Is_uptodate() == false)
	{
		DEBUG("[Claw]  Receive timeout, disabling movement\n");
		Set_motor_enable(false);
		return;
	}

	if (Watchdog_ok() == false)
	{
		DEBUG("[Claw] Watchdog nok - set motor enable false\n");
		Set_motor_enable(false);
		if (receive_pause == false)
		{
			printf("[Claw] No command received since more than 110ms (Watchdog timeout) - motors are disabled\n");
			receive_pause = true;
		}
		return;
	}
	else if (receive_pause == true)
	{
		receive_pause = false;
		printf("[Claw] Command frame was received again, re-enabling motor movement.\n");
	}
}

void Claw::Sending_thread()
{
	static bool command_rx_timeout_message = false;
	
	receive_pause = false;
	
	unsigned char sendbuffer[Claw_frames::COMMAND_FRAME_LENGTH];
	
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
				Claw_frames::Command_to_bytes(current_set_values, sendbuffer);
				current_set_values_locker.unlock();
				
				ecu_socket->send(sendbuffer, Claw_frames::COMMAND_FRAME_LENGTH);
					
				//DEBUG("Segment %d (%d) sent: %d\n", segment_nr, host_port, current_set_values.set_position);
			}
		}
		else
		{
			tTimeDuration dt = boost::posix_time::microsec_clock::local_time() - reset_ecu_timestamp;
			if (dt.total_milliseconds() > RESET_ECU_DURATION) 
			{
				reset_ecu = false;
				printf("[Claw] Connection was reset. Resuming to send commands.\n");
			}
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(40));
	}
}

void Claw::Run() 
{
	if (max_input_value == 0) max_input_value = DEFAULT_MAX_INPUT_VALUE;
	Init_frames();
	
	sending_thread_running = true;
	ecu_socket = new SocketUDPBinary(host_ip, host_port, host_port, Claw_frames::STATUS_FRAME_LENGTH); // bind to same port as host_port, since ecu will send responses there
	ecu_socket->start_listen(this);

	//DEBUG("starting sending thread...\n");
	sending_thread = boost::thread(boost::bind(&Claw::Sending_thread, this));
	//DEBUG("after thread creation. host ip: %s\n", host_ip.c_str());

	DEBUG("Claw opened connection to ip %s, port %d\n", host_ip.c_str(), host_port);
}

void Claw::Stop()
{
	sending_thread_running = false;
	
	DEBUG("[Claw] Stopping - Waiting for threads to join...\n");
	sending_thread.join();
	DEBUG("[Claw] ECUClient::run() completed\n");

	ecu_socket->stop_listen();
	delete ecu_socket;
	ecu_socket = 0;
}

void Claw::Init_frames()
{
	// initialize internal frames to safe values - they are all set to 0 by default
	current_set_values_locker.lock();
	current_set_values.set_rotation = INVALID_ANGLE_VALUE;
	current_set_values.set_grip = INVALID_ANGLE_VALUE;
	current_set_values.bitfield = 0;
	current_set_values_locker.unlock();

	current_get_values_locker.lock();
	current_get_values.get_rotation = INVALID_ANGLE_VALUE;
	current_get_values.get_grip = INVALID_ANGLE_VALUE;
	current_get_values.force = 0;
	current_get_values_locker.unlock();
}

// attention: this not only sets watchdog, but also command_rx_timeout_enabled!
void Claw::Set_watchdog_enabled(bool state)
{
	watchdog_enabled = state;
}

void Claw::Set_pause_sending(bool pause)
{
	bool print_msg = (pause_sending != pause);
	pause_sending = pause;
	Init_frames();
	
	if (print_msg)
	{
		if (pause_sending == true)
		{
			printf("[Claw] control was disabled -- starting relay server\n");
		}
		else
		{
			printf("[Claw] control was enabled -- stopping relay server\n");
		}
	}
}


void Claw::Set_on_received_callback(void (*callback)())
{
	on_receive_callback = callback;
}


// receive from ecu
void Claw::On_data_received(unsigned char* data, int len, char* from_remote_ip, int from_remote_port, int from_local_port)
{
	current_get_values_locker.lock();
	int16_t old_rotation = current_get_values.get_rotation;
	int16_t old_grip = current_get_values.get_grip;
	current_get_values_locker.unlock();
	
	latest_rx_frame_time = boost::posix_time::microsec_clock::local_time();
	
	// store decode result in an intermediate status frame, so decode errors won't
	// compromise our current get values
	Claw_status_frame received_values;
	if (Claw_frames::Bytes_to_status(data, &received_values))
	{		
		current_get_values_locker.lock();
		
		// write received values from intermediate frame to our current_get_values
		current_get_values.sequence_number = received_values.sequence_number;
		current_get_values.get_rotation = received_values.get_rotation;
		current_get_values.get_grip = received_values.get_grip;
		current_get_values.force = received_values.force;
		current_get_values.error_code = received_values.error_code;
	
		int16_t drotation = current_get_values.get_rotation - old_rotation;
		int16_t dgrip = current_get_values.get_grip - old_grip;
				
		current_get_values_locker.unlock();
		
		if (first_frame_received == false)
		{
			// get initial set value from first received frame -- safety measure
			first_frame_received = true;
			current_set_values_locker.lock();
			current_get_values_locker.lock();
			DEBUG("Received first frame with positions %d, %d\n", current_get_values.get_rotation, current_get_values.get_grip);
			current_set_values.set_rotation = current_get_values.get_rotation;
			current_set_values.set_grip = current_get_values.get_grip;
			current_get_values_locker.unlock();
			current_set_values_locker.unlock();
		}
		
		// check if we are static when we shouldn't be, and if the time to reach 
		// our set point is reasonable -- otherwise stop motors
		if (abs(drotation) <= SEGMENT_STATIC_TOLERANCE && abs(dgrip) <= SEGMENT_STATIC_TOLERANCE)
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
		printf("[Claw] Status frame has invalid format -- doing nothing.\n");
	}
}


void Claw::Feed_watchdog()
{
	latest_watchdog_time = boost::posix_time::microsec_clock::local_time();
}


// ############ SETTERS & GETTERS ##############

// expects rad in range [0..2pi]
void Claw::Set_rotation(float rot)
{
	if (!pause_sending && !receive_pause && rot==rot) 	// the rot==rot expression is a NaN check
	{
		bool changed = false;
		DEBUG("[Claw] received claw set rot to %f\n", rot);	
		rot *= (180.0f/M_PI);  	// convert from rad to deg
		int16_t new_set_rot = (int16_t)((fmod((360.0 + rot), 360.0)) * ((double)max_input_value / 360.0));
		//DEBUG("[Claw  this translates to %d\n", new_set_pos);

		current_set_values_locker.lock();
		
		//DEBUG("[Claw] Setting position %f, which is %d; current value: %d\n", pos, new_set_pos, current_set_values.set_position);
		// see if we have a new set value
		if (new_set_rot != current_set_values.set_rotation)
		{
			changed = true;
			latest_drive_command_time = boost::posix_time::microsec_clock::local_time();
			segment_static_counter = 0; 	// reset static counter so we can move without immediately being stopped again (stall prevention)
		}

		current_set_values.set_rotation = new_set_rot;
		current_set_values_locker.unlock();
		
		if (changed) 	// only actively enable motor if we have a new set value - the motor 
		{				// might have stalled (SEGMENT_STATIC_COUNTER), while we may constantly send the same value
			Set_motor_enable(true); 	// must call outside of lock since it locks on its own
		}
	}
	else
	{
		if (rot != rot) printf("[Claw] Received NaN in Set_rotation!\n");
	}
}

// expects rad in range [0..2pi]
void Claw::Set_grip(float grip)
{
	if (!pause_sending && !receive_pause && grip==grip) 	// the grip==grip expression is a NaN check
	{
		bool changed = false;
		DEBUG("[Claw] received claw set grip to %f\n", grip);	
		grip *= (180.0f/M_PI);  	// convert from rad to deg
		int16_t new_set_grip = (uint16_t)((fmod((360.0 + grip), 360.0)) * ((double)max_input_value / 360.0));
		//DEBUG("[Claw  this translates to %d\n", new_set_pos);

		current_set_values_locker.lock();
		
		//DEBUG("[Claw] Setting position %f, which is %d; current value: %d\n", pos, new_set_pos, current_set_values.set_position);
		// see if we have a new set value
		if (new_set_grip != current_set_values.set_grip)
		{
			changed = true;
			latest_drive_command_time = boost::posix_time::microsec_clock::local_time();
			segment_static_counter = 0; 	// reset static counter so we can move without immediately being stopped again (stall prevention)
		}

		current_set_values.set_grip = new_set_grip;
		current_set_values_locker.unlock();
		
		if (changed) 	// only actively enable motor if we have a new set value - the motor 
		{				// might have stalled (SEGMENT_STATIC_COUNTER), while we may constantly send the same value
			Set_motor_enable(true); 	// must call outside of lock since it locks on its own
		}
	}
	else
	{
		if (grip != grip) printf("[Claw] Received NaN in Set_grip!\n");
	}
}

void Claw::Set_motor_enable(bool enable)
{
	//if (enable) printf("Set motor enable TRUE\n");
	//else printf("Set motor enable FALSE\n");
	
	if (!pause_sending && !receive_pause)
	{
		current_set_values_locker.lock();
		if (enable)
		{
			current_set_values.bitfield |= ((1 << 0) | (1 << 1) | (1 << 2)); 	// bit 0: motor enable, bit 1: rotation absolute mode, bit 2: grip absolute mode
		}
		else
		{
			current_set_values.bitfield &= ~((1 << 0) | (1 << 1) | (1 << 2));
		}
		current_set_values_locker.unlock();
	}
}

void Claw::Set_light(bool enable)
{
	current_set_values_locker.lock();
	if (enable)
	{
		current_set_values.bitfield |= (1 << 3);
	}
	else
	{
		current_set_values.bitfield &= ~(1 << 3);
	}
	current_set_values_locker.unlock();
}

// returns rad in range [0..2pi]
float Claw::Get_rotation()
{
	float ret = 0;
	current_get_values_locker.lock();
	if (current_get_values.get_rotation == INVALID_ANGLE_VALUE)
	{
		ret = std::numeric_limits<float>::quiet_NaN();
	}
	else
	{
		ret = (360.0f + (((double)current_get_values.get_rotation / max_input_value) * 360.0f));
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

	//DEBUG("[Claw] requesting position, which is currently %d, that means %f deg\n", current_get_values.position, ret);
	
	ret *= M_PI/180.0f;
	//printf(" - that translates to %f rad\n", ret);
	return ret;
}

// returns rad in range [0..2pi]
float Claw::Get_grip()
{
	float ret = 0;
	current_get_values_locker.lock();
	if (current_get_values.get_grip == INVALID_ANGLE_VALUE)
	{
		ret = std::numeric_limits<float>::quiet_NaN();
	}
	else
	{
		ret = (360.0f + (((double)current_get_values.get_grip / max_input_value) * 360.0f));
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

	//DEBUG("[Claw] requesting position, which is currently %d, that means %f deg\n", current_get_values.position, ret);
	
	ret *= M_PI/180.0f;
	//printf(" - that translates to %f rad\n", ret);
	return ret;
}

// returns uint between 0 and 100 for force measurement
float Claw::Get_force()
{
	float ret = 0;
	current_get_values_locker.lock();
	ret = (float)((((double)current_get_values.force - 150.0) / 800.0) * 100.0);
	current_get_values_locker.unlock();
	return ret;
}

unsigned char Claw::Get_error_code()
{
	unsigned char ret = 0;
	current_get_values_locker.lock();
	ret = current_get_values.error_code;
	current_get_values_locker.unlock();
	return ret;
}

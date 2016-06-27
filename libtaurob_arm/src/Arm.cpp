/**************************************************************************//**
 *
 * @file Arm.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Library for communication with taurob arm
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
 
#include <libtaurob_arm/libtaurob_arm.h>

Arm::Arm(Arm_config configuration, bool control_enabled_initially) : on_receive_callback(0)
{
	config_successful = false;
	
	int config_size = (int)configuration.joint_ips.size();
	bool consistent = true;
	
	// check if the configuration is consistent
	if (configuration.joint_names.size() != config_size)
	{
		printf("taurob Arm config error: the number of specified joint names (%d) does not match the number of specified IPs (%d)!\n", 
				(int)configuration.joint_names.size(),
				config_size);
		consistent = false;
	}
	
	if (configuration.joint_ports.size() != config_size)
	{
		printf("taurob Arm config error: the number of specified joint ports (%d) does not match the number of specified IPs (%d)!\n", 
				(int)configuration.joint_ports.size(),
				config_size);
		consistent = false;
	}
	
	if (consistent == true)
	{
		config_successful = true;
		for (int i = 0; i < configuration.joint_ips.size(); i++)
		{
			Arm_segment* seg = new Arm_segment(configuration.joint_names[i], 
											   configuration.joint_ips[i], 
											   configuration.joint_ports[i], 
											   i, 
											   control_enabled_initially,									   
											   Arm_segment::DEFAULT_MAX_INPUT_VALUE,
											   Arm_segment::DEFAULT_UI_SERVER_BASE_PORT);
			seg->Set_on_received_callback(this);
			segments.push_back(seg);
		}
	}
}

Arm::~Arm()
{
	for (int i = 0; i < segments.size(); i++)
	{
		delete segments[i];
	}
}

void Arm::On_segment_receive(int segment_nr)
{
	if (on_receive_callback != 0)
	{
		on_receive_callback(segment_nr);
	}
}


void Arm::On_friction_clutch_slipped(int segment_nr)
{
	if (on_friction_clutch_slipped_callback != 0)
	{
		on_friction_clutch_slipped_callback(segment_nr);
	}
}

void Arm::Reset_friction_clutch()
{
	for (int i = 0; i < segments.size(); i++)
	{
		segments[i]->Reset_friction_clutch();
	}
}


bool Arm::Is_uptodate()
{
	bool ret = true;
	bool notified = false;
	
	for (int i = 0; i < segments.size(); i++)
	{
		ret = ret & segments[i]->Is_uptodate(); 	// logical AND so that if one segment isn't uptodate, the whole arm isn't either
		if (segments[i]->Is_uptodate() == false && segments[i]->Watchdog_ok() == true && 
		    (boost::posix_time::microsec_clock::local_time() - last_notuptodate_notification).total_milliseconds() > MIN_UPTODATE_WARNING_INTERVAL)
		{
			notified = true;
			printf("[Arm seg. #%d] WARNING: Watchdog is ok, but no current values can be received! Connection to this segment will be reset!\n", i);
			segments[i]->Print_timestamp_debug();
			segments[i]->Reset_connection();
		}
	} 
	
	if (notified) last_notuptodate_notification = boost::posix_time::microsec_clock::local_time();
		
	return ret;
}

bool Arm::Watchdog_ok()
{
	bool ret = true;	
	for (int i = 0; i < segments.size(); i++)
	{
		if (segments[i]->Watchdog_ok() == false)
		{
			ret = false;
			break;
		}
	}
	return ret;
}

int Arm::Get_segment_count()
{
	return segments.size();
}

void Arm::Run()
{
	if (config_successful == true)
	{
		for (int i = 0; i < segments.size(); i++)
		{
			segments[i]->Run();
		}
	}
	else
	{
		printf("Cannot run taurob Arm: invalid configuration!\n");
	}
}

void Arm::Stop()
{
	for (int i = 0; i < segments.size(); i++)
	{
		segments[i]->Stop();
	}
}
   
void Arm::Feed_watchdog()
{
	for (int i = 0; i < segments.size(); i++)
	{
		segments[i]->Feed_watchdog();
	}
}

void Arm::Set_on_received_callback(void (*callback)(int segment_nr))
{
	on_receive_callback = callback;
}

void Arm::Set_on_friction_clutch_slipped_callback(void (*callback)(int segment_nr))
{
	on_friction_clutch_slipped_callback = callback;
}

void Arm::Set_watchdog_enabled(bool state)
{
	for (int i = 0; i < segments.size(); i++)
	{
		segments[i]->Set_watchdog_enabled(state);
	}
}

void Arm::Set_pause_sending(bool pause)
{
	for (int i = 0; i < segments.size(); i++)
	{
		segments[i]->Set_pause_sending(pause);
	}
}

std::string Arm::Get_segment_name(int segment)
{
	std::string ret;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_name();
	}
	return ret;
}

float Arm::Get_position(int segment)
{
	float ret = 0;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_position();
	}
	else
	{
		printf("## ERROR: tried to get position of segment %d, but we only have %d!", segment, (int)segments.size());
	}
	
	return ret;
}

float Arm::Get_current(int segment)
{
	float ret = 0;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_current();
	}
	return ret;
}

float Arm::Get_temperature(int segment)
{
	float ret = 0;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_temperature();
	}
	return ret;
}
 
unsigned char Arm::Get_error_code(int segment)
{
	unsigned char ret = 0;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_error_code();
	}
	return ret;
}

unsigned char Arm::Get_bitfield(int segment)
{
	float ret = 0;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_bitfield();
	}
	return ret;
}

void Arm::Set_position(int segment, float pos)
{
	if (segment < segments.size())
	{			
		segments[segment]->Set_position(pos);
	}
	else
	{
		printf("## ERROR: tried to set position of segment %d, but we only have %d!", segment, (int)segments.size());
	}
}

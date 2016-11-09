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

Flipper::Flipper(std::string ip_address, int port, bool control_enabled_initially) 
	  : Arm(Flipper::Build_arm_config(ip_address, port), control_enabled_initially)
{ 
	backup_flipper_pos = 0;
	FILE* backup_flipper_file = fopen("flipper_pos.txt", "r");
	if (backup_flipper_file != 0)
	{
		fscanf(backup_flipper_file, "%f", &backup_flipper_pos);
		fclose(backup_flipper_file);
		printf("[Flipper] Initializing flipper with backed-up position %f\n", backup_flipper_pos);
	}
}

Arm_config Flipper::Build_arm_config(std::string ip_address, int port)
{
	Arm_config config;
	config.joint_names.push_back("flipper_front");
	config.joint_ips.push_back(ip_address);
	config.joint_ports.push_back(port);
	config.joint_channels.push_back(0);
	return config;
}

void Flipper::On_segment_receive(int segment_nr)
{
	FILE* backup_flipper_file = fopen("flipper_pos.txt", "w+");
	if (backup_flipper_file != 0)
	{
		fprintf(backup_flipper_file, "%f", Get_position());
		fclose(backup_flipper_file);
	}
}

void Flipper::Force_motor_enable_once()
{
	Arm::Force_motor_enable_once(0);
}

std::string Flipper::Get_segment_name()
{
	return Arm::Get_segment_name(0);
}

float Flipper::Get_position()
{
	float ret = Arm::Get_position(0);
	ret += backup_flipper_pos;
	
	// Get_position is expected to return [0..2pi], make sure that's respected
	while (ret > 2 * M_PI)
	{
		ret -= 2 * M_PI;
	}
	return ret;
}

float Flipper::Get_current()
{
	return Arm::Get_current(0);
}

float Flipper::Get_temperature()
{
	return Arm::Get_temperature(0);
}
 
unsigned char Flipper::Get_error_code()
{
	return Arm::Get_error_code(0);
}

unsigned char Flipper::Get_bitfield()
{
	return Arm::Get_bitfield(0);
}

void Flipper::Set_position(float pos)
{
	printf("Received set position: %f, backup pos: %f\n", pos, backup_flipper_pos);
	float setpos = pos - backup_flipper_pos;
	
	// Arm::Set_position expects [0..2pi], make sure that's expected
	while (setpos < 0)
	{
		setpos += 2.0f * M_PI;
	}
	printf("setting: %f\n", setpos);
	Arm::Set_position(0, setpos);
}

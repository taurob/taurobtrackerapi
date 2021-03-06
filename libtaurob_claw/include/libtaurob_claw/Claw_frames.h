/**************************************************************************//**
 *
 * @file Claw_frames.h
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Library for communication with taurob gripper (claw), protocol definitions
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
 
#ifndef CLAWFRAME_H_
#define CLAWFRAME_H_

#include <string>
#include <ctype.h>
#include <inttypes.h>
#include <libtaurob_tools/Debug.h>
#include <libtaurob_tools/ParseException.h>

typedef struct Claw_status_frame_
{
	unsigned char sequence_number;
	uint16_t get_rotation, get_grip, force;
	unsigned char error_code;
	
	// constructor to have all values at 0 as default
	Claw_status_frame_() : sequence_number(0), get_rotation(0), get_grip(0), force(0), error_code(0) {}
} Claw_status_frame;

typedef struct Claw_command_frame_
{
	unsigned char sequence_number, bitfield;
	int16_t set_rotation, set_grip;
	
	// constructor to have all values at 0 as default
	Claw_command_frame_() : sequence_number(0), bitfield(0), set_rotation(0), set_grip(0) {}
} Claw_command_frame;

class Claw_frames
{
	public:
		static const int COMMAND_FRAME_LENGTH = 7;
		static const int STATUS_FRAME_LENGTH = 9;
		
		static bool Bytes_to_status(unsigned char* frame, Claw_status_frame* ret);
		static void Status_to_bytes(Claw_status_frame frame, unsigned char* dest);
		
		static Claw_command_frame Bytes_to_command(unsigned char* frame);
		static void Command_to_bytes(Claw_command_frame frame, unsigned char* dest);

	private:
		static uint16_t Calc_CRC(uint16_t init, int len, unsigned char* src);
		static float hex_to_float(char* hexdump);
		static void float_to_hex(float num, char* dest);
		static float uint32_to_float(uint32_t num);
};

#endif

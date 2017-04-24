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
 
#ifndef BASEFRAME_H_
#define BASEFRAME_H_

#include <string>
#include <boost/regex.hpp>
#include <libtaurob_tools/ParseException.h>
#include <ctype.h>

typedef struct Base_status_frame_
{
	uint sequence_number;
	int BLDC1_speed_get;
	int BLDC2_speed_get;
	uint BLDC1_current;
	uint BLDC2_current;
	int distance_travelled_1;
	int distance_travelled_2;
	uint gripper_pos_get;
	uint manipulator_angle;
	uint voltage;
	uint temp1;
	uint temp2;
	uint temp3;
	uint temp4;
	uint pitch;
	uint roll;
	uint error_code;
	uint aux_in;
    uint airflow;
	
	// constructor to have all values at 0 as default
	Base_status_frame_() : sequence_number(0), BLDC1_speed_get(0), BLDC2_speed_get(0), 
									BLDC1_current(0), BLDC2_current(0), distance_travelled_1(0),
									distance_travelled_2(0), gripper_pos_get(0), manipulator_angle(0),
									voltage(0), temp1(0), temp2(0), temp3(0), temp4(0), pitch(0),
                                    roll(0), error_code(0), aux_in(0), airflow(0) {}
} Base_status_frame;

typedef struct Base_command_frame_
{
	uint sequence_number;
	int vector_x;
	int vector_y;
	uint gripper_pos_set;
	uint manipulator_pos_set;
	int overturn_angle_front;
	int overturn_angle_back;
	int overturn_angle_left;
	int overturn_angle_right;
	int ot_threshold_stage1;
	int ot_threshold_stage2;
	uint bitfield;
	
	// constructor to have all values at 0 as default
	Base_command_frame_() : sequence_number(0), vector_x(0), vector_y(0), gripper_pos_set(0),
									manipulator_pos_set(0), overturn_angle_front(0), overturn_angle_back(0),
									overturn_angle_left(0), overturn_angle_right(0), ot_threshold_stage1(0),
									ot_threshold_stage2(0), bitfield(0) {}
} Base_command_frame;

class Base_frames
{
	public:
        static bool String_to_status(std::string frame, Base_status_frame* outputMsg);
        static std::string Command_to_string(Base_command_frame frame);
};

#endif /* BASEFRAME_H_ */

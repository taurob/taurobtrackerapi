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
 
#include "libtaurob_base/Base_frames.h"

bool Base_frames::String_to_status(std::string ecu_msg, Base_status_frame* outputMsg)
{
	bool success = false;	

    int conversions = sscanf(ecu_msg.data(),
            "2 SQ:%u S1:%d S2:%d C1:%u C2:%u D1:%d D2:%d GP:%u M1:%u BA:%u T1:%u T2:%u T3:%u T4:%u PI:%u RO:%u EC:%u AX:%u AC:%u",
            (unsigned int*)&outputMsg->sequence_number,
            (unsigned int*)&outputMsg->BLDC1_speed_get,
            (unsigned int*)&outputMsg->BLDC2_speed_get,
            (unsigned int*)&outputMsg->BLDC1_current,
            (unsigned int*)&outputMsg->BLDC2_current,
            (unsigned int*)&outputMsg->distance_travelled_1,
            (unsigned int*)&outputMsg->distance_travelled_2,
            (unsigned int*)&outputMsg->gripper_pos_get,
            (unsigned int*)&outputMsg->manipulator_angle,
            (unsigned int*)&outputMsg->voltage,
            (unsigned int*)&outputMsg->temp1,
            (unsigned int*)&outputMsg->temp2,
            (unsigned int*)&outputMsg->temp3,
            (unsigned int*)&outputMsg->temp4,
            (unsigned int*)&outputMsg->pitch,
            (unsigned int*)&outputMsg->roll,
            (unsigned int*)&outputMsg->error_code,
            (unsigned int*)&outputMsg->aux_in,
            (unsigned int*)&outputMsg->airflow);

    if (conversions != 19)
    {
        printf("%s", ecu_msg.data());
        std::string errorMsg = "String Message Format do not match with ECUOutputMsg Format. "
                "Please check the regular expression and message format.";
        printf("%s", errorMsg.c_str());
    }
    else
    {
        success = true;
    }

	
	return success;
}

std::string Base_frames::Command_to_string(Base_command_frame ui_msg)
{
	char result[256];

    sprintf(result,
        "1 SQ:%u XV:%d YV:%d GS:%u MP:%u OF:%d OB:%d OL:%d OR:%d S1:%d S2:%d RC:1800 PC:1800 BF:%u",
        (unsigned int)ui_msg.sequence_number,
        (unsigned int)ui_msg.vector_x,
        (unsigned int)ui_msg.vector_y,
        (unsigned int)ui_msg.gripper_pos_set,
        (unsigned int)ui_msg.manipulator_pos_set,
        (int)ui_msg.overturn_angle_front,
        (int)ui_msg.overturn_angle_back,
        (int)ui_msg.overturn_angle_left,
        (int)ui_msg.overturn_angle_right,
        (int)ui_msg.ot_threshold_stage1,
        (int)ui_msg.ot_threshold_stage2,
        (unsigned int)ui_msg.bitfield);

	return result;
}

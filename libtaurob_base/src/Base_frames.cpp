/**************************************************************************//**
 *
 * @file Base_frames.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Library for communication with taurob base ECU, protocol handling
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
 
#include "libtaurob_base/Base_frames.h"

float Base_frames::hex_to_float(char* hexdump)
{
	uint32_t num = 0;
	sscanf(hexdump, "%08X", &num);

	return uint32_to_float(num);
}

float Base_frames::uint32_to_float(uint32_t num)
{
	// reverse endianness
	num = (((num & 0x000000FF) << 8*3) | ((num & 0x0000FF00) << 8) | ((num & 0x00FF0000) >> 8) | ((num & 0xFF000000) >> 8*3));

	float ret = *((float*)&num);
	return ret;
}

void Base_frames::float_to_hex(float num, char* dest)
{
	uint8_t* pnum = (uint8_t*)(&num);
	sprintf(dest, "%02X%02X%02X%02X", *(pnum+0), *(pnum+1), *(pnum+2), *(pnum+3));
}

std::string Base_frames::Status_to_string(Base_status_frame outputMsg, int protocol_version)
{
	char result[512];	// buffer for result string
	
	if (protocol_version == 2)
	{
	char acc_x[9], acc_y[9], acc_z[9];
	char gyro_x[9], gyro_y[9], gyro_z[9];
	char mag_x[9], mag_y[9], mag_z[9];
	char pos_x[9], pos_y[9], pos_z[9];
	int aircount;

	float_to_hex(outputMsg.acc_x, acc_x);
	float_to_hex(outputMsg.acc_y, acc_y);
	float_to_hex(outputMsg.acc_z, acc_z);
	float_to_hex(outputMsg.gyro_x, gyro_x);
	float_to_hex(outputMsg.gyro_y, gyro_y);
	float_to_hex(outputMsg.gyro_z, gyro_z);
	float_to_hex(outputMsg.mag_x, mag_x);
	float_to_hex(outputMsg.mag_y, mag_y);
	float_to_hex(outputMsg.mag_z, mag_z);
	float_to_hex(outputMsg.pos_x, pos_x);
	float_to_hex(outputMsg.pos_y, pos_y);
	float_to_hex(outputMsg.pos_z, pos_z);

	sprintf(result,
			"2 SQ:%u S1:%d S2:%d C1:%u C2:%u D1:%d D2:%d GP:%u M1:%u BA:%u T1:%u T2:%u T3:%u T4:%u PI:%u RO:%u YW:%u CX:%s CY:%s CZ:%s GX:%s GY:%s GZ:%s MX:%s MY:%s MZ:%s PX:%s PY:%s PZ:%s EC:%u AX:%u AC:%u",
			(unsigned int)outputMsg.sequence_number,
			(unsigned int)outputMsg.BLDC1_speed_get,
			(unsigned int)outputMsg.BLDC2_speed_get,
			(unsigned int)outputMsg.BLDC1_current,
			(unsigned int)outputMsg.BLDC2_current,
			(unsigned int)outputMsg.distance_travelled_1,
			(unsigned int)outputMsg.distance_travelled_2,
			(unsigned int)outputMsg.gripper_pos_get,
			(unsigned int)outputMsg.manipulator_angle,
			(unsigned int)outputMsg.voltage,
			(unsigned int)outputMsg.temp1,
			(unsigned int)outputMsg.temp2,
			(unsigned int)outputMsg.temp3,
			(unsigned int)outputMsg.temp4,
			(unsigned int)outputMsg.pitch,
			(unsigned int)outputMsg.roll,
			(unsigned int)outputMsg.yaw,
			acc_x,
			acc_y,
			acc_z,
			gyro_x,
			gyro_y,
			gyro_z,
			mag_x,
			mag_y,
			mag_z,
			pos_x,
			pos_y,
			pos_z,
			(unsigned int)outputMsg.error_code,
			(unsigned int)outputMsg.aux_in,
			aircount);
	}
	else if (protocol_version == 2)
	{
	char acc_x[9], acc_y[9], acc_z[9];
	char gyro_x[9], gyro_y[9], gyro_z[9];
	char mag_x[9], mag_y[9], mag_z[9];
	char pos_x[9], pos_y[9], pos_z[9];

	float_to_hex(outputMsg.acc_x, acc_x);
	float_to_hex(outputMsg.acc_y, acc_y);
	float_to_hex(outputMsg.acc_z, acc_z);
	float_to_hex(outputMsg.gyro_x, gyro_x);
	float_to_hex(outputMsg.gyro_y, gyro_y);
	float_to_hex(outputMsg.gyro_z, gyro_z);
	float_to_hex(outputMsg.mag_x, mag_x);
	float_to_hex(outputMsg.mag_y, mag_y);
	float_to_hex(outputMsg.mag_z, mag_z);
	float_to_hex(outputMsg.pos_x, pos_x);
	float_to_hex(outputMsg.pos_y, pos_y);
	float_to_hex(outputMsg.pos_z, pos_z);

	sprintf(result,
			"2 SQ:%u S1:%d S2:%d C1:%u C2:%u D1:%d D2:%d GP:%u M1:%u BA:%u T1:%u T2:%u T3:%u T4:%u PI:%u RO:%u YW:%u CX:%s CY:%s CZ:%s GX:%s GY:%s GZ:%s MX:%s MY:%s MZ:%s PX:%s PY:%s PZ:%s EC:%u AX:%u",
			(unsigned int)outputMsg.sequence_number,
			(unsigned int)outputMsg.BLDC1_speed_get,
			(unsigned int)outputMsg.BLDC2_speed_get,
			(unsigned int)outputMsg.BLDC1_current,
			(unsigned int)outputMsg.BLDC2_current,
			(unsigned int)outputMsg.distance_travelled_1,
			(unsigned int)outputMsg.distance_travelled_2,
			(unsigned int)outputMsg.gripper_pos_get,
			(unsigned int)outputMsg.manipulator_angle,
			(unsigned int)outputMsg.voltage,
			(unsigned int)outputMsg.temp1,
			(unsigned int)outputMsg.temp2,
			(unsigned int)outputMsg.temp3,
			(unsigned int)outputMsg.temp4,
			(unsigned int)outputMsg.pitch,
			(unsigned int)outputMsg.roll,
			(unsigned int)outputMsg.yaw,
			acc_x,
			acc_y,
			acc_z,
			gyro_x,
			gyro_y,
			gyro_z,
			mag_x,
			mag_y,
			mag_z,
			pos_x,
			pos_y,
			pos_z,
			(unsigned int)outputMsg.error_code,
			(unsigned int)outputMsg.aux_in);
	}
	else if (protocol_version == 1)
	{
		sprintf(result,
			"2 SQ:%u S1:%d S2:%d C1:%u C2:%u D1:%d D2:%d GP:%u M1:%u BA:%u T1:%u T2:%u T3:%u T4:%u PI:%u RO:%u YW:%u EC:%u AX:%u",
			(unsigned int)outputMsg.sequence_number,
			(unsigned int)outputMsg.BLDC1_speed_get,
			(unsigned int)outputMsg.BLDC2_speed_get,
			(unsigned int)outputMsg.BLDC1_current,
			(unsigned int)outputMsg.BLDC2_current,
			(unsigned int)outputMsg.distance_travelled_1,
			(unsigned int)outputMsg.distance_travelled_2,
			(unsigned int)outputMsg.gripper_pos_get,
			(unsigned int)outputMsg.manipulator_angle,
			(unsigned int)outputMsg.voltage,
			(unsigned int)outputMsg.temp1,
			(unsigned int)outputMsg.temp2,
			(unsigned int)outputMsg.temp3,
			(unsigned int)outputMsg.temp4,
			(unsigned int)outputMsg.pitch,
			(unsigned int)outputMsg.roll,
			(unsigned int)outputMsg.yaw,
			(unsigned int)outputMsg.error_code,
			(unsigned int)outputMsg.aux_in);
	}
	else
	{
		sprintf(result,
			"2 SQ:%u S1:%d S2:%d C1:%u C2:%u D1:%d D2:%d GP:%u M1:%u BA:%u T1:%u T2:%u T3:%u T4:%u PI:%u RO:%u EC:%u AX:%u AC:0",
			(unsigned int)outputMsg.sequence_number,
			(unsigned int)outputMsg.BLDC1_speed_get,
			(unsigned int)outputMsg.BLDC2_speed_get,
			(unsigned int)outputMsg.BLDC1_current,
			(unsigned int)outputMsg.BLDC2_current,
			(unsigned int)outputMsg.distance_travelled_1,
			(unsigned int)outputMsg.distance_travelled_2,
			(unsigned int)outputMsg.gripper_pos_get,
			(unsigned int)outputMsg.manipulator_angle,
			(unsigned int)outputMsg.voltage,
			(unsigned int)outputMsg.temp1,
			(unsigned int)outputMsg.temp2,
			(unsigned int)outputMsg.temp3,
			(unsigned int)outputMsg.temp4,
			(unsigned int)outputMsg.pitch,
			(unsigned int)outputMsg.roll,
			(unsigned int)outputMsg.error_code,
			(unsigned int)outputMsg.aux_in);
	}
	return result;
}

 bool Base_frames::String_to_status(std::string ecu_msg, int protocol_version, Base_status_frame* outputMsg)
{
	bool success = false;
	
	if (protocol_version == 3)
	{
		uint32_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, pos_x, pos_y, pos_z, aircount;
		
		int conversions = sscanf(ecu_msg.data(),
				"2 SQ:%u S1:%d S2:%d C1:%u C2:%u D1:%d D2:%d GP:%u M1:%u BA:%u T1:%u T2:%u T3:%u T4:%u PI:%u RO:%u YW:%u CX:%08X CY:%08X CZ:%08X GX:%08X GY:%08X GZ:%08X MX:%08X MY:%08X MZ:%08X PX:%X PY:%X PZ:%X EC:%u AX:%u AC:%u",
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
				(unsigned int*)&outputMsg->yaw,
				&acc_x,
				&acc_y,
				&acc_z,
				&gyro_x,
				&gyro_y,
				&gyro_z,
				&mag_x,
				&mag_y,
				&mag_z,
				&pos_x,
				&pos_y,
				&pos_z,
				(unsigned int*)&outputMsg->error_code,
				(unsigned int*)&outputMsg->aux_in,
				(unsigned int*)&aircount);

		if (conversions != 32)
		{
			DEBUG("%s", ecu_msg.data());
			std::string errorMsg = "String Message Format do not match with ECUOutputMsg Format. "
					"Please check the regular expression and message format.";
			DEBUG("%s", errorMsg.c_str());
		}
		else
		{
			success = true;
		}

		outputMsg->acc_x = uint32_to_float(acc_x);
		outputMsg->acc_y = uint32_to_float(acc_y);
		outputMsg->acc_z = uint32_to_float(acc_z);
		outputMsg->gyro_x = uint32_to_float(gyro_x);
		outputMsg->gyro_y = uint32_to_float(gyro_y);
		outputMsg->gyro_z = uint32_to_float(gyro_z);
		outputMsg->mag_x = uint32_to_float(mag_x);
		outputMsg->mag_y = uint32_to_float(mag_y);
		outputMsg->mag_z = uint32_to_float(mag_z);
		outputMsg->pos_x = uint32_to_float(pos_x);
		outputMsg->pos_y = uint32_to_float(pos_y);
		outputMsg->pos_z = uint32_to_float(pos_z);
	}
	else if (protocol_version == 2)
	{
		uint32_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, pos_x, pos_y, pos_z;
		
		int conversions = sscanf(ecu_msg.data(),
				"2 SQ:%u S1:%d S2:%d C1:%u C2:%u D1:%d D2:%d GP:%u M1:%u BA:%u T1:%u T2:%u T3:%u T4:%u PI:%u RO:%u YW:%u CX:%08X CY:%08X CZ:%08X GX:%08X GY:%08X GZ:%08X MX:%08X MY:%08X MZ:%08X PX:%X PY:%X PZ:%X EC:%u AX:%u",
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
				(unsigned int*)&outputMsg->yaw,
				&acc_x,
				&acc_y,
				&acc_z,
				&gyro_x,
				&gyro_y,
				&gyro_z,
				&mag_x,
				&mag_y,
				&mag_z,
				&pos_x,
				&pos_y,
				&pos_z,
				(unsigned int*)&outputMsg->error_code,
				(unsigned int*)&outputMsg->aux_in);

		if (conversions != 31)
		{
			DEBUG("%s", ecu_msg.data());
			std::string errorMsg = "String Message Format do not match with ECUOutputMsg Format. "
					"Please check the regular expression and message format.";
			DEBUG("%s", errorMsg.c_str());
		}
		else
		{
			success = true;
		}

		outputMsg->acc_x = uint32_to_float(acc_x);
		outputMsg->acc_y = uint32_to_float(acc_y);
		outputMsg->acc_z = uint32_to_float(acc_z);
		outputMsg->gyro_x = uint32_to_float(gyro_x);
		outputMsg->gyro_y = uint32_to_float(gyro_y);
		outputMsg->gyro_z = uint32_to_float(gyro_z);
		outputMsg->mag_x = uint32_to_float(mag_x);
		outputMsg->mag_y = uint32_to_float(mag_y);
		outputMsg->mag_z = uint32_to_float(mag_z);
		outputMsg->pos_x = uint32_to_float(pos_x);
		outputMsg->pos_y = uint32_to_float(pos_y);
		outputMsg->pos_z = uint32_to_float(pos_z);
	}
	else if (protocol_version == 1)
	{
		int conversions = sscanf(ecu_msg.data(),
				"2 SQ:%u S1:%d S2:%d C1:%u C2:%u D1:%d D2:%d GP:%u M1:%u BA:%u T1:%u T2:%u T3:%u T4:%u PI:%u RO:%u YW:%u EC:%u AX:%u",
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
				(unsigned int*)&outputMsg->yaw,
				(unsigned int*)&outputMsg->error_code,
				(unsigned int*)&outputMsg->aux_in);

		if (conversions != 19)
		{
			DEBUG("%s", ecu_msg.data());
			std::string errorMsg = "String Message Format do not match with ECUOutputMsg Format. "
					"Please check the regular expression and message format.";
			DEBUG("%s", errorMsg.c_str());
		}
		else
		{
			success = true;
		}
	}
	else
	{
		unsigned int aircount = 0; 	// ignore
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
				&aircount);

		if (conversions != 19)
		{
			DEBUG("%s", ecu_msg.data());
			std::string errorMsg = "String Message Format do not match with ECUOutputMsg Format. "
					"Please check the regular expression and message format.";
			DEBUG("%s", errorMsg.c_str());
		}
		else
		{
			success = true;
		}
	}
	
	return success;
}

std::string Base_frames::Command_to_string(Base_command_frame ui_msg, int protocol_version)
{
	char result[256];

	if (protocol_version == 1 || protocol_version == 2)
	{
		sprintf(result,
			"1 SQ:%u XV:%d YV:%d GS:%u MP:%u OF:%d OB:%d OL:%d OR:%d S1:%d S2:%d BF:%u",
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
	}
	else
	{
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
	}

	return result;
}

Base_command_frame Base_frames::String_to_command(std::string ui_msg, int protocol_version)
{
	Base_command_frame outputMsg;
	
	if (protocol_version == 1 || protocol_version == 2)
	{
		int convertions = sscanf(ui_msg.data(),
				"1 SQ:%u XV:%d YV:%d GS:%u MP:%u OF:%d OB:%d OL:%d OR:%d S1:%d S2:%d BF:%u",
				(unsigned int*)&outputMsg.sequence_number,
				(unsigned int*)&outputMsg.vector_x,
				(unsigned int*)&outputMsg.vector_y,
				(unsigned int*)&outputMsg.gripper_pos_set,
				(unsigned int*)&outputMsg.manipulator_pos_set,
				(int*)&outputMsg.overturn_angle_front,
				(int*)&outputMsg.overturn_angle_back,
				(int*)&outputMsg.overturn_angle_left,
				(int*)&outputMsg.overturn_angle_right,
				(int*)&outputMsg.ot_threshold_stage1,
				(int*)&outputMsg.ot_threshold_stage2,
				(unsigned int*)&outputMsg.bitfield);

		if (convertions != 12)
		{
			std::string errorMsg = "String Message Format do not match with UIOutputMsg Format. "
					"Please check the regular expression and message format.";
			DEBUG("%s", errorMsg.c_str());
			throw ParseException(errorMsg);
		}
	}
	else
	{
		unsigned int rc, pc; 	// ignore
		
		int convertions = sscanf(ui_msg.data(),
				"1 SQ:%u XV:%d YV:%d GS:%u MP:%u OF:%d OB:%d OL:%d OR:%d S1:%d S2:%d RC:%d PC:%d BF:%u",
				(unsigned int*)&outputMsg.sequence_number,
				(unsigned int*)&outputMsg.vector_x,
				(unsigned int*)&outputMsg.vector_y,
				(unsigned int*)&outputMsg.gripper_pos_set,
				(unsigned int*)&outputMsg.manipulator_pos_set,
				(int*)&outputMsg.overturn_angle_front,
				(int*)&outputMsg.overturn_angle_back,
				(int*)&outputMsg.overturn_angle_left,
				(int*)&outputMsg.overturn_angle_right,
				(int*)&outputMsg.ot_threshold_stage1,
				(int*)&outputMsg.ot_threshold_stage2,
				&rc,
				&pc,
				(unsigned int*)&outputMsg.bitfield);

		if (convertions != 14)
		{
			std::string errorMsg = "String Message Format do not match with UIOutputMsg Format. "
					"Please check the regular expression and message format.";
			DEBUG("%s", errorMsg.c_str());
			throw ParseException(errorMsg);
		}
	}
	return outputMsg;
}

/*
 * LSMStatusMsgConverter.h
 *
 *  Created on: 10.12.2014
 *      Author: mschenk
 */

#ifndef LSMSTATUSMSGCONVERTER_H_
#define LSMSTATUSMSGCONVERTER_H_

#include <ctype.h>

#define LSM_STATUS_FRAME_ID 1
#define LSM_STATUS_FRAME_LENGTH 11


typedef struct LSMStatusFrame_
{
	uint32_t timestamp;
	double current_angle;
	double current_speed;
	unsigned char bitfield;
} LSMStatusFrame;

class LSMStatusMsgConverter
{
	public:
		static LSMStatusFrame ConvertToLSMStatusMsg(unsigned char* lsm_status_msg, unsigned char* seq_dest);
        static void ConvertToBytes(LSMStatusFrame lsm_status_msg, unsigned char* dest, unsigned char seq);
};

#endif

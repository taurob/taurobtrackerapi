/*
 * LSMStatusMsgConverter.cpp
 *
 *  Created on: 11.12.2014
 *      Author: mschenk
 */

#include <string>
#include <boost/regex.hpp>
#include "taurob_laser_scanner_mount/LSMStatusMsgConverter.h"
#include <stdint.h>
#include <ros/console.h>


void LSMStatusMsgConverter::ConvertToBytes(LSMStatusFrame frame, unsigned char* dest, unsigned char seq)
{
	unsigned int timestamp = frame.timestamp;
	uint16_t current_speed = frame.current_speed;
	uint16_t current_angle = ((frame.current_angle * (180.0/M_PI)) / 0.088); 	// convert to degrees and then to steps
	unsigned char bitfield = frame.bitfield;
	unsigned char frame_id = LSM_STATUS_FRAME_ID; 

	dest[0] = frame_id;
	dest[1] = seq;
	dest[2] = ((timestamp & 0xFF000000) >> 8*3);
	dest[3] = ((timestamp & 0x00FF0000) >> 8*2);
	dest[4] = ((timestamp & 0x0000FF00) >> 8*1);
	dest[5] = ((timestamp & 0x000000FF) >> 8*0);
	dest[6] = ((current_angle & 0xFF00) >> 8*1);
	dest[7] = ((current_angle & 0x00FF) >> 8*0);
	dest[8] = ((current_speed & 0xFF00) >> 8*1);
	dest[9] = ((current_speed & 0x00FF) >> 8*0);
	dest[10] = bitfield;
    }

LSMStatusFrame LSMStatusMsgConverter::ConvertToLSMStatusMsg(unsigned char* lsm_status_msg, unsigned char* seq_dest)
{
	LSMStatusFrame output_frame;
	
	unsigned char frame_id = 0;
	unsigned int timestamp = 0;	// transmitted as uint32, msb first (big-endian)
	unsigned int current_speed = 0; 	// transmitted as uint16, big-endian
	double current_angle = 0;	// transmitted as uint16, big-endian
	unsigned char bitfield = 0; 
	
	// get values from frame
	frame_id = lsm_status_msg[0];
	if (frame_id == LSM_STATUS_FRAME_ID)
	{
		*seq_dest = lsm_status_msg[1];
		timestamp = ((lsm_status_msg[2] << 8*3) | (lsm_status_msg[3] << 8*2) | (lsm_status_msg[4] << 8*1) | (lsm_status_msg[5] << 8*0));
		current_angle = ((lsm_status_msg[6] << 8*1) | (lsm_status_msg[7] << 8*0));
		current_speed = ((lsm_status_msg[8] << 8*1) | (lsm_status_msg[9] << 8*0));
		bitfield = lsm_status_msg[10];
	
		// invert turning direction of current_angle
		current_angle = 4095 - current_angle;
		
		// convert current_angle from 'ticks' (4096) to degrees (360), and then to rad
		current_angle *= 0.088;
		current_angle *= (M_PI / 180.0);
	
		// fill frame
		output_frame.timestamp = timestamp;
		output_frame.current_angle = current_angle;
		output_frame.current_speed = current_speed;
		output_frame.bitfield = bitfield;
	}
	else
	{
		ROS_WARN("received LSM frame with unknown frame id %d -- ignoring this frame.", frame_id);
	}

        return output_frame;
    }


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
 
#include "libtaurob_arm/Arm_frames.h"
#include <stdio.h>
#include <string.h>
#include <byteswap.h>

Arm_frames::Arm_frames() :
    handshake_completed(false),
    last_rx_sequence_number(0),
    sequence_number(0)
{

}

/** Deserializes a byte buffer to a status frame struct, touches *status_frame only
 *  if the frame was received successfully
 * 
 *  @param *byte_buffer
 *  @param byte_buffer_length
 *  @param *status_frame
 *  @return true if a correct status frame has been received, else false
 */
bool Arm_frames::Deserialize_status_frame(unsigned char *byte_buffer,
									 int byte_buffer_length, 
									 MOD_ARM_STATUS_FRAME *status_frame)
{
    MOD_ARM_STATUS_FRAME received_frame;
	uint16_t calculated_crc = 0;
	
    if (byte_buffer_length != MOD_ARM_STATUS_FRAME_LENGTH)
	{
		printf("## Unable to deserialize status frame: Wrong length received (%d)\n", byte_buffer_length);
		return false;
	}

    received_frame.frame_id = byte_buffer[0];
    received_frame.protocol_version = byte_buffer[1];
    received_frame.sequence_number = byte_buffer[2];
    received_frame.bitfield = byte_buffer[3];
    received_frame.timestamp_seconds = ((uint32_t)byte_buffer[4] << 24) +
                                       ((uint32_t)byte_buffer[5] << 16) +
                                       ((uint32_t)byte_buffer[6] << 8) +
                                       ((uint32_t)byte_buffer[7]);
    received_frame.timestamp_microseconds = ((uint32_t)byte_buffer[8] << 24) +
                                            ((uint32_t)byte_buffer[9] << 16) +
                                            ((uint32_t)byte_buffer[10] << 8) +
                                            ((uint32_t)byte_buffer[11]);
    received_frame.position = ((uint16_t)byte_buffer[12] << 8) + byte_buffer[13];
    received_frame.speed = ((uint16_t)byte_buffer[14] << 8) + byte_buffer[15];
    received_frame.effort = ((uint16_t)byte_buffer[16] << 8) + byte_buffer[17];
    received_frame.current = byte_buffer[18];
    received_frame.temperature = byte_buffer[19];
    received_frame.voltage = byte_buffer[20];
    received_frame.crc = ((uint16_t)byte_buffer[21] << 8) + byte_buffer[22];

    if ((received_frame.frame_id != 2) ||
        (received_frame.protocol_version != 1))
		
	{
		printf("## Unable to deserialize status frame: Wrong frame id or protocol version received (%d, %d)\n", 
               received_frame.frame_id,
               received_frame.protocol_version);
		return false;
	}
	/* sequence number has to be greater than last sequence number (taking overflow into account) */
	else if ((handshake_completed) &&
             (((received_frame.sequence_number - last_rx_sequence_number) == 0) ||
              ((received_frame.sequence_number - last_rx_sequence_number) > 128)))
	{
		printf("## Unable to deserialize status frame: Wrong sequence number received (%d, %d)\n", 
               received_frame.sequence_number,
		       last_rx_sequence_number);
		return false;
	}
	else
	{
		handshake_completed = true;
	}

    last_rx_sequence_number = received_frame.sequence_number;

    calculated_crc = Calc_CRC(0xffff, MOD_ARM_STATUS_FRAME_LENGTH - 2, (uint8_t *)byte_buffer);
		
    if (calculated_crc != received_frame.crc)
	{
        printf("## Unable to deserialize status frame: Wrong CRC (%04X, %04X)\n", calculated_crc, received_frame.crc);
		return false;
	}

	/* a valid command frame has been received */
    memcpy (status_frame, (const void *) &received_frame, MOD_ARM_STATUS_FRAME_LENGTH);

	return true;
}

/** Serializes a command frame to a byte buffer
 *  @param *command_frame
 *  @param *byte_buffer (must have at least MOD_ARM_COMMAND_FRAME_LENGTH bytes)
 *  @param byte_buffer_length
 */
void Arm_frames::Serialize_command_frame(MOD_ARM_COMMAND_FRAME *command_frame,
									unsigned char* byte_buffer, 
									int byte_buffer_length)
{
    if (byte_buffer_length >= MOD_ARM_COMMAND_FRAME_LENGTH)
	{
		command_frame->frame_id = 1;
		command_frame->protocol_version = 1;
		command_frame->sequence_number = sequence_number;

        byte_buffer[0] = command_frame->frame_id;
        byte_buffer[1] = command_frame->protocol_version;
        byte_buffer[2] = command_frame->sequence_number;
        byte_buffer[3] = command_frame->state;
        byte_buffer[4] = (uint8_t)(command_frame->set_position >> 8);
        byte_buffer[5] = (uint8_t)(command_frame->set_position & 0xFF);
        byte_buffer[6] = (uint8_t)(command_frame->max_speed >> 8);
        byte_buffer[7] = (uint8_t)(command_frame->max_speed & 0xFF);
        byte_buffer[8] = (uint8_t)(command_frame->set_speed >> 8);
        byte_buffer[9] = (uint8_t)(command_frame->set_speed & 0xFF);
        byte_buffer[10] = command_frame->bitfield;

        command_frame->crc = Calc_CRC(0xffff, MOD_ARM_COMMAND_FRAME_LENGTH - 2, (uint8_t *)byte_buffer);

        byte_buffer[11] = (uint8_t)(command_frame->crc >> 8);
        byte_buffer[12] = (uint8_t)(command_frame->crc & 0xFF);

		sequence_number++;
	}
	else
	{
		printf("## Unable to serialize command frame: wrong byte_buffer_lenght given");
	}
}


uint16_t Arm_frames::Calc_CRC(uint16_t init, int len, unsigned char* src)
{
    uint16_t u2_crc_tbl0[] =
    {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
    };

    for (int i = 0; i < len; i++)
    {
        /* Byte-at-a-time processing. */
        init = (uint16_t)(u2_crc_tbl0[src[i] ^ (init & 0xff)] ^ (init >> 8));
    }

    return init;
}

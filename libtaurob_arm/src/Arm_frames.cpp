/**************************************************************************//**
 *
 * @file Arm_frames.h
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Library for communication with taurob arm, protocol handling
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
 
#include "libtaurob_arm/Arm_frames.h"

float Arm_frames::hex_to_float(char* hexdump)
{
	uint32_t num = 0;
	sscanf(hexdump, "%08X", &num);

	return uint32_to_float(num);
}

float Arm_frames::uint32_to_float(uint32_t num)
{
	// reverse endianness
	num = (((num & 0x000000FF) << 8*3) | ((num & 0x0000FF00) << 8) | ((num & 0x00FF0000) >> 8) | ((num & 0xFF000000) >> 8*3));

	float ret = *((float*)&num);
	return ret;
}

void Arm_frames::float_to_hex(float num, char* dest)
{
	uint8_t* pnum = (uint8_t*)(&num);
	sprintf(dest, "%02X%02X%02X%02X", *(pnum+0), *(pnum+1), *(pnum+2), *(pnum+3));
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

void Arm_frames::Status_to_bytes(Arm_status_frame data, unsigned char* dest)
{
	dest[0] = 1;
	dest[1] = data.sequence_number;
	dest[2] = ((data.position) >> 8);
	dest[3] = ((data.position) & 0xFF);
	dest[4] = data.current;
	dest[5] = data.temperature;
	dest[6] = data.bitfield;
	dest[7] = data.error_code;
	
	uint16_t crc = Calc_CRC(0xFFFF, 8, dest);
	dest[8] = (crc >> 8);
	dest[9] = (crc & 0xFF);
}

void hexdump(void *pAddressIn, long  lSize)
{
 char szBuf[100];
 long lIndent = 1;
 long lOutLen, lIndex, lIndex2, lOutLen2;
 long lRelPos;
 struct { char *pData; unsigned long lSize; } buf;
 unsigned char *pTmp,ucTmp;
 unsigned char *pAddress = (unsigned char *)pAddressIn;

   buf.pData   = (char *)pAddress;
   buf.lSize   = lSize;

   while (buf.lSize > 0)
   {
      pTmp     = (unsigned char *)buf.pData;
      lOutLen  = (int)buf.lSize;
      if (lOutLen > 16)
          lOutLen = 16;

      // create a 64-character formatted output line:
      sprintf(szBuf, " >                            "
                     "                      "
                     "    %08lX", pTmp-pAddress);
      lOutLen2 = lOutLen;

      for(lIndex = 1+lIndent, lIndex2 = 53-15+lIndent, lRelPos = 0;
          lOutLen2;
          lOutLen2--, lIndex += 2, lIndex2++
         )
      {
         ucTmp = *pTmp++;

         sprintf(szBuf + lIndex, "%02X ", (unsigned short)ucTmp);
         if(!isprint(ucTmp))  ucTmp = '.'; // nonprintable char
         szBuf[lIndex2] = ucTmp;

         if (!(++lRelPos & 3))     // extra blank after 4 bytes
         {  lIndex++; szBuf[lIndex+2] = ' '; }
      }

      if (!(lRelPos & 3)) lIndex--;

      szBuf[lIndex  ]   = '<';
      szBuf[lIndex+1]   = ' ';

      printf("%s\n", szBuf);

      buf.pData   += lOutLen;
      buf.lSize   -= lOutLen;
   }
}

bool Arm_frames::Bytes_to_status(unsigned char* data, Arm_status_frame* ret)
{
	bool success = false;
	
	if (data[0] != 0x01)
	{
		printf("## Unable to deserialize status frame byte sequence: No valid signature found! (instead: %d)\n", data[0]);
		hexdump(data, 10);
		//throw ParseException("Unable to deserialize status frame byte sequence: No valid signature found!");
	}
	ret->sequence_number = data[1];
	ret->position = ((int)(data[2]) << 8) | (data[3]);
	ret->current = data[4];
	ret->temperature = data[5];
	ret->bitfield = data[6];
	ret->error_code = data[7];
	
	//printf("Received position %02X %02X, which decodes to %d\n", data[2], data[3], ret->position);
	
	// watch the endianness!
	uint16_t rx_crc = ((int)(data[9]) << 8) | (data[8]);
	uint16_t calc_crc = Calc_CRC(0xFFFF, 8, data);
	
	if (rx_crc != calc_crc)
	{
		printf("## Invalid CRC: %04X instead of %04X, seq. # is %d\n", rx_crc, calc_crc, ret->sequence_number);
		printf("Received:\n");
		hexdump(data, 10);
		printf("----------\n");
	}
	else
	{
		success = true; 	// indicate decode success
	}
	return success;
}

void Arm_frames::Command_to_bytes(Arm_command_frame data, unsigned char* dest)
{
	dest[0] = '~';
	dest[1] = 1;
	dest[2] = data.sequence_number;
	dest[3] = data.bitfield;
	dest[4] = ((data.set_position) >> 8);
	dest[5] = ((data.set_position) & 0xFF);
	dest[6] = ((data.limit_position) >> 8);
	dest[7] = ((data.limit_position) & 0xFF);
	
	uint16_t crc = Calc_CRC(0xffff, 8, dest);
	dest[8] = (crc >> 8);
	dest[9] = (crc & 0xFF);
}

Arm_command_frame Arm_frames::Bytes_to_command(unsigned char* data)
{
	Arm_command_frame ret;
	if (data[0] != '~' || data[1] != 0x01)
	{
		//throw ParseException("Unable to deserialize command frame byte sequence: No valid signature found!");
	}
	ret.sequence_number = data[2];
	ret.bitfield = data[3];
	ret.set_position = ((int)(data[4]) << 8) | (data[5]);
	ret.limit_position = ((int)(data[6]) << 8) | (data[7]);
	
	// check crc
	uint16_t rx_crc = ((int)(data[8]) << 8) | (data[9]);
	uint16_t calc_crc = Calc_CRC(0xFFFF, 8, data);
	
	if (rx_crc != calc_crc)
	{
		//throw ParseException("Error when deserializing command frame byte sequence: Invalid CRC code!");
	}
	
	return ret;
}

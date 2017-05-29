/**************************************************************************//**
 *
 * @file flipper_frames.h
 * @author taurob GmbH
 * @brief Library for communication with taurob flipper, protocol definitions
 *
 *
 *  Copyright (c) 2017 taurob GmbH. All rights reserved.
 *  Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ******************************************************************************/
 
#ifndef FLIPPER_FRAMES_H_
#define FLIPPER_FRAMES_H_

#include <string>
#include <ctype.h>
#include <inttypes.h>

#define FLIPPER_COMMAND_FRAME_LENGTH 8

struct FlipperCommandFrame
{
    uint8_t frame_id;		     //!< 1 for command frame
    uint8_t protocol_version;        //!< protocol version information, currently: 1
    uint8_t sequence_number;         //!< always corresponds to the previously received command frame
    uint8_t flags;                   //!< flags
    int16_t angle;                   //!< absolute angle in 1e-3 rad
    uint16_t crc;                    //!< CRC-16
};

#define FLIPPER_STATUS_FRAME_LENGTH 18

struct FlipperStatusFrame
{
    uint8_t frame_id;                // 2 for status frame
    uint8_t protocol_version;        // protocol version information, currently: 1
    uint8_t sequence_number;         // always corresponds to the previously received command frame
    uint8_t bitfield;                // flags
    uint32_t timestamp_seconds;      // if RTC has been set, seconds since 1.1.1970, else 0
    uint16_t timestamp_microseconds; // if RTC has been set, the remainder of seconds in microseconds, else 0
    int16_t angle;                   // absolute position, 1e-3 rad
    int16_t speed;                   // motor speed in rpm at motor shaft
    uint16_t current;                // motor current in 1e-3 ampere 
    uint16_t crc;                    // CRC-16
};

class FlipperFrames
{
    public:
        FlipperFrames();

        /** Deserializes a byte buffer to a status frame struct, touches *status_frame only
         *  if the frame was received successfully
         *
         *  @param byte_buffer
         *  @param byte_buffer_length
         *  @param status_frame
         *  @return true if a correct status frame has been received, else false
         */
        bool Deserialize_status_frame(const unsigned char *byte_buffer,
                                      int byte_buffer_length,
                                      FlipperStatusFrame *status_frame);

        /** Serializes a command frame to a byte buffer
         *  @param *command_frame
         *  @param *byte_buffer (must have at least MOD_ARM_COMMAND_FRAME_LENGTH bytes)
         *  @param byte_buffer_length
         */
        void Serialize_command_frame(FlipperCommandFrame *command_frame,
                                     unsigned char* byte_buffer,
                                     int byte_buffer_length);

    private:
        static uint16_t Calc_CRC(uint16_t init, int len, unsigned char* src);

        bool handshake_completed;
        uint8_t last_rx_sequence_number;
        uint8_t sequence_number;
};

#endif

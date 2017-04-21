/**************************************************************************//**
 *
 * @file Arm_frames.h
 * @author Lukas Silberbauer, taurob GmbH
 * @date 31 January 2017
 * @brief Library for communication with taurob arm, protocol definitions
 *
 *
 *  Copyright (c) 2017 taurob GmbH. All rights reserved.
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
 
#ifndef ARMFRAMES_H_
#define ARMFRAMES_H_

#include <string>
#include <ctype.h>
#include <inttypes.h>

#define MOD_ARM_CF_STATE_STOP                  0          //!< the arm joint is stopped
#define MOD_ARM_CF_POSITION_CONTROL            1          //!< the arm joint drives to a set position
#define MOD_ARM_CF_SPEED_CONTROL               2          //!< the arm joint rotates with a set speed

/* MOD_ARM_COMMAND_FRAME->bitfield */
#define MOD_ARM_CF_RESET_FRICTION_CLUTCH        0x01      //!< acknowledges and resets the friction_clutch_slipped field if true
#define MOD_ARM_CF_CONTROL_AUX_PORT             0x02      //!< if true the aux_values are used - can only be used by one task per U-ECU!
#define MOD_ARM_CF_AUX_FIVE_V_POWER_ENABLE      0x04      //!< switches the 5V power supply output on if true
#define MOD_ARM_CF_AUX_TWELVE_V_POWER_ENABLE    0x08      //!< switches the 12V power supply output on if true
#define MOD_ARM_CF_AUX_DIG_OUT_1_ENABLE         0x10      //!< switches the digital 1 output on/off
#define MOD_ARM_CF_AUX_DIG_OUT_2_ENABLE         0x20      //!< switches the digital 2 output on/off
#define MOD_ARM_CF_TRIGGER_U_ECU_RESET          0x40      //!< allows to trigger a U-ECU reset - only possible once each minute

/* MOD_ARM_STATUS_FRAME->bitfield */
#define MOD_ARM_SF_POSITION_NOT_VALID           0x01      //!< true if the position reported is not valid (i.e. encoder error)
#define MOD_ARM_SF_FRICTION_CLUTCH_SLIPPED      0x02      //!< true if the friction clutch has slipped
#define MOD_ARM_SF_DIG_IN_1_STATUS              0x04      //!< logic level of digital input 1
#define MOD_ARM_SF_DIG_IN_2_STATUS              0x08      //!< logic level of digital input 2

#define MOD_ARM_COMMAND_FRAME_DEFAULT_VALUES \
{                                            \
    .frame_id = 1,							 \
    .protocol_version = 1,					 \
    .sequence_number = 0,					 \
    .state = MOD_ARM_CF_STATE_STOP,          \
    .set_position = 0,     			         \
    .max_speed = 0,						     \
    .set_speed = 0,                          \
    .bitfield = 0,							 \
    .crc = 0								 \
}

#define MOD_ARM_STATUS_FRAME_DEFAULT_VALUES  \
{											 \
    .frame_id = 2,							 \
    .protocol_version = 1,					 \
    .sequence_number = 0,  					 \
    .bitfield = 0,   						 \
    .timestamp_seconds = 0,					 \
    .timestamp_microseconds = 0,			 \
    .position = -1,							 \
    .speed = 0,          					 \
    .current = 0,             				 \
    .temperature = 0,           			 \
    .voltage = 0,       					 \
    .crc = 0          						 \
}

#define MOD_ARM_COMMAND_FRAME_LENGTH	13

typedef struct _MOD_ARM_COMMAND_FRAME
{
    uint8_t frame_id;			     //!< 1 for command frame
    uint8_t protocol_version;        //!< protocol version information, currently: 1
    uint8_t sequence_number;	     //!< always corresponds to the previously received command frame
    uint8_t state;                   //!< stop / position control / speed control
    int16_t set_position;            //!< position control: absolute position in 1e-3 rad
    int16_t max_speed;               //!< position control: maximum joint speed in 1e-3 rad/sec to reach position or 0 to use default setting
    int16_t set_speed;               //!< speed control: signed joint speed in 1e-3 rad/sec
    uint8_t bitfield;                //!< see corresponding defines
    uint16_t crc;                    //!< CRC-16 as implemented by utils.h
} MOD_ARM_COMMAND_FRAME;

#define MOD_ARM_STATUS_FRAME_LENGTH		23

typedef struct _MOD_ARM_STATUS_FRAME
{
    uint8_t frame_id;                // 2 for status frame
    uint8_t protocol_version;        // protocol version information, currently: 1
    uint8_t sequence_number;         // always corresponds to the previously received command frame
    uint8_t bitfield;                // see corresponding defines
    uint32_t timestamp_seconds;      // if RTC has been set, seconds since 1.1.1970, else 0
    uint32_t timestamp_microseconds; // if RTC has been set, the remainder of seconds in microseconds, else 0
    int16_t position;                // absolute position, 1e-3 rad
    int16_t speed;                   // signed joint speed in 1e-3 rad/sec
    int16_t effort;                  // signed effort in Ndm (Nm * 10)
    uint8_t current;                 // motor current in ampere * 10
    uint8_t temperature;             // temperature in °C + 40, i.e. 0 = -40 °C, 40 = 0 °C, 255 = 215 °C
    uint8_t voltage;                 // voltage in Volt - 10 * 10, i.e. 0 = 10.0V, 100 = 20.0V, 255 = 35.5V
    uint16_t crc;                    // CRC-16 as implemented by utils.h
} MOD_ARM_STATUS_FRAME;

class Arm_frames
{
    public:
        Arm_frames();

        /** Deserializes a byte buffer to a status frame struct, touches *status_frame only
         *  if the frame was received successfully
         *
         *  @param *byte_buffer
         *  @param byte_buffer_length
         *  @param *status_frame
         *  @return true if a correct status frame has been received, else false
         */
        bool Deserialize_status_frame(unsigned char *byte_buffer,
                                             int byte_buffer_length,
                                             MOD_ARM_STATUS_FRAME *status_frame);

        /** Serializes a command frame to a byte buffer
         *  @param *command_frame
         *  @param *byte_buffer (must have at least MOD_ARM_COMMAND_FRAME_LENGTH bytes)
         *  @param byte_buffer_length
         */
        void Serialize_command_frame(MOD_ARM_COMMAND_FRAME *command_frame,
                                            unsigned char* byte_buffer,
                                            int byte_buffer_length);

    private:

        bool handshake_completed;
        uint8_t last_rx_sequence_number;
        uint8_t sequence_number;

        uint16_t Calc_CRC(uint16_t init, int len, unsigned char* src);
};

#endif

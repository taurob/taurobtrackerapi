/**************************************************************************//**
 *
 * @file Arm_segment.cpp
 * @author Lukas Silberbauer, taurob GmbH
 * @date 2 Feb 2017
 * @brief Library for communication with taurob arm, class representing one arm segment
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

#include "libtaurob_arm/libtaurob_arm.h"

#include <string>
#include <stdlib.h>

using namespace boost;

Arm_segment::Arm_segment(std::string segment_name,
						 std::string host_ip, 
						 int host_port, 
                         int segment_nr,
                         int position_tolerance_mrad,
                         IReceiveCallback *status_update_callback) :
    segment_name(segment_name),
	host_ip(host_ip), 
	host_port(host_port),	
    segment_nr(segment_nr),
    reset_friction_clutch_counter(0),
    latest_rx_frame_time(boost::posix_time::time_from_string("1970-01-01 00:00:00.000")),
    sending_thread_running(false),
    position_tolerance_mrad(position_tolerance_mrad),
    parent_callback(status_update_callback),
    position_reached(false),
    last_converted_position_value(0)
{ 
    arm_frames = new Arm_frames();
}

Arm_segment::~Arm_segment() 
{
    Stop();
}

void Arm_segment::On_data_received(unsigned char* data, int len, char* from_remote_ip, int from_remote_port, int from_local_port)
{
    MOD_ARM_STATUS_FRAME status_frame;

    if (arm_frames->Deserialize_status_frame(data, len, &status_frame))
    {
        /* a valid frame was received */
        latest_rx_frame_time = boost::posix_time::microsec_clock::local_time();

        current_status_locker.lock();

        if ((status_frame.bitfield & MOD_ARM_SF_FRICTION_CLUTCH_SLIPPED) != 0)
        {
            current_status.status_bitfield |= SEGMENT_STATUS_FRICTION_CLUTCH_SLIPPED;
        }
        else
        {
            current_status.status_bitfield &= ~SEGMENT_STATUS_FRICTION_CLUTCH_SLIPPED;
        }

        if ((status_frame.bitfield & MOD_ARM_SF_POSITION_NOT_VALID) != 0)
        {
            current_status.status_bitfield |= SEGMENT_STATUS_ANGLE_NOT_VALID;

            /* the position is currently not known */
            current_status.position = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            current_status.status_bitfield &= ~SEGMENT_STATUS_ANGLE_NOT_VALID;

            current_status.position = ((float)status_frame.position) / 1000; // convert to radian
        }

        current_status.dig_in_1_state = ((status_frame.bitfield & MOD_ARM_SF_DIG_IN_1_STATUS) != 0);
        current_status.dig_in_2_state = ((status_frame.bitfield & MOD_ARM_SF_DIG_IN_2_STATUS) != 0);

        current_status.timestamp_sec = status_frame.timestamp_seconds;
        current_status.timestamp_ns = status_frame.timestamp_microseconds * 1000;

        current_status.speed = ((float)status_frame.speed) / 1000; // convert to radian/sec
        current_status.effort = ((float)status_frame.effort) / 10; // convert to Nm
        current_status.current = (float)status_frame.current / 10; // convert to ampere
        current_status.temperature = (float)status_frame.temperature - 40; // convert to celsius
        current_status.voltage = ((float)status_frame.voltage / 10) + 10; // convert to volt

        if (parent_callback != NULL)
        {
            parent_callback->On_status_update_received(segment_nr, current_status);
        }

        current_status_locker.unlock();
    }
    else
    {
        printf("[Arm seg. #%d (%d)] Status frame has invalid format -- doing nothing.\n", segment_nr, host_port);
    }
}

void Arm_segment::Run()
{
    Init_UDP_frames();

    ecu_socket = new SocketUDPBinary(host_ip, host_port, host_port, MOD_ARM_STATUS_FRAME_LENGTH); // bind to same port as host_port, since ecu will send responses there
    ecu_socket->start_listen(this);

    sending_thread_running = true;
    sending_thread = boost::thread(boost::bind(&Arm_segment::Sending_thread, this));
}

void Arm_segment::Stop()
{
    sending_thread_running = false;

    sending_thread.join();

    ecu_socket->stop_listen();
    delete ecu_socket;
    ecu_socket = 0;
}

void Arm_segment::Set(ARM_SEGMENT_COMMAND command)
{
    int16_t converted_position_value = 0;
    int16_t converted_max_speed_value = 0;
    int16_t converted_speed_value = 0;

    /* perform NaN check first */
    if ((command.set_position == command.set_position) &&
        (command.max_speed == command.max_speed) &&
        (command.set_speed == command.set_speed))
    {
        converted_position_value = (int16_t)(command.set_position * 1000);
        converted_max_speed_value = (int16_t)(command.max_speed * 1000);
        converted_speed_value = (int16_t)(command.set_speed * 1000);
    }
    else
    {
        printf("[Arm seg. #%d (%d)] Received NaN in set_value!", segment_nr, host_port);
        command.state = STOP;
    }

    current_set_values_locker.lock();

    switch (command.state)
    {
        case STOP:
            current_set_values.state = MOD_ARM_CF_STATE_STOP;
            current_set_values.set_position = 0;
            current_set_values.max_speed = 0;
            current_set_values.set_speed = 0;
            break;
        case POSITION_CONTROL:
            current_set_values.state = MOD_ARM_CF_POSITION_CONTROL;
            current_set_values.set_position = converted_position_value;
            current_set_values.max_speed = converted_max_speed_value;
            current_set_values.set_speed = 0;

            if (converted_position_value != last_converted_position_value)
            {
                position_reached = false;
            }

            last_converted_position_value = converted_position_value;
            break;
        case SPEED_CONTROL:
            current_set_values.state = MOD_ARM_CF_SPEED_CONTROL;
            current_set_values.set_position = 0;
            current_set_values.max_speed = 0;
            current_set_values.set_speed = converted_speed_value;
            position_reached = false;
    }

    if (command.reset_friction_clutch)
    {
        current_set_values.bitfield |= MOD_ARM_CF_RESET_FRICTION_CLUTCH;
        reset_friction_clutch_counter = RESET_FRICTION_CLUTCH_FRAMES;
    }

    if (command.control_aux_port)
    {
        current_set_values.bitfield &= ~(MOD_ARM_CF_AUX_FIVE_V_POWER_ENABLE   |
                                         MOD_ARM_CF_AUX_TWELVE_V_POWER_ENABLE |
                                         MOD_ARM_CF_AUX_DIG_OUT_1_ENABLE      |
                                         MOD_ARM_CF_AUX_DIG_OUT_2_ENABLE);

        current_set_values.bitfield |= MOD_ARM_CF_CONTROL_AUX_PORT;

        if (command.enable_five_volt)
        {
            current_set_values.bitfield |= MOD_ARM_CF_AUX_FIVE_V_POWER_ENABLE;
        }

        if (command.enable_twelve_volt)
        {
            current_set_values.bitfield |= MOD_ARM_CF_AUX_TWELVE_V_POWER_ENABLE;
        }

        if (command.enable_dig_out_1)
        {
            current_set_values.bitfield |= MOD_ARM_CF_AUX_DIG_OUT_1_ENABLE;
        }

        if (command.enable_dig_out_2)
        {
            current_set_values.bitfield |= MOD_ARM_CF_AUX_DIG_OUT_2_ENABLE;
        }
    }

    if (command.reset_u_ecu)
    {
        current_set_values.bitfield |= MOD_ARM_CF_TRIGGER_U_ECU_RESET;
    }
    else
    {
        current_set_values.bitfield &= ~MOD_ARM_CF_TRIGGER_U_ECU_RESET;
    }

    current_set_values_locker.unlock();
}

/** returns the segment's name */
std::string Arm_segment::Get_name()
{
    return segment_name;
}

void Arm_segment::produce_connection_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (ecu_socket->connection_timeout())
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "disconnected");
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
}

void Arm_segment::Init_UDP_frames()
{
    // initialize internal frames to safe values - they are all set to 0 by default
    current_set_values_locker.lock();

    current_set_values.state = MOD_ARM_CF_STATE_STOP;
    current_set_values.set_position = 0;
    current_set_values.max_speed = 0;
    current_set_values.set_speed = 0;
    current_set_values.bitfield = 0;

    reset_friction_clutch_counter = 0;

    current_set_values_locker.unlock();

    current_status_locker.lock();

    current_status.status_bitfield = 0;

    current_status_locker.unlock();
}

void Arm_segment::Sending_thread()
{	
    unsigned char sendbuffer[MOD_ARM_COMMAND_FRAME_LENGTH];
	
    printf("[Arm seg. #%d (%d)] Starting sending thread!\n", segment_nr, host_port);
	
	while (sending_thread_running)
	{
        /* check connection */
        if ((boost::posix_time::microsec_clock::local_time() - latest_rx_frame_time).total_milliseconds() > MAX_CONNECTION_LOSS_TIME_MS)
        {
            current_status_locker.lock();
            current_status.status_bitfield |= SEGMENT_STATUS_CONNECTION_LOST;
            current_status_locker.unlock();

            /* since the arm joint does not respond anymore, we have to notify the parent here */
            if (parent_callback != NULL)
            {
                printf("[Arm seg. #%d (%d)] Lost connection\n", segment_nr, host_port);
                parent_callback->On_status_update_received(segment_nr, current_status);
            }
        }
        else
        {   current_status_locker.lock();
            current_status.status_bitfield &= ~SEGMENT_STATUS_CONNECTION_LOST;
            current_status_locker.unlock();
        }

        current_status_locker.lock();
        current_set_values_locker.lock();

        if ((current_set_values.state == MOD_ARM_CF_POSITION_CONTROL) &&
             (abs(current_set_values.set_position - (current_status.position * 1000)) < position_tolerance_mrad))
        {
            position_reached = true;
        }

        /* if there is an error or the position is reached (to prevent shaking), stop the arm */
        if ((current_status.status_bitfield != SEGMENT_STATUS_NO_ERROR) ||
            (position_reached))
        {
            /* stop the arm */
            current_set_values.state = MOD_ARM_CF_STATE_STOP;
            current_set_values.set_position = 0;
            current_set_values.max_speed = 0;
            current_set_values.set_speed = 0;
        }

        current_status_locker.unlock();

        if (reset_friction_clutch_counter > 0)
        {
            reset_friction_clutch_counter--;
        }
        else
        {
            /*  we have sent 10 frames with MOD_ARM_CF_RESET_FRICTION_CLUTCH bit set */
            current_set_values.bitfield &= ~MOD_ARM_CF_RESET_FRICTION_CLUTCH;
        }

        arm_frames->Serialize_command_frame(&current_set_values, sendbuffer, sizeof(sendbuffer));

		current_set_values_locker.unlock();

        ecu_socket->send(sendbuffer, sizeof(sendbuffer));
		
        boost::this_thread::sleep(boost::posix_time::milliseconds(SENDING_THREAD_PERIOD_MS));
	}
	
    printf("[Arm seg. #%d (%d)] Ending sending thread!\n", segment_nr, host_port);
}

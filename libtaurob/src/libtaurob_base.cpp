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

#include "libtaurob_base/libtaurob_base.h"
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost::tuples;
using namespace boost;


Taurob_base::Taurob_base(std::string host_ip,
                         int host_port,
                         double gear_ratio,
                         double wheel_diameter,
                         double track_width,
                         double turning_geometry_factor) :
	ECU_host_ip(host_ip), 
	ECU_host_port(host_port), 	
    gear_ratio(gear_ratio),
    wheel_diameter(wheel_diameter),
    track_width(track_width),
    turning_geometry_factor(turning_geometry_factor),
	on_receive_callback(0),
    avg_total_motor_current(0),
    cooldown_active(0),
    cooldown_time(boost::posix_time::time_from_string("1970-01-01 00:00:00.000")),
    latest_watchdog_time(boost::posix_time::time_from_string("1970-01-01 00:00:00.000")),
    sending_thread_running(false)
{ 

}

Taurob_base::~Taurob_base() 
{
    Stop();
}

/** implementation for IUdpReceiverBinary */
void Taurob_base::On_string_received(std::string msg_data, char* from_remote_ip, int from_remote_port, int from_local_port)
{
    Base_status_frame received_values;
    DRIVETRAIN_STATUS status;

    if (Base_frames::String_to_status(msg_data, &received_values))
    {
        double distance_travelled_left_wheel = ((float)received_values.distance_travelled_1 * wheel_diameter * M_PI) / (gear_ratio * 24);
        double distance_travelled_right_wheel = ((float)received_values.distance_travelled_2 * wheel_diameter * M_PI) / (gear_ratio * 24);

        status.distance_travelled_forward = (distance_travelled_left_wheel - distance_travelled_right_wheel) / 2;
        status.angle_turned = (distance_travelled_left_wheel + distance_travelled_right_wheel) / (track_width * turning_geometry_factor);

        double speed_left_wheel = ((received_values.BLDC1_speed_get * 2 * M_PI) / (gear_ratio * 60)); // in rad/sec
        double speed_right_wheel = ((received_values.BLDC2_speed_get * 2 * M_PI) / (gear_ratio * 60)); // in rad/sec

        status.driving_speed = ((speed_left_wheel - speed_right_wheel) / 2) * (wheel_diameter / 2); // in m/sec
        status.turning_speed = ((speed_left_wheel + speed_right_wheel) * (wheel_diameter / 2)) / (track_width * turning_geometry_factor); // in rad/sec

        status.left_motor_current = ((float)received_values.BLDC1_current / 2399) * 1000; // in ampere
        status.right_motor_current = ((float)received_values.BLDC2_current / 2399) * 1000; // in ampere

        avg_total_motor_current -= avg_total_motor_current / CURRENT_AVERAGE_ELEMENTS;
        avg_total_motor_current += status.left_motor_current + status.right_motor_current;
        double total_current_average = avg_total_motor_current / CURRENT_AVERAGE_ELEMENTS;
        float temperature = received_values.temp1 - 40.0f;

        if (total_current_average > MAX_TOTAL_MOTOR_CURRENT)
        {
            printf("[Base ECU] OVERCURRENT ON\n");
            cooldown_active = true;
            cooldown_time = boost::posix_time::microsec_clock::local_time();
        }
        else if (temperature > MAX_TEMPERATURE)
        {
            printf("[Base ECU] OVERTEMPERATURE ON\n");
            cooldown_active = true;
            cooldown_time = boost::posix_time::microsec_clock::local_time();
        }

        status.cooldown_active = cooldown_active;
        status.temperature = temperature;
        status.voltage = (received_values.voltage / 10.0) + 10.0;
        status.dig_in_1_state = ((received_values.aux_in & 0x01) != 0);
        status.dig_in_2_state = ((received_values.aux_in & 0x02) != 0);

        /* pressure sensor reading */
        double adc_voltage = ((double)(((uint16_t)received_values.temp3 << 8) + received_values.temp4)) / 1024 * 3.3;

        /* transfer function of MPX4250A Pressure Sensor: VOUT = VS (P × 0.004 - 0.04), VS=5V, /100 for conversion from kPa to Bar */
        double current_air_pressure = (((adc_voltage / 5) + 0.04) / 0.004) / 100;

        avg_air_pressure -= avg_air_pressure / AIR_PRESSURE_ELEMENTS;
        avg_air_pressure += current_air_pressure;
        status.air_pressure = avg_air_pressure / AIR_PRESSURE_ELEMENTS;

        if (on_receive_callback != 0)
        {
            on_receive_callback(status);
        }
    }
    else
    {
        printf("[Base ECU] Status frame has invalid format -- doing nothing.\n");
    }
}

/** Starts communicating with the drivetrain u-ecu via UDP*/
void Taurob_base::Run()
{
    Init_frames();
    sending_thread = boost::thread(boost::bind(&Taurob_base::Sending_thread, this));
    sending_thread_running = true;

    ecu_socket = new SocketUDP(ECU_host_ip, ECU_host_port); // To Send & Receive
    ecu_socket->start_listen(this);
}

/** Stops communicating with the drivetrain u-ecu via UDP*/
void Taurob_base::Stop()
{
    sending_thread_running = false;

    printf("[Base ECU] Stopping - Waiting for threads to join...\n");
    sending_thread.join();
    printf("[Base ECU] ECUClient::run() completed\n");

    ecu_socket->stop_listen();
    delete ecu_socket;
    ecu_socket = null;
}

/** Safety feature: has to be called every 250 ms or the drivetrain will stop */
void Taurob_base::Feed_watchdog()
{
    latest_watchdog_time = boost::posix_time::microsec_clock::local_time();
}

/** Tells the robot where to go
  *
  * @param forward_speed in m/sec
  * @param turning_speed in radian/sec
  */
void Taurob_base::Set_drive_command(float forward_speed, float turning_speed)
{
    float vector_y = (float) ((forward_speed * gear_ratio * 60.0) / (wheel_diameter * M_PI));
    float vector_x = (float) ((-turning_speed * gear_ratio * 60.0 * track_width * turning_geometry_factor)
                            / (wheel_diameter * 2 * M_PI));

    current_set_values_locker.lock();
    current_set_values.vector_x = vector_x;
    current_set_values.vector_y = vector_y;
    current_set_values_locker.unlock();
}

/** Called if a status frame was received */
void Taurob_base::Set_on_received_callback(void (*callback)(DRIVETRAIN_STATUS status))
{
    on_receive_callback = callback;
}

void Taurob_base::Sending_thread()
{
    printf("starting sending thread\n");

    while (sending_thread_running)
    {

        // lock the locker from early on to avoid other threads messing up our safe state (->watchdog)
        current_set_values_locker.lock();

        if ((!Watchdog_ok()) || (cooldown_active))
        {
            // if watchdog timed out or we have an overcurrent error, force-stop the robot
            current_set_values.vector_x = 0;
            current_set_values.vector_y = 0;
        }

        current_set_values.sequence_number = (current_set_values.sequence_number + 1) % 255;

        ecu_socket->send(Base_frames::Command_to_string(current_set_values));

        if (cooldown_active)
        {
            boost::posix_time::time_duration dt = boost::posix_time::microsec_clock::local_time() - cooldown_time;

            if(dt.total_milliseconds() > COOLDOWN_PERIOD_MS)
            {
                printf("[Base ECU] COOLDOWN RESET\n");
                cooldown_active = false;
            }
        }

        current_set_values_locker.unlock();

        boost::this_thread::sleep(boost::posix_time::milliseconds(40));
    }
}

void Taurob_base::Init_frames()
{
    // initialize internal frames to safe values - they are all set to 0 by default
    current_set_values_locker.lock();
    current_set_values.vector_x = 0;
    current_set_values.vector_y = 0;
    current_set_values.ot_threshold_stage1 = 255;
    current_set_values.ot_threshold_stage2 = 255;
    current_set_values.manipulator_pos_set = 0;
    current_set_values.gripper_pos_set = 0;
    current_set_values_locker.unlock();
}

bool Taurob_base::Watchdog_ok()
{
        bool ret = false;

        boost::posix_time::time_duration dt = boost::posix_time::microsec_clock::local_time() - latest_watchdog_time;
        if (dt.total_milliseconds() <= WATCHDOG_MAX_TIME) ret = true;

        return ret;
}

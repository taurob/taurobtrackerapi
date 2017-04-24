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
 
#include "libtaurob_arm/libtaurob_arm.h"


Arm::Arm(Arm_config configuration) :
    config_successful(false),
    latest_watchdog_time(boost::posix_time::time_from_string("1970-01-01 00:00:00.000")),
    watchdog_checker_thread_running(false),
    watchdog_has_expired(false)
{
	int config_size = (int)configuration.joint_ips.size();
	bool consistent = true;
	
	// check if the configuration is consistent
	if (configuration.joint_names.size() != config_size)
	{
		printf("taurob Arm config error: the number of specified joint names (%d) does not match the number of specified IPs (%d)!\n", 
				(int)configuration.joint_names.size(),
				config_size);
		consistent = false;
	}
	
    if (configuration.joint_ports.size() != config_size)
    {
        printf("taurob Arm config error: the number of specified joint ports (%d) does not match the number of specified IPs (%d)!\n",
                (int)configuration.joint_ports.size(),
                config_size);
        consistent = false;
    }

    if (configuration.joint_position_tolerance_mrad.size() != config_size)
    {
        printf("taurob Arm config error: the number of specified joint tolerances (%d) does not match the number of specified IPs (%d)!\n",
                (int)configuration.joint_position_tolerance_mrad.size(),
                config_size);
        consistent = false;
    }

	if (consistent == true)
	{
		config_successful = true;

        for (int i = 0; i < configuration.joint_ips.size(); i++)
		{
            ARM_SEGMENT_STATUS empty_status = { };
            segment_status.push_back(empty_status);

			Arm_segment* seg = new Arm_segment(configuration.joint_names[i], 
											   configuration.joint_ips[i], 
											   configuration.joint_ports[i], 
                                               i,
                                               configuration.joint_position_tolerance_mrad[i],
                                               this);

            segments.push_back(seg);
		}
	}

    watchdog_checker_thread_running = true;
    watchdog_checker_thread = boost::thread(boost::bind(&Arm::Watchdog_checker, this));
}

Arm::~Arm()
{
	for (int i = 0; i < segments.size(); i++)
	{
		delete segments[i];
	}

    watchdog_checker_thread_running = false;
    watchdog_checker_thread.join();
}

/** Starts communicating with all arm joints via UDP */
void Arm::Run()
{
    if (config_successful == true)
    {
        for (int i = 0; i < segments.size(); i++)
        {
            segments[i]->Run();
        }
    }
    else
    {
        printf("Cannot run taurob Arm: invalid configuration!\n");
    }
}

/** Stops communicating with all arm joints via UDP  */
void Arm::Stop()
{
    for (int i = 0; i < segments.size(); i++)
    {
        segments[i]->Stop();
    }
}

/** Safety feature: has to be called every 250 ms or the arm will stop */
void Arm::Feed_watchdog()
{
    latest_watchdog_time = boost::posix_time::microsec_clock::local_time();
    watchdog_has_expired = false;
}

/** Control an arm joint */
void Arm::Set(int segment_number, ARM_SEGMENT_COMMAND command)
{
    if (segment_number < segments.size())
    {
        /* before sending the commmand make sure the arm is not at fault */
        if ((!(Is_ok())) || (watchdog_has_expired))
        {
            /* an error has been detected, make sure the arm stays stopped */
            command.state = STOP;
        }

        segments[segment_number]->Set(command);
    }
    else
    {
        printf("## ERROR: tried to set segment %d, but we only have %d!", segment_number, (int)segments.size());
    }
}

/** Get arm joint data */
ARM_SEGMENT_STATUS Arm::Get(int segment_number)
{
    ARM_SEGMENT_STATUS return_value;

    segment_status_locker.lock();
    return_value = segment_status[segment_number];
    segment_status_locker.unlock();

    return return_value;
}

/** get number of segments */
int Arm::Get_segment_count(void)
{
    return segments.size();
}

/** returns the segment's name */
std::string Arm::Get_segment_name(int segment_nr)
{
    return segments[segment_nr]->Get_name();
}

/** Returns false if an arm_segment has detected an error */
bool Arm::Is_ok(void)
{
    bool status = true;

    segment_status_locker.lock();

    for (int i = 0; i < segments.size(); i++)
    {
        if (segment_status[i].status_bitfield != SEGMENT_STATUS_NO_ERROR)
        {
            status = false;
            break;
        }
    }

    segment_status_locker.unlock();

    return status;
}

/** Set call back for status update events */
void Arm::On_status_update_received(int segment_number,
                                    ARM_SEGMENT_STATUS current_status)
{
    if (current_status.status_bitfield != SEGMENT_STATUS_NO_ERROR)
    {
        /* the current arm joint has stopped, immediately stop the entire arm */
        Halt_entire_arm();
    }

    segment_status_locker.lock();
    segment_status[segment_number] = current_status;
    segment_status_locker.unlock();
}

void Arm::produce_connection_diagnostics(int segment_number, diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (segment_number >= segments.size())
    {
        return;
    }

    segments[segment_number]->produce_connection_diagnostics(stat);
}

void Arm::produce_temperature_diagnostics(int segment_number, diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (segment_number >= segments.size())
    {
        return;
    }

    ARM_SEGMENT_STATUS status = this->Get(segment_number);
    std::stringstream ss;
    ss
        << std::fixed
        << std::setprecision(1)
        << status.temperature
        << " C";

    if (status.temperature > 95)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, ss.str());
    }
    else if (status.temperature > 85)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, ss.str());
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, ss.str());
    }
}

void Arm::produce_clutch_diagnostics(int segment_number, diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (segment_number >= segments.size())
    {
        return;
    }

    ARM_SEGMENT_STATUS status = this->Get(segment_number);
    if (status.status_bitfield & SEGMENT_STATUS_FRICTION_CLUTCH_SLIPPED)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Slipped");
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
}

/** Stops the movement but not the communication to the arm joints */
void Arm::Halt_entire_arm(void)
{
    ARM_SEGMENT_COMMAND stop_command;
    stop_command.state = STOP;
    stop_command.set_position = 0;
    stop_command.max_speed = 0;
    stop_command.set_speed = 0;
    stop_command.reset_friction_clutch = false;
    stop_command.control_aux_port = false;
    stop_command.reset_u_ecu = false;

    for (int i = 0; i < segments.size(); i++)
    {
        segments[i]->Set(stop_command);
    }
}

/** Checks if the watchdog has been fed well */
void Arm::Watchdog_checker(void)
{
    while (watchdog_checker_thread_running)
    {
        if (((boost::posix_time::microsec_clock::local_time() - latest_watchdog_time).total_milliseconds() > WATCHDOG_MAX_TIME_MS) &&
            (!(watchdog_has_expired)))
        {
            Halt_entire_arm();
            watchdog_has_expired = true;
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(WATCHDOG_CHECKER_THREAD_PERIOD_MS));
    }
}

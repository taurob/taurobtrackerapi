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
 
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/param.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "libtaurob_arm/libtaurob_arm.h"
#include "taurob_driver_msgs/JointAdditionalInfo.h"
#include "taurob_driver_msgs/JointAdditionalCommand.h"

using namespace ros;
using namespace std;

static const int JOINT_STATE_PUBLISH_PERIOD_MS = 20;
static const int JOINT_ADDITIONAL_INFO_PUBLISH_PERIOD_MS = 1000;
static const int FRICTION_CLUTCH_TIMEOUT_SEC = 10;

NodeHandle* nh;
Subscriber sub_jointstate, sub_watchdog, sub_additional_commands;
Publisher pub_jointstate, pub_additional_info;

Arm* man = 0;
Arm_config man_config;
std::vector<ARM_SEGMENT_COMMAND> arm_segment_commands;
boost::mutex arm_segement_commands_locker;

double ecu_com_diag_min_freq = 4; 	// 1 msg every 250ms
double ecu_com_diag_max_freq = 40; 	// 1 msg every 25ms
std::vector<diagnostic_updater::HeaderlessTopicDiagnostic*> ecu_coms_diags;
diagnostic_updater::Updater* updater;

boost::thread joint_state_publisher_thread;
bool joint_state_publisher_thread_running = false;

int configured_joint_count = 0;

struct joint_config
{
    std::string name;
    std::string ip;
    int port;
    int position_tolerance_mrad;
};

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) elems.push_back(item);
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

// Parses a line of the config file, returns 'true' if the line was parsed
// correctly, 'false' otherwise.
static bool parse_joint_config(std::string line, joint_config& config)
{
    std::vector<std::string> sp = split(line, ':');
    if (sp.size() != 4 )
    {
        return false;
    }

    config.name = sp[0];
    config.ip = sp[1];
    config.port = atoi(sp[2].c_str());
    config.position_tolerance_mrad = atoi(sp[3].c_str());

    return true;
}

void sigint_handler(int sig)
{
    ros::shutdown();
}


void jointstate_cmd_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if (man != 0)
	{
		std::vector<std::string> jointnames = msg->name;

		for (int index = 0; index < jointnames.size(); index++)
		{
			for (int i = 0; i < man_config.joint_names.size(); i++)
			{
				if (strcmp(jointnames[index].c_str(), man_config.joint_names[i].c_str()) == 0)
				{
                    arm_segement_commands_locker.lock();
                    arm_segment_commands[i].state = POSITION_CONTROL;
                    arm_segment_commands[i].set_position = msg->position[index];
                    arm_segment_commands[i].max_speed = 0;
                    man->Set(i, arm_segment_commands[i]);
                    arm_segement_commands_locker.unlock();
				}
			}
		}
	}
}

void watchdog_feed_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (man != 0 && msg->data == true)
	{
		man->Feed_watchdog();
	}
}

void joint_additional_command_callback(const taurob_driver_msgs::JointAdditionalCommand::ConstPtr& msg)
{
    if (man != 0)
    {
        std::vector<std::string> jointnames = msg->name;

        for (int index = 0; index < jointnames.size(); index++)
        {
            for (int i = 0; i < man_config.joint_names.size(); i++)
            {
                if (strcmp(jointnames[index].c_str(), man_config.joint_names[i].c_str()) == 0)
                {
                    arm_segement_commands_locker.lock();
                    arm_segment_commands[i].control_aux_port = msg->control_aux_port[index];
                    arm_segment_commands[i].enable_five_volt = msg->aux_five_v_power_enable[index];
                    arm_segment_commands[i].enable_twelve_volt = msg->aux_twelve_v_power_enable[index];
                    arm_segment_commands[i].enable_dig_out_1 = msg->aux_dig_out_1_enable[index];
                    arm_segment_commands[i].enable_dig_out_2 = msg->aux_dig_out_2_enable[index];
                    arm_segment_commands[i].reset_u_ecu = msg->trigger_u_ecu_reset[index];
                    man->Set(i, arm_segment_commands[i]);
                    arm_segement_commands_locker.unlock();
                }
            }
        }
    }
}

void init()
{
    std::vector<std::string> man_config_str;

	signal(SIGINT, sigint_handler);
	
	ROS_INFO("reading parameters...");
	
	ros::NodeHandle local_nh = NodeHandle(ros::this_node::getName());

    if (local_nh.getParam("arm_config", man_config_str))  	// yaml config is directly in namespace, not in sub-namespace of this node
    {
        configured_joint_count = man_config_str.size();

        for (int i = 0; i < man_config_str.size(); i++)
        {
            joint_config jc;

            if (!parse_joint_config(man_config_str[i], jc))
            {
                ROS_ERROR("invalid config: \"%s\"!", man_config_str[i].c_str());
                continue;
            }

            updater->setHardwareID("taurob_arm");

            ROS_DEBUG_STREAM("[Arm hw iface] parsed config " << i << ":");
            ROS_DEBUG_STREAM(" name: " << jc.name);
            ROS_DEBUG_STREAM(" IP: " << jc.ip);
            ROS_DEBUG_STREAM(" port: " << jc.port);
            ROS_DEBUG_STREAM(" position tolerance: " << jc.position_tolerance_mrad);

            man_config.Add_segment(jc.name, jc.ip, jc.port, jc.position_tolerance_mrad);

            ARM_SEGMENT_COMMAND stop_command = { };
            arm_segment_commands.push_back(stop_command);

            std::stringstream diag_name;
            diag_name << "Arm ECU " << i << " Communication";

            // prepare diagnostics updater for this joint
            diagnostic_updater::HeaderlessTopicDiagnostic* coms_diag = new diagnostic_updater::HeaderlessTopicDiagnostic(
                diag_name.str(),
                *updater,
                diagnostic_updater::FrequencyStatusParam(&ecu_com_diag_min_freq, &ecu_com_diag_max_freq, 0.1, 10)
            );
            ecu_coms_diags.push_back(coms_diag);

            updater->update();
        }
    }
    else
    {
        ROS_ERROR("did not find config");
    }
	
	sub_jointstate = nh->subscribe("jointstate_cmd", 1, &jointstate_cmd_callback);
	sub_watchdog = nh->subscribe("watchdog_feed", 1, &watchdog_feed_callback);
    sub_additional_commands = nh->subscribe("joint_additional_command", 1, &joint_additional_command_callback);
	pub_jointstate = nh->advertise<sensor_msgs::JointState>("jointstate_status", 1);
    pub_additional_info = nh->advertise<taurob_driver_msgs::JointAdditionalInfo>("joint_additional_info", 1);
	
	ROS_INFO("Initialization complete");
}

void publish_joint_states()
{
    int friction_clutch_reset_counter[man->Get_segment_count()];
    int publish_additional_info_counter[man->Get_segment_count()];
    bool dig_in_1_last_state[man->Get_segment_count()];
    bool dig_in_2_last_state[man->Get_segment_count()];

    while (joint_state_publisher_thread_running)
    {
        for (int i = 0; i < configured_joint_count; i++)
        {
            ARM_SEGMENT_STATUS current_status =  man->Get(i);

            /* publish joint state */
            sensor_msgs::JointState js;
            js.name.push_back(man_config.joint_names[i]);
            ros::Time ros_timestamp(current_status.timestamp_sec, current_status.timestamp_ns);
            js.header.stamp = ros_timestamp;
            js.position.push_back(current_status.position);
            js.velocity.push_back(current_status.speed);
            js.effort.push_back(current_status.effort);

            pub_jointstate.publish(js);

            if (((current_status.status_bitfield & SEGMENT_STATUS_FRICTION_CLUTCH_SLIPPED) != 0) &&
                 (friction_clutch_reset_counter[i] == 0))
            {
                friction_clutch_reset_counter[i] = ((FRICTION_CLUTCH_TIMEOUT_SEC * 1000) /
                                                     JOINT_STATE_PUBLISH_PERIOD_MS) + 1;

                ROS_WARN("Friction clutch of segment %d slipped", i);
            }

            arm_segement_commands_locker.lock();
            if ((friction_clutch_reset_counter[i] > 0) ||
                (arm_segment_commands[i].reset_friction_clutch))
            {
                if (friction_clutch_reset_counter[i] == 0)
                {
                    arm_segment_commands[i].reset_friction_clutch = false;
                }
                else if (friction_clutch_reset_counter[i] == 1)
                {
                    ROS_INFO("Automatically resetting friction clutch of segment %d", i);
                    arm_segment_commands[i].reset_friction_clutch = true;
                    friction_clutch_reset_counter[i] = friction_clutch_reset_counter[i] - 1;
                }
                else
                {
                    friction_clutch_reset_counter[i] = friction_clutch_reset_counter[i] - 1;
                }
            }
            arm_segement_commands_locker.unlock();

            /* publish additional joint information at lower frequency or if a dig_in_state has changed */
            if ((publish_additional_info_counter[i] * JOINT_STATE_PUBLISH_PERIOD_MS >= JOINT_ADDITIONAL_INFO_PUBLISH_PERIOD_MS) ||
                (dig_in_1_last_state[i] != current_status.dig_in_1_state) ||
                (dig_in_2_last_state[i] != current_status.dig_in_2_state))
            {
                taurob_driver_msgs::JointAdditionalInfo additional_info;
                additional_info.name.push_back(man_config.joint_names[i]);
                ros::Time ros_timestamp(current_status.timestamp_sec, current_status.timestamp_ns);
                additional_info.header.stamp = ros_timestamp;
                additional_info.temperature.push_back(current_status.temperature);
                additional_info.voltage.push_back(current_status.voltage);
                additional_info.dig_in_1_state.push_back(current_status.dig_in_1_state);
                additional_info.dig_in_2_state.push_back(current_status.dig_in_2_state);
                pub_additional_info.publish(additional_info);

                publish_additional_info_counter[i] = 0;
                dig_in_1_last_state[i] = current_status.dig_in_1_state;
                dig_in_2_last_state[i] = current_status.dig_in_2_state;
            }
            else
            {
                publish_additional_info_counter[i]++;
            }
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(JOINT_STATE_PUBLISH_PERIOD_MS));
    }
}

int main(int argc, char **argv) 
{
    //getchar();

	ROS_INFO("starting arm node");
	ros::init(argc, argv, "taurob_arm_node");
    diagnostic_updater::Updater base_node_updater;
    updater = &base_node_updater;
	
	NodeHandle rosnh;
	nh = &rosnh;

	init();
	ROS_INFO("\ntaurob Arm Node is running.\n");
	
    man = new Arm(man_config);
	man->Run();

    joint_state_publisher_thread_running = true;
    joint_state_publisher_thread = boost::thread(&publish_joint_states);

	ros::spin();
	
    joint_state_publisher_thread_running = false;
    joint_state_publisher_thread.join();

	man->Stop();
	delete(man);
	
	ROS_INFO("Terminating.");	
	return 0;
}


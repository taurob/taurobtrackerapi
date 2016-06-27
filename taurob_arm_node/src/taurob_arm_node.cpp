/**************************************************************************//**
 *
 * @file taurob_arm_node.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Node to integrate taurob arm functionality with ROS
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
 
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/param.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <boost/tuple/tuple.hpp>
#include <XmlRpcValue.h>

#include <libtaurob_arm/libtaurob_arm.h>

using namespace ros;
using namespace std;

NodeHandle* nh;
Subscriber sub_jointstate, sub_watchdog, sub_enabler;
Publisher pub_jointstate;

Arm* man = 0;
Arm_config man_config;

bool publish_tf;
bool watchdog;
bool control_enabled;

std::vector<float> offsets;

// helper functions
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
					float setpos = (msg->position[index] - offsets[i]);
					
					// make sure setpos is in [0..2pi]
					while (setpos < 0) setpos += 2*M_PI;
					while (setpos > 2*M_PI) setpos -= 2*M_PI;
					ROS_INFO("setting arm %d angle to %f rad (offset is %f)", i, setpos, offsets[i]);
					man->Set_position(i, setpos);
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

void enable_control_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (man != 0)
	{
		ROS_INFO("setting control to %s", (msg->data ? (char*)"true" : (char*)"false"));
		man->Set_pause_sending(!(msg->data)); 	// if control enabled, pause=false. if control disabled, pause=true.
	}
}

void init()
{
	signal(SIGINT, sigint_handler);
	
	ROS_INFO("reading parameters...");
	
	ros::NodeHandle local_nh = NodeHandle(ros::this_node::getName());
	
	// get parameters -- try to, at least
	try
	{
		local_nh.param<bool>("watchdog", watchdog, true);
		ROS_INFO("Safeguard (Watchdog or Command RX Timeout): %s", (char*)(watchdog ? "Watchdog" : "Command RX Timeout"));
		
		local_nh.param<bool>("control_enabled_at_startup", control_enabled, true);
		ROS_INFO("Robot control enabled at startup: %s", (char*)(control_enabled ? "true" : "false"));
		
		std::vector<std::string> man_config_str;
	
		if (local_nh.getParam("arm_config", man_config_str))
		{
			for (int i=0; i < man_config_str.size(); i++) 
			{
				std::vector<std::string> joint_config = split(man_config_str[i], ':');
				if (joint_config.size() != 4)
				{
					ROS_ERROR("invalid config: expected 4 tokens, found %d (\"%s\")!", (int)joint_config.size(), man_config_str[i].c_str());
				}
				else
				{
					ROS_INFO("parsed config:");
					for (int j=0; j < joint_config.size(); j++) 
					{
						if (j == 0) 
						{ 	
							ROS_INFO("Joint %d name: %s", i, joint_config[j].c_str());
							man_config.joint_names.push_back(joint_config[j]);
						} 
						else if (j == 1) 
						{ 	
							ROS_INFO("Joint %d IP: %s", i, joint_config[j].c_str());
							man_config.joint_ips.push_back(joint_config[j]);
						} 
						else if (j == 2) 
						{ 	
							ROS_INFO("Joint %d port: %s", i, joint_config[j].c_str());
							int port = atoi(joint_config[j].c_str());
							man_config.joint_ports.push_back(port);
						} 
						else if (j == 3)
						{
							ROS_INFO("Joint %d offset: %s", i, joint_config[j].c_str());
							float offset = atof(joint_config[j].c_str());
							offsets.push_back(offset);
						}
						
						man_config.joint_channels.push_back(0); 	// channels not used for now
					}
				}
			}
		}
		else
		{
			ROS_ERROR("did not find config");
		}
	}
	catch(...)
	{
		ROS_ERROR("No configuration found -- won't be doing anything.");
	}
	
	sub_jointstate = nh->subscribe("jointstate_cmd", 1, &jointstate_cmd_callback);
	sub_watchdog = nh->subscribe("watchdog_feed", 1, &watchdog_feed_callback);
	sub_enabler = nh->subscribe("enable_control", 1, &enable_control_callback);
	pub_jointstate = nh->advertise<sensor_msgs::JointState>("jointstate_status", 1);
	
	ROS_INFO("Initialization complete");
}

void On_segment_receive(int segment_nr)
{
	ROS_DEBUG("Segment %d received; angle is %f", segment_nr, man->Get_position(segment_nr));
	
	sensor_msgs::JointState js;
	js.name.push_back(man_config.joint_names[segment_nr]);
	js.header.stamp = ros::Time::now();
	
	if (offsets.size() <= segment_nr) ROS_ERROR("Error in On_segment_receive: segment nr is larger than offsets array size!");
	double pos = (man->Get_position(segment_nr) + offsets[segment_nr]);
	while (pos > M_PI) pos -= 2*M_PI;
	while (pos < -M_PI) pos += 2*M_PI;
	js.position.push_back(pos);

	pub_jointstate.publish(js);
}


int main(int argc, char **argv) 
{
	ROS_INFO("starting arm node");
	ros::init(argc, argv, "taurob_arm_node");
	
	NodeHandle rosnh;
	nh = &rosnh;
	
	init();
	ROS_INFO("\ntaurob Arm Node is running.\n");
	
	man = new Arm(man_config, control_enabled);
	man->Set_on_received_callback(&On_segment_receive);
	man->Set_watchdog_enabled(watchdog);
	man->Run();

	ros::spin();
	
	man->Stop();
	delete(man);
	
	ROS_INFO("Terminating.");	
	return 0;
}


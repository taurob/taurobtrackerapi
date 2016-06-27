/**************************************************************************//**
 *
 * @file taurob_claw_node.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Node to integrate taurob gripper functionality with ROS
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
#include <std_msgs/Float64.h>

#include <boost/tuple/tuple.hpp>
#include <XmlRpcValue.h>

#include <libtaurob_claw/libtaurob_claw.h>

using namespace ros;
using namespace std;

NodeHandle* nh;
Subscriber sub_jointstate, sub_watchdog, sub_enabler;
Publisher pub_jointstate, pub_force;

Claw* claw = 0;

bool publish_tf;
bool watchdog;
bool control_enabled;

double rot_offset;
double grip_offset;
std::string ip_address;
int port;


void sigint_handler(int sig)
{
    ros::shutdown();
}

void jointstate_cmd_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if (claw != 0)
	{
		std::vector<std::string> jointnames = msg->name;

		for (int index = 0; index < jointnames.size(); index++)
		{
			if (strcmp(jointnames[index].c_str(), "claw_rotation") == 0)
			{
				float setrot = (msg->position[index] - rot_offset);
				
				// make sure setrot is in [0..2pi]
				while (setrot < 0) setrot += 2*M_PI;
				while (setrot > 2*M_PI) setrot -= 2*M_PI;
				ROS_INFO("setting claw rotation angle to %f rad (offset is %f)", setrot, rot_offset);
				claw->Set_rotation(setrot);
			}
			else if (strcmp(jointnames[index].c_str(), "claw_grip") == 0)
			{
				float setgrip = (msg->position[index] - grip_offset);
				
				// make sure setpos is in [0..2pi]
				while (setgrip < 0) setgrip += 2*M_PI;
				while (setgrip > 2*M_PI) setgrip -= 2*M_PI;
				ROS_INFO("setting claw grip angle to %f rad (offset is %f)", setgrip, grip_offset);
				claw->Set_grip(setgrip);
			}
		}
	}
}


void watchdog_feed_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (claw != 0 && msg->data == true)
	{
		claw->Feed_watchdog();
	}
}

void enable_control_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (claw != 0)
	{
		ROS_INFO("setting control to %s", (msg->data ? (char*)"true" : (char*)"false"));
		claw->Set_pause_sending(!(msg->data)); 	// if control enabled, pause=false. if control disabled, pause=true.
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
		
		local_nh.param<std::string>("ip_address", ip_address, "10.0.0.50");
		ROS_INFO("IP address: %s", ip_address.c_str());
		
		local_nh.param<int>("port", port, 1235);
		ROS_INFO("Port: %d", port);
		
		local_nh.param<double>("rot_offset", rot_offset, 0.0);
		local_nh.param<double>("grip_offset", grip_offset, 0.0);
		ROS_INFO("Offsets: rot: %f, grip: %f", rot_offset, grip_offset);
	}
	catch(...)
	{
		ROS_ERROR("No configuration found -- won't be doing anything.");
	}
	
	sub_jointstate = nh->subscribe("jointstate_cmd", 1, &jointstate_cmd_callback);
	sub_watchdog = nh->subscribe("watchdog_feed", 1, &watchdog_feed_callback);
	sub_enabler = nh->subscribe("enable_control", 1, &enable_control_callback);
	pub_jointstate = nh->advertise<sensor_msgs::JointState>("jointstate_status", 1);
	pub_force = nh->advertise<std_msgs::Float64>("claw_force", 1);
	
	ROS_INFO("Initialization complete");
}

void On_claw_receive()
{
	ROS_DEBUG("Claw received; rot angle is %f, grip angle is %f", claw->Get_rotation(), claw->Get_grip());
	
	sensor_msgs::JointState js;
	js.name.push_back("claw_rotation");
	js.name.push_back("claw_grip");
	js.header.stamp = ros::Time::now();
	
	double rot = (claw->Get_rotation() + rot_offset);
	while (rot > M_PI) rot -= 2*M_PI;
	while (rot < -M_PI) rot += 2*M_PI;
	js.position.push_back(rot);

	double grip = (claw->Get_grip() + grip_offset);
	while (grip > M_PI) grip -= 2*M_PI;
	while (grip < -M_PI) grip += 2*M_PI;
	js.position.push_back(grip);
	
	pub_jointstate.publish(js);

	std_msgs::Float64 forcemsg;
	forcemsg.data = claw->Get_force();
	pub_force.publish(forcemsg);
}


int main(int argc, char **argv) 
{
	ROS_INFO("starting arm node");
	ros::init(argc, argv, "taurob_arm_node");
	
	NodeHandle rosnh;
	nh = &rosnh;
	
	init();
	ROS_INFO("\ntaurob Claw Node is running.\n");
	
	claw = new Claw(ip_address, port, control_enabled);
	claw->Set_on_received_callback(&On_claw_receive);
	claw->Set_watchdog_enabled(watchdog);
	claw->Run();

	ros::spin();
	
	claw->Stop();
	delete(claw);
	
	ROS_INFO("Terminating.");	
	return 0;
}


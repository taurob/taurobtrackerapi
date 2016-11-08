/**************************************************************************//**
 *
 * @file taurob_flipper_node.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Node to integrate taurob flipper functionality with ROS
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

#include <libtaurob_flipper/libtaurob_flipper.h>

using namespace ros;
using namespace std;

NodeHandle* nh;
Subscriber sub_angle, sub_watchdog, sub_enabler;
Publisher pub_jointstate;

Flipper* flipper = 0;

bool publish_tf;
bool watchdog;
bool control_enabled;

double pos_offset;
std::string ip_address;
int port;


void sigint_handler(int sig)
{
    ros::shutdown();
}

void set_angle_callback(const std_msgs::Float64::ConstPtr& msg)
{
	if (flipper != 0)
	{
		flipper->Set_position(0, msg->data);
	}
}

void watchdog_feed_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (flipper != 0 && msg->data == true)
	{
		flipper->Feed_watchdog();
	}
}

void enable_control_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (flipper != 0)
	{
		ROS_INFO("setting control to %s", (msg->data ? (char*)"true" : (char*)"false"));
		flipper->Set_pause_sending(!(msg->data)); 	// if control enabled, pause=false. if control disabled, pause=true.
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
		
		local_nh.param<double>("pos_offset", pos_offset, 0.0);
		ROS_INFO("Offset: %f", pos_offset);
	}
	catch(...)
	{
		ROS_ERROR("No configuration found -- won't be doing anything.");
	}
	
	sub_angle = nh->subscribe("set_flipper_angle", 1, &set_angle_callback);
	sub_watchdog = nh->subscribe("watchdog_feed", 1, &watchdog_feed_callback);
	sub_enabler = nh->subscribe("enable_control", 1, &enable_control_callback);
	pub_jointstate = nh->advertise<sensor_msgs::JointState>("jointstate_status", 1);
	
	ROS_INFO("Initialization complete");
}

void On_flipper_receive(int segment_nr)
{
	ROS_DEBUG("Flipper received; pos is %f", flipper->Get_position(0));
	
	sensor_msgs::JointState js;
	js.name.push_back("flipper_position");
	js.header.stamp = ros::Time::now();
	
	double pos = (flipper->Get_position(0) + pos_offset);
	while (pos > M_PI) pos -= 2*M_PI;
	while (pos < -M_PI) pos += 2*M_PI;
	js.position.push_back(pos);
	
	pub_jointstate.publish(js);
}

int main(int argc, char **argv) 
{
	ROS_INFO("starting flipper node");
	ros::init(argc, argv, "taurob_flipper_node");
	
	NodeHandle rosnh;
	nh = &rosnh;
	
	init();
	ROS_INFO("\ntaurob Flipper Node is running.\n");
	
	Flipper_config config;
	config.joint_names.push_back("flipper_front");
	config.joint_ips.push_back(ip_address);
	config.joint_ports.push_back(port);
	config.joint_channels.push_back(0);
	
	flipper = new Flipper(config, control_enabled);
	flipper->Set_on_received_callback(&On_flipper_receive);
	flipper->Set_watchdog_enabled(watchdog);
	flipper->Run();

	ros::spin();
	
	flipper->Stop();
	delete(flipper);
	
	ROS_INFO("Terminating.");	
	return 0;
}


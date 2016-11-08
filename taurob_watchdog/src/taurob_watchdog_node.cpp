/**************************************************************************//**
 *
 * @file taurob_watchdog_node.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Watchdog server for taurob ROS driver nodes
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

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <libtaurob_tools/SocketUDP.h>
#include <pthread.h> 
#include <unistd.h>
#include "boost/date_time/posix_time/posix_time_types.hpp"

typedef boost::posix_time::ptime tTime;
typedef boost::posix_time::time_duration tTimeDuration;

using namespace std;

static const int WATCHDOG_MAX_TIME = 150;	// ms

int supply_watchdog_after_coms_loss_secs;
int watchdog_detect_after_coms_loss_secs;

ros::Publisher pub_feed;
ros::Publisher pub_coms_lost;

char* watchdog_emitter_ip = 0;
bool coms_loss = false;
bool ping_thread_running = false;
bool watchdog_received_once = false;
bool emergency_stop = false;
bool watchdog_log_warned = false;
bool supply_coms_loss_watchdog = false;

tTime latest_watchdog_time;
tTime coms_loss_time;

void receive_callback(std::string data, char* from_remote_ip, int from_remote_port, int from_local_port)
{
	ROS_DEBUG("received from UDP: %s", data.c_str());
	if (data == "TWF\n" && 
		!emergency_stop)
	{
		ROS_DEBUG("watchdog feed signal received. sending ROS message...");
		watchdog_received_once = true;
		
		if (watchdog_emitter_ip == 0 || strcmp(watchdog_emitter_ip, from_remote_ip) != 0)
		{
			ROS_INFO("Updating watchdog emitter IP to %s", from_remote_ip);
			watchdog_emitter_ip = from_remote_ip;
		}
		std_msgs::Bool bool_msg;
		bool_msg.data = true;
		pub_feed.publish(bool_msg);
		
		latest_watchdog_time = boost::posix_time::microsec_clock::local_time();
	}
}


void* test_connectivity(void* ptr)
{
	char command[127];
	
	while (ros::ok())
	{
		if (watchdog_emitter_ip != 0)
		{
			snprintf(command, 127, "ping -c3 %s > /dev/null", watchdog_emitter_ip);
			ROS_DEBUG("starting connectivity test using `%s`", command);
			int result = system(command);

			if (result == 0)
			{
				if (coms_loss == true)
				{
					ROS_INFO("connectivity test result was positive! signaling that connection is restored.");
					coms_loss = false;
				}
				else ROS_DEBUG("connectivity test result was positive (result: %d).", result);
			}
			else
			{
				if (coms_loss != true)
				{
					coms_loss = true;
					coms_loss_time = boost::posix_time::microsec_clock::local_time();
					ROS_WARN("connectivity test result was negative (result: %d)! indicating COMS loss...", result);
				}
				
			}
		}
		else
		{
			coms_loss = false;
			ROS_DEBUG("connectivity could not be tested as no valid control station IP could be determined!");
		}
		
		std_msgs::Bool msg;
		msg.data = coms_loss;	
		pub_coms_lost.publish(msg);
		
		sleep(5); 	// sleep 5 seconds so we only test connectivity every 5 seconds
	}
	
	ping_thread_running = false;
	pthread_exit(NULL);
	return NULL;
}


bool is_watchdog_ok()
{
	bool ret = false;

	tTimeDuration dt = boost::posix_time::microsec_clock::local_time() - latest_watchdog_time;
	if (dt.total_milliseconds() <= WATCHDOG_MAX_TIME) ret = true;

	return ret;
}

int secs_since_com_loss()
{
	tTimeDuration dt = boost::posix_time::microsec_clock::local_time() - coms_loss_time;
	return dt.total_seconds();
}

void emergency_stop_callback(const std_msgs::BoolConstPtr& msg)
{
	if (msg->data) ROS_WARN("Emergency stop received -- suspending watchdog!");
	else ROS_INFO("Emergency stop released -- resuming watchdog.");
	emergency_stop = msg->data;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "taurob_watchdog");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh = ros::NodeHandle(ros::this_node::getName());
	
	int listen_port = 19090;
	local_nh.param<int>("listen_port", listen_port, 19090);
	local_nh.param<int>("supply_watchdog_after_coms_loss_secs", supply_watchdog_after_coms_loss_secs, 60);
	local_nh.param<int>("watchdog_detect_after_coms_loss_tolerance_secs", watchdog_detect_after_coms_loss_secs, 30);
	
	pub_feed = nh.advertise<std_msgs::Bool>("watchdog_feed", 1);
	pub_coms_lost = nh.advertise<std_msgs::Bool>("/communication_lost", 1);
	ros::Subscriber sub_twist = nh.subscribe("/emergency_stop", 1, &emergency_stop_callback);
	
	SocketUDP udp(listen_port);
	udp.start_listen(receive_callback);
	
	// start thread to see if we've lost connection to control station
	pthread_t ping_thread;
	ping_thread_running = true;
	if (pthread_create(&ping_thread, NULL, test_connectivity, NULL)) ROS_ERROR("Error creating ping thread\n");
	else ROS_INFO("Connectivity test thread started...waiting for watchdog client IP");
	
	ros::Rate loop_rate(10); // 10 Hz == 100ms
	while (ros::ok())
	{
		if (watchdog_received_once && is_watchdog_ok() == false)
        {
			if (watchdog_log_warned == false)
			{
				watchdog_log_warned = true;
				ROS_WARN("Watchdog missed! Ceasing to feed.");
			}
		}
		else
		{
			if (watchdog_log_warned == true)
			{
				watchdog_log_warned = false;
				ROS_INFO("received watchdog again");
			}
		}

		// if communication was lost, send watchdog for supply_watchdog_after_coms_loss_secs seconds to allow for autonomous reaction
		// ..but only if we received a watchdog within watchdog_detect_after_coms_loss_secs seconds prior to COMS loss
		if (coms_loss && 
		    (secs_since_com_loss() < (supply_watchdog_after_coms_loss_secs + watchdog_detect_after_coms_loss_secs)) && 
			!emergency_stop)
		{
			// ..but only if we received a watchdog within 12 seconds prior to COMS loss
			if (supply_coms_loss_watchdog == false)
			{
				// decide if watchdog is gone so long that we don't think it was active before COMS loss
				tTimeDuration dt = boost::posix_time::microsec_clock::local_time() - latest_watchdog_time;

				if (dt.total_milliseconds() <= watchdog_detect_after_coms_loss_secs * 1000) 
				{
					// we do think it was active, so supply system with watchdog for watchdog_detect_after_coms_loss_secs
					supply_coms_loss_watchdog = true;
				}
			}
			
			if (supply_coms_loss_watchdog)
			{
				std_msgs::Bool bool_msg;
				bool_msg.data = true;
				pub_feed.publish(bool_msg);
			}
		}
		if (!coms_loss)
		{
			supply_coms_loss_watchdog = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	udp.stop_listen();
}


#include <ros/ros.h>
#include <libtaurob_arm/libtaurob_arm.h>
#include "taurob_relay_drop/DropRelay.h"

Arm* man;
Arm_config man_config;
int drop_counter = 0;
ros::ServiceServer service;

bool drop(taurob_relay_drop::DropRelay::Request  &req,
          taurob_relay_drop::DropRelay::Response &res)
{
	ROS_INFO("Dropping relay!");
	drop_counter = 5;
	return true;
}

void configure(ros::NodeHandle* nh)
{
	std::string ip_addr;
	int port;
	
	ros::NodeHandle local_nh = ros::NodeHandle(ros::this_node::getName());

	local_nh.param<std::string>("ip_address", ip_addr, "10.0.0.42");
	local_nh.param<int>("port", port, 8042);
	man_config.joint_names.push_back("Relay_drop");
	man_config.joint_ips.push_back(ip_addr);
	man_config.joint_ports.push_back(port);
	man_config.joint_channels.push_back(0); 	// channels not used for now
		
	//sub_watchdog = nh->subscribe("/watchdog_feed", 1, &watchdog_feed_callback);
	service = local_nh.advertiseService("drop_relay", drop);	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "relay_drop");
	ros::NodeHandle nh;
    
	configure(&nh);
	
	man = new Arm(man_config, true);
	man->Set_watchdog_enabled(true);
	man->Set_check_for_static_motor(false);
	man->Run();
	
	ros::Duration wait_interval(0.05); 	// 50ms
	while (ros::ok())
	{
		man->Feed_watchdog();
		ros::spinOnce();
		
		if (drop_counter > 1)
		{
			ROS_INFO("triggering");
			drop_counter--;
			man->Force_motor_enable_once(0);
		}
		else if (drop_counter > 0)
		{
			drop_counter--;
		}
		wait_interval.sleep();
	}
}

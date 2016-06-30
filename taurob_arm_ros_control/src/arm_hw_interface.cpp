/**************************************************************************//**
 *
 * @file arm_hw_interface.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Node to integrate taurob arm functionality with ROS Control
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

#include <taurob_arm_ros_control/arm_hw_interface.h>
#include <pthread.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <libtaurob_tools/Helper_functions.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

//#define VERBOSE_DEBUGGING

controller_manager::ControllerManager* pcm = 0;
Arm* man;
Arm_config man_config;

Arm_hw_interface* ctrl = 0;

bool controller_thread_running = false;
bool watchdog;

ros::Subscriber sub_watchdog, sub_enabler;
std::vector<ros::Publisher> pubs_temp;
ros::Publisher pub_emergency_stop;
std::vector<float> offsets;

uint8_t emergency_stops = 0;
uint8_t last_emergency_stops = 0;

double ecu_com_diag_min_freq = 4; 	// 1 msg every 250ms
double ecu_com_diag_max_freq = 40; 	// 1 msg every 25ms
std::vector<diagnostic_updater::HeaderlessTopicDiagnostic*> ecu_coms_diags;
diagnostic_updater::Updater* updater;

bool ros_control_enabled = false;
bool trajectory_controller_stopped = false;


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
		ROS_DEBUG("setting control to %s", (msg->data ? (char*)"true" : (char*)"false"));
		man->Set_pause_sending(!(msg->data)); 	// if control enabled, pause=false. if control disabled, pause=true.
		ros_control_enabled = msg->data;
	}
}

int configured_joint_count = 0;

void configure(ros::NodeHandle* nh)
{
	std::vector<std::string> man_config_str;
	ros::NodeHandle local_nh = ros::NodeHandle(ros::this_node::getName());

	local_nh.param<bool>("watchdog", watchdog, true);
	ROS_INFO("[Arm hw iface] Safeguard (Watchdog or Command RX Timeout): %s", (char*)(watchdog ? "Watchdog" : "Command RX Timeout"));
	
	local_nh.param<bool>("control_enabled_at_startup", ros_control_enabled, true);
	ROS_INFO("[Arm hw iface] Robot control enabled at startup: %s", (char*)(ros_control_enabled ? "true" : "false"));
	
	if (nh->getParam("/taurob_tracker/arm_config", man_config_str))  	// yaml config is directly in namespace, not in sub-namespace of this node
	{
		configured_joint_count = man_config_str.size();
		for (int i=0; i < man_config_str.size(); i++) 
		{
			std::vector<std::string> joint_config = split(man_config_str[i], ':');
			if (joint_config.size() != 4)
			{
				ROS_ERROR("invalid config: expected 4 tokens, found %d (\"%s\")!", (int)joint_config.size(), man_config_str[i].c_str());
			}
			else
			{
				updater->setHardwareID("taurob_arm");
				
				ROS_DEBUG("[Arm hw iface] parsed config:");
				for (int j=0; j < joint_config.size(); j++) 
				{
					if (j == 0) 
					{ 	
						ROS_DEBUG("[Arm hw iface] Joint %d name: %s", i, joint_config[j].c_str());
						man_config.joint_names.push_back(joint_config[j]);
					} 
					else if (j == 1) 
					{ 	
						ROS_DEBUG("[Arm hw iface] Joint %d IP: %s", i, joint_config[j].c_str());
						man_config.joint_ips.push_back(joint_config[j]);
					} 
					else if (j == 2) 
					{ 	
						ROS_DEBUG("[Arm hw iface] Joint %d port: %s", i, joint_config[j].c_str());
						int port = atoi(joint_config[j].c_str());
						man_config.joint_ports.push_back(port);
					} 
					else if (j == 3)
					{
						ROS_DEBUG("[Arm hw iface] Joint %d offset: %s", i, joint_config[j].c_str());
						float offset = atof(joint_config[j].c_str());
						offsets.push_back(offset);
					}
					
					man_config.joint_channels.push_back(0); 	// channels not used for now
					
					std::stringstream diag_name;
					diag_name << "Arm ECU " << j << " Communication";
					
					// prepare diagnostics updater for this joint
					diagnostic_updater::HeaderlessTopicDiagnostic* coms_diag = new diagnostic_updater::HeaderlessTopicDiagnostic
											(
												diag_name.str(), 
												*updater, 
												diagnostic_updater::FrequencyStatusParam(&ecu_com_diag_min_freq, 
																						 &ecu_com_diag_max_freq, 
																						 0.1, 
																						 10)
											);
					ecu_coms_diags.push_back(coms_diag);
				}
				
				updater->update();
			}
		}
	}
	else
	{
		ROS_ERROR("did not find config");
	}
		
	sub_watchdog = nh->subscribe("/watchdog_feed", 1, &watchdog_feed_callback);
	sub_enabler = nh->subscribe("/enable_control", 1, &enable_control_callback);
	
	for (int i = 0; i < configured_joint_count; i++)
	{
		std::string topicname = "/arm_" + std::to_string(i) + "_temperature";
		pubs_temp.push_back(nh->advertise<std_msgs::Int8>(topicname, 1));
	}
	pub_emergency_stop = nh->advertise<std_msgs::Bool>("/emergency_stop", 1, true);
}

bool start_all_controllers()
{
	bool ret = true;
	
	// wait for up-to-date values from actor
	ros::Duration wait_for_connection_interval(0.1); 	// 100ms

	while ((man->Is_uptodate() == false) &&
		   ros::ok())
	{
#ifdef VERBOSE_DEBUGGING
		ROS_INFO("[arm hw iface] not uptodate or not ok");
#endif
		ros::spinOnce();
		ctrl->read(); 	// get latest values from actor
		wait_for_connection_interval.sleep();
	}
	
	assert(pcm != 0);
	
	std::vector<std::string> tostart;
	std::vector<std::string> tostop;
	tostart.push_back("arm_traj_controller");
	tostart.push_back("joint_state_controller");
	
	if (pcm->switchController(tostart, tostop, 2) == false)
	{
		ROS_ERROR("[Arm hw iface] Error starting controllers!");
		ret = false;
	}
	else ROS_INFO("[Arm hw iface] Started controllers");

	return ret;
}

bool stop_all_controllers()
{
	bool ret = true;
	assert(pcm != 0);
	
	std::vector<std::string> tostart;
	std::vector<std::string> tostop;
	tostop.push_back("arm_traj_controller");
	tostop.push_back("joint_state_controller");
	
	if (pcm->switchController(tostart, tostop, 2) == false)
	{
		ROS_ERROR("[Arm hw iface] Error stopping controllers!");
		ret = false;
	}
	else ROS_INFO("[Arm hw iface] Stopped controllers");
	return ret;
}

bool start_trajectory_controller()
{
	bool ret = true;
	
	// wait for up-to-date values from actor
	ros::Duration wait_for_connection_interval(0.1); 	// 100ms

	while ((man->Is_uptodate() == false) &&
		   ros::ok())
	{
#ifdef VERBOSE_DEBUGGING
		ROS_INFO("[Arm hw iface] not uptodate or not ok");
#endif
		ros::spinOnce();
		ctrl->read(); 	// get latest values from actor
		wait_for_connection_interval.sleep();
	}
	
	assert(pcm != 0);
	
	std::vector<std::string> tostart;
	std::vector<std::string> tostop;
	tostart.push_back("arm_traj_controller");
	
	if (pcm->switchController(tostart, tostop, 1) == false)
	{
		ROS_ERROR("[Arm hw iface] Error starting trajectory controller!");
		ret = false;
	}
	else ROS_INFO("[Arm hw iface] Started trajectory controller");

	return ret;
}

bool stop_trajectory_controller()
{
	bool ret = true;
	assert(pcm != 0);
	
	std::vector<std::string> tostart;
	std::vector<std::string> tostop;
	tostop.push_back("arm_traj_controller");
	
	if (pcm->switchController(tostart, tostop, 2) == false) 	// 2 is the strictness
	{
		ROS_ERROR("[Arm hw iface] Error stopping controllers!");
		ret = false;
	}
	else ROS_INFO("[Arm hw iface] Stopped trajectory controller");
	return ret;
}

bool unload_controllers()
{
	ROS_INFO("[Arm hw iface] Unloading controllers");
	bool ret = false;
	
	assert(pcm != 0);
	if (pcm->unloadController("arm_traj_controller"))
	{
		ROS_INFO("[Arm hw iface] Successfully unloadeded trajectory controller");
		ret = true;
	}
	else
	{
		ROS_ERROR("Unable to unload trajectory controller!");
	}
	
	if (ret)
	{
		if (pcm->unloadController("joint_state_controller"))
		{
			ROS_INFO("[Arm hw iface] Successfully unloaded joint state controller");
		}
		else
		{
			ROS_ERROR("Unable to unload joint state controller");
			ret = false;
		}
	}
	
	if (ctrl != 0) delete ctrl;
	ctrl = 0;
	
	return ret;
}


bool load_controllers()
{
	ROS_INFO("[Arm hw iface] Loading controllers");
	bool ret = false;
	
	assert(pcm != 0);
	
	if (pcm->loadController("arm_traj_controller"))
	{
		ROS_INFO("[Arm hw iface] Successfully loaded trajectory controller");
		ret = true;
	}
	else
	{
		ROS_ERROR("Unable to load trajectory controller!");
	}
	
	if (ret)
	{
		if (pcm->loadController("joint_state_controller"))
		{
			ROS_INFO("[Arm hw iface] Successfully loaded joint state controller");
		}
		else
		{
			ROS_ERROR("Unable to load joint state controller");
			ret = false;
		}
	}
	
	return ret;
}

void *controller_thread_fn(void *ptr)
{
	assert(pcm != 0);
	ROS_DEBUG("Entering control loop");
	ros::Duration controller_interval(0.02); 	// 20ms				
	while (ros::ok() && controller_thread_running)
	{
		ctrl->read();
		
#ifdef VERBOSE_DEBUGGING
		if (man->Watchdog_ok() == false) ROS_INFO("[Arm hw iface] watchdog nok (1)");
		if (man->Is_uptodate() == false) ROS_INFO("[Arm hw iface] not uptodate (1)");
#endif
		
		pcm->update(ros::Time::now(), controller_interval);			
		
		// only actually send commands if the manipulator is up to date and watchdog is active
		if (man->Watchdog_ok() && man->Is_uptodate())
		{
			ctrl->write();			
		}
		
		if (man->Is_uptodate())
		{
			// publish joint temperatures
			for (int i = 0; i < configured_joint_count; i++)
			{
				std_msgs::Int8 temp;
				temp.data = man->Get_temperature(i);
				pubs_temp[i].publish(temp);
			}
		}
		
		controller_interval.sleep();
	}
	ROS_INFO("[Arm hw iface] Leaving control loop");
}

void On_friction_clutch_slipped_cb(int segment_nr)
{
	ROS_WARN("Friction clutch slipped at segment %d!", segment_nr);
	man->Reset_friction_clutch();
}


void On_received_cb(int segment_nr)
{
	if (ecu_coms_diags.size() > segment_nr && ecu_coms_diags[segment_nr] != 0)
	{
		ecu_coms_diags[segment_nr]->tick();
		updater->update();
	}
	
	unsigned char bitfield = man->Get_bitfield(segment_nr);

	if (((bitfield & 0x01) != 0) && 
		(segment_nr == 4)) 	// emergency stop bit is 0, and it's low-active. only present on segments 3 and 4, but only 4 is working so far (3 is perma-high)
	{
		emergency_stops |= (1 << segment_nr); 	// emergency stop pressed
	}
	else
	{
		emergency_stops &= ~(1 << segment_nr);
	}
	
	if (last_emergency_stops != emergency_stops)
	{
		std_msgs::Bool msg;
		msg.data = (emergency_stops != 0);
		pub_emergency_stop.publish(msg);
		last_emergency_stops = emergency_stops;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm_hw_interface");
	ros::NodeHandle nh;
	diagnostic_updater::Updater base_node_updater;
    updater = &base_node_updater;
    
	configure(&nh);
	
	man = new Arm(man_config, ros_control_enabled);
	man->Set_watchdog_enabled(watchdog);
	man->Set_on_friction_clutch_slipped_callback(&On_friction_clutch_slipped_cb);
	man->Set_on_received_callback(&On_received_cb);
	man->Run();
	
	ctrl = new Arm_hw_interface(man, offsets);		
	controller_manager::ControllerManager cm(ctrl);
	pcm = &cm;
	load_controllers();
	
	pthread_t controller_thread;
	controller_thread_running = true;	
	if (pthread_create(&controller_thread, NULL, controller_thread_fn, NULL)) 
	{
		fprintf(stderr, "Error creating controller thread\n");
		return 1;
	}

	while (ros::ok()) 
	{
		// wait until actors are reachable and deliver valid position
		ROS_INFO("[Arm hw iface] Waiting for actor to deliver current values and watchdog to become active...");
		
		ros::Duration wait_for_connection_interval(0.1); 	// 100ms
		while ((man->Is_uptodate() == false || man->Watchdog_ok() == false) &&
			   ros::ok())
		{
#ifdef VERBOSE_DEBUGGING
			if (man->Watchdog_ok() == false) ROS_INFO("[Arm hw iface] watchdog nok (3)");
			if (man->Is_uptodate() == false) ROS_INFO("[Arm hw iface] not uptodate (3)");
#endif
			ros::spinOnce();
			wait_for_connection_interval.sleep();
		}
		
		ROS_INFO("[Arm hw iface] Got current values");
		
		if (start_all_controllers()) 	// needs a running controller thread (if update isn't called during start_controllers, the whole thing hangs)
		{
			// wait until we lose connection or want to quit - use the time to spin			
			while (ros::ok())
			{
				ros::spinOnce();
				
#ifdef VERBOSE_DEBUGGING
				if (man->Watchdog_ok() == false) ROS_INFO("[Arm hw iface] watchdog nok (4)");
				if (man->Is_uptodate() == false) ROS_INFO("[Arm hw iface] not uptodate (4)");
#endif

				// if we switched to commander or lost connection, stop the trajectory controller
				if ((ros_control_enabled == false || man->Watchdog_ok() == false || man->Is_uptodate() == false) &&
					trajectory_controller_stopped == false)
				{
					stop_trajectory_controller();
					trajectory_controller_stopped = true;
				} 	// ..or re-start it if everything is in order again
				else if (ros_control_enabled && man->Watchdog_ok() && man->Is_uptodate() && 
						 trajectory_controller_stopped == true)
				{
					start_trajectory_controller();
					trajectory_controller_stopped = false;
				}
				
				wait_for_connection_interval.sleep();  	// just use the same interval as before, it's not really critical here
			}
			
			if (man->Is_uptodate() == false) ROS_WARN("[Arm hw iface] Timeout while waiting for new values from actor. Stopping controllers!");
			if (man->Watchdog_ok() == false) ROS_WARN("[Arm hw iface] Watchdog timeout - stopping controllers!");
	
			if (!stop_all_controllers()) break;
		}	
		else break;
	}
	
	ROS_INFO("[Arm hw iface] shutting down");
	controller_thread_running = false;
	
	if (pthread_join(controller_thread, NULL)) 
	{
		fprintf(stderr, "Error joining thread\n");
		return 2;
	}
	
	man->Stop();
	delete man;
	
	return 0;
}

Arm_hw_interface::Arm_hw_interface(Arm* man, std::vector<float> offsets) 
{ 
	manipulator = man;
	this->offsets = offsets;
	
	ROS_INFO("[Arm hw iface] Initializing manipulator with %d joints.\n", man->Get_segment_count());
	
	// initialize vector first, so that we can use addresses of members
	for (int current_segment = 0; current_segment < man->Get_segment_count(); current_segment++)
	{
		cmds.push_back(std::numeric_limits<double>::quiet_NaN());
		vels.push_back(std::numeric_limits<double>::quiet_NaN());
		poss.push_back(std::numeric_limits<double>::quiet_NaN());
		effs.push_back(std::numeric_limits<double>::quiet_NaN());
		
		last_valid_poss.push_back(std::numeric_limits<double>::quiet_NaN());
		received_nan_pos.push_back(false);
	}
	suppress_writing = false;
	
	for (int current_segment = 0; current_segment < man->Get_segment_count(); current_segment++)
	{
		// connect and register the joint state interface
		hardware_interface::JointStateHandle* current_state_handle = 
			new hardware_interface::JointStateHandle(man->Get_segment_name(current_segment), 
													 &poss[current_segment], 
													 &vels[current_segment], 
													 &effs[current_segment]);
		
		joint_state_handles.push_back(current_state_handle);
		jnt_state_interface.registerHandle(*current_state_handle);

		// connect and register the joint position interface
		hardware_interface::JointHandle* current_pos_handle = 
			new hardware_interface::JointHandle(jnt_state_interface.getHandle(man->Get_segment_name(current_segment)), 
													 &cmds[current_segment]);
		joint_handles.push_back(current_pos_handle);
		jnt_pos_interface.registerHandle(*current_pos_handle);		
	}

	registerInterface(&jnt_state_interface);
	registerInterface(&jnt_pos_interface);
}

Arm_hw_interface::~Arm_hw_interface()
{
	for (unsigned int i = 0; i < joint_state_handles.size(); i++) 
	{
		delete joint_state_handles[i];
	}
	
	for (unsigned int i = 0; i < joint_handles.size(); i++)
	{
		delete joint_handles[i];
	}
}

void Arm_hw_interface::read()
{
	for (int i = 0; i < manipulator->Get_segment_count(); i++)
	{
		cmds[i] = std::numeric_limits<double>::quiet_NaN();
		
		double getpos = manipulator->Get_position(i) + offsets[i];
		
		if (getpos != getpos) 	// NaN check -- we never want to publish NaN values
		{
			getpos = last_valid_poss[i]; 	// publish last known good
			suppress_writing = true;
			
			if (received_nan_pos[i] == false)
			{
				ROS_WARN("[Arm hw iface] Received NaN position from joint %d! Setting this jointstate to the last known good value. Arm movements will not work until valid values are received again!", i);
				received_nan_pos[i] = true;
			}
		}
		else 	// valid position
		{
			if (i == 0) getpos = -getpos;

			// map in range [-pi..+pi]
			while (getpos > M_PI) getpos -= 2*M_PI;
			while (getpos < -M_PI) getpos += 2*M_PI;
			
			// if position is valid, store it as the last valid
			last_valid_poss[i] = getpos;
			
			// is this the first valid position after a period of NaNs?
			if (received_nan_pos[i] == true)
			{
				received_nan_pos[i] = false;
				ROS_INFO("[Arm hw iface] Segment %d sent a valid position again (%f). Resuming to publish the current jointstate for this segment.", i, getpos);
				
				// maybe even all joint states are now valid again?
				bool all_jointstates_valid = true;
				for (int j=0; j < received_nan_pos.size(); j++)
				{
					if (received_nan_pos[j] == true) all_jointstates_valid = false;
				}
				if (all_jointstates_valid == true)
				{
					// great, notify user that arm is movable again, and resume to write to actor
					ROS_INFO("[Arm hw iface] Valid positions for all arm segments are available again. Resuming to write to actor - arm movements should work from now on again!");
					suppress_writing = false;
				}
				else
				{
					ROS_INFO("[Arm hw iface] Note that other segments still lack valid positions, so the arm cannot be moved yet!");
				}
			}
		}
		
		poss[i] = getpos;
		effs[i] = manipulator->Get_current(i);
		vels[i] = 0;
		//ROS_DEBUG("current manipulator joint %d position is %f", i, poss[i]);
	}
	//ROS_INFO("[Arm hw iface] ### current arm positions: %f %f %f %f %f", poss[0], poss[1], poss[2], poss[3], poss[4]);
}

void Arm_hw_interface::write()
{
	if (suppress_writing == false)
	{
		for (int i = 0; i < manipulator->Get_segment_count(); i++)
		{
			if (cmds[i] == cmds[i]) 	// tests for NaN
			{
				//ROS_INFO("setting manipulator joint %d to position %f", i, cmds[i]);
				double setpos = cmds[i] - offsets[i];
				if (i == 0) setpos = -cmds[i] - offsets[i];
				
				// make sure value is in [0..2pi]
				while (setpos < 0) setpos += 2*M_PI;
				while (setpos > 2*M_PI) setpos -= 2*M_PI;
				
				manipulator->Set_position(i, setpos);
			}
			else ROS_DEBUG("[Arm hw iface] not setting anything for joint %d - detected NaN", i);
		}
		
		//ROS_INFO("[Arm hw iface] ## writing arm pos: %f %f %f %f %f", cmds[0], cmds[1], cmds[2], cmds[3], cmds[4]);
	}
}

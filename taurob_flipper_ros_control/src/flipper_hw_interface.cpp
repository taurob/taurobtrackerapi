/**************************************************************************//**
 *
 * @file flipper_hw_interface.cpp
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

#include <pthread.h>
#include <ros/ros.h>
#include <taurob_flipper_ros_control/flipper_hw_interface.h>
#include <pthread.h>
#include <std_msgs/Bool.h>
#include <libtaurob_tools/Helper_functions.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

//#define VERBOSE_DEBUGGING
//#define FLIPPER_SIMULATION

controller_manager::ControllerManager* pcm = 0;
Flipper* flipper;

Flipper_hw_interface* ctrl = 0;

std::string ip_address;
int port;
bool controller_thread_running = false;
bool watchdog = false;

ros::Subscriber sub_watchdog, sub_enabler;
double offset = 0;
float temperature = 0;

bool ros_control_enabled = false;
bool trajectory_controller_stopped = false;

double ecu_com_diag_min_freq = 4; 	// 1 msg every 250ms
double ecu_com_diag_max_freq = 40; 	// 1 msg every 25ms
diagnostic_updater::HeaderlessTopicDiagnostic* ecu_coms_diag = 0;
diagnostic_updater::Updater* updater;


void On_received_cb(int segment_nr)
{
	temperature = flipper->Get_temperature();
	if (ecu_coms_diag != 0) ecu_coms_diag->tick();
	updater->update();
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
		ROS_DEBUG("setting control to %s", (msg->data ? (char*)"true" : (char*)"false"));
		flipper->Set_pause_sending(!(msg->data)); 	// if control enabled, pause=false. if control disabled, pause=true.
		ros_control_enabled = msg->data;
	}
}

void check_temperature(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	for (int i = 0; i < 2; i++)
	{
		std::stringstream statname;
		if (temperature > 70)
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Over 70 degrees");
		else if (temperature > 60)
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Over 60 degrees");
		else if (temperature > 50)
			stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Over 50 degrees");
		else
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Below 50 degrees");
		
		statname << "Flipper temperature";
		stat.add(statname.str(), temperature);
	}
}

void configure(ros::NodeHandle* nh)
{
	ros::NodeHandle local_nh = ros::NodeHandle(ros::this_node::getName());
	
	local_nh.param<bool>("watchdog", watchdog, true);
	ROS_INFO("[Flipper hw iface] Safeguard (Watchdog or Command RX Timeout): %s", (char*)(watchdog ? "Watchdog" : "Command RX Timeout"));
	
	local_nh.param<bool>("control_enabled_at_startup", ros_control_enabled, true);
	ROS_INFO("[Flipper hw iface] Robot control enabled at startup: %s", (char*)(ros_control_enabled ? "true" : "false"));
	
	// get parameters -- try to, at least
	try
	{		
		local_nh.param<std::string>("ip_address", ip_address, "10.0.0.50");
		ROS_INFO("IP address: %s", ip_address.c_str());
		
		local_nh.param<int>("port", port, 1235);
		ROS_INFO("Port: %d", port);
		
		local_nh.param<double>("offset", offset, 0.0);
		ROS_INFO("Offset: %f", offset);
	}
	catch(...)
	{
		ROS_ERROR("No configuration found -- won't be doing anything.");
	}
	
	updater->setHardwareID("taurob_flipper");
	std::stringstream diag_name;	

	diag_name << "Flipper ECU temperature";
	diagnostic_updater::FunctionDiagnosticTask* temperature_checker = new diagnostic_updater::FunctionDiagnosticTask
			(
				diag_name.str(), 
				boost::bind(&check_temperature, _1)
			);

	updater->add(*temperature_checker);
	
	diag_name.str(std::string());
	diag_name << "Flipper ECU Communication";

	ecu_coms_diag = new diagnostic_updater::HeaderlessTopicDiagnostic
						(
							diag_name.str(), 
							*updater, 
							diagnostic_updater::FrequencyStatusParam(&ecu_com_diag_min_freq, 
																	 &ecu_com_diag_max_freq, 
																	 0.1, 
																	 10)
						);
	ecu_coms_diag->addTask(temperature_checker);
		
	updater->update();	
	updater->force_update();
		
	sub_watchdog = nh->subscribe("/watchdog_feed", 1, &watchdog_feed_callback);
	sub_enabler = nh->subscribe("/enable_control", 1, &enable_control_callback);
}

bool start_all_controllers()
{
	bool ret = true;
	assert(pcm != 0);
	
	std::vector<std::string> tostart;
	std::vector<std::string> tostop;
	tostart.push_back("flipper_traj_controller");
	tostart.push_back("joint_state_controller");
	
	if (pcm->switchController(tostart, tostop, 2) == false)
	{
		ROS_ERROR("Error starting controllers!");
		ret = false;
	}
	else ROS_INFO("[Flipper hw iface] Started controllers");
	return ret;
}

bool stop_all_controllers()
{
	bool ret = true;
	assert(pcm != 0);
	
	std::vector<std::string> tostart;
	std::vector<std::string> tostop;
	tostop.push_back("flipper_traj_controller");
	tostop.push_back("joint_state_controller");
	
	if (pcm->switchController(tostart, tostop, 2) == false)
	{
		ROS_ERROR("Error stopping controllers!");
		ret = false;
	}
	else ROS_INFO("[Flipper hw iface] Stopped controllers");
	return ret;
}


bool start_trajectory_controller()
{
	bool ret = true;
	
	// wait for up-to-date values from actor
	ros::Duration wait_for_connection_interval(0.1); 	// 100ms

	while ((flipper->Is_uptodate() == false) &&
		   ros::ok())
	{
#ifdef VERBOSE_DEBUGGING
		ROS_INFO("[Flipper hw iface] not uptodate or not ok");
#endif
		ros::spinOnce();
		ctrl->read(); 	// get latest values from actor
		wait_for_connection_interval.sleep();
	}
	
	assert(pcm != 0);
	
	std::vector<std::string> tostart;
	std::vector<std::string> tostop;
	tostart.push_back("flipper_traj_controller");
	
	if (pcm->switchController(tostart, tostop, 1) == false)
	{
		ROS_ERROR("[Flipper hw iface] Error starting trajectory controller!");
		ret = false;
	}
	else ROS_INFO("[Flipper hw iface] Started trajectory controller");

	return ret;
}

bool stop_trajectory_controller()
{
	bool ret = true;
	assert(pcm != 0);
	
	std::vector<std::string> tostart;
	std::vector<std::string> tostop;
	tostop.push_back("flipper_traj_controller");
	
	if (pcm->switchController(tostart, tostop, 2) == false)
	{
		ROS_ERROR("[Flipper hw iface] Error stopping controllers!");
		ret = false;
	}
	else ROS_INFO("[Flipper hw iface] Stopped trajectory controller");
	return ret;
}

bool unload_controllers()
{
	ROS_INFO("[Flipper hw iface] Unloading controllers");
	bool ret = false;
	
	assert(pcm != 0);
	if (pcm->unloadController("flipper_traj_controller"))
	{
		ROS_INFO("[Flipper hw iface] Successfully unloadeded trajectory controller");
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
			ROS_INFO("[Flipper hw iface] Successfully unloaded joint state controller");
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
	ROS_INFO("[Flipper hw iface] Loading controllers");
	bool ret = false;
	
	assert(pcm != 0);
	
	if (pcm->loadController("flipper_traj_controller"))
	{
		ROS_INFO("[Flipper hw iface] Successfully loaded trajectory controller");
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
			ROS_INFO("[Flipper hw iface] Successfully loaded joint state controller");
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
		else
		{
			if (flipper->Watchdog_ok() == false) ROS_INFO("[Flipper hw iface] watchdog nok (1)");
			if (flipper->Is_uptodate() == false) ROS_INFO("[Flipper hw iface] not uptodate (1)");
		}
#endif

		pcm->update(ros::Time::now(), controller_interval);			
		
		// only actually send commands if the manipulator is up to date and watchdog is active
		if (flipper->Watchdog_ok() && flipper->Is_uptodate())
		{
			ctrl->write();			
		}
#ifdef VERBOSE_DEBUGGING
		else
		{
			if (flipper->Watchdog_ok() == false) ROS_INFO("[Flipper hw iface] watchdog nok (2)");
			if (flipper->Is_uptodate() == false) ROS_INFO("[Flipper hw iface] not uptodate (2)");
		}
#endif
		
		controller_interval.sleep();
	}
	ROS_INFO("[Flipper hw iface] Leaving control loop");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "flipper_hw_interface");
	ros::NodeHandle nh;
	
	diagnostic_updater::Updater base_node_updater;
    updater = &base_node_updater;
    
	configure(&nh);

	flipper = new Flipper(ip_address, port, ros_control_enabled);
	flipper->Set_watchdog_enabled(watchdog);
	flipper->Set_on_received_callback(&On_received_cb);
	flipper->Run();
		
	ctrl = new Flipper_hw_interface(flipper, offset);
	controller_manager::ControllerManager cm(ctrl);
	pcm = &cm;
	load_controllers();
	
	pthread_t controller_thread;
	controller_thread_running = true;
	if (pthread_create(&controller_thread, NULL, controller_thread_fn, NULL))
	{
		fprintf(stderr, "Error creating controller thread!\n");
		return 1;
	}
	
	while (ros::ok()) 
	{	
		// wait until actors are reachable and deliver valid position
		ROS_INFO("[Flipper hw iface] Waiting for actor to deliver current values and watchdog to become active...");

		ros::Duration wait_for_connection_interval(0.1); 	// 100ms
#ifndef FLIPPER_SIMULATION	
		while ((flipper->Is_uptodate() == false) && ros::ok())		// wait until actor has connected
		{
#ifdef VERBOSE_DEBUGGING
			if (flipper->Watchdog_ok() == false) ROS_INFO("[Flipper hw iface] watchdog nok (3)");
			if (flipper->Is_uptodate() == false) ROS_INFO("[Flipper hw iface] not uptodate (3)");
#endif
			ros::spinOnce();
			wait_for_connection_interval.sleep();
		}
#endif		
		ROS_INFO("[Flipper hw iface] Got current values");
		
		if (start_all_controllers()) 	// needs a running controller thread (if update isn't called during start_controllers, the whole thing hangs)
		{
			// wait until we lose connection or want to quit - use the time to spin			
			while (ros::ok())
			{
				ros::spinOnce();

#ifdef VERBOSE_DEBUGGING
				if (flipper->Watchdog_ok() == false) ROS_INFO("[Flipper hw iface] watchdog nok (4)");
				if (flipper->Is_uptodate() == false) ROS_INFO("[Flipper hw iface] not uptodate (4)");
#endif

				// if we switched to commander or lost connection, stop the trajectory controller
				if ((ros_control_enabled == false || flipper->Watchdog_ok() == false || flipper->Is_uptodate() == false) &&
					trajectory_controller_stopped == false)
				{
					stop_trajectory_controller();
					trajectory_controller_stopped = true;
				} 	// ..or re-start it if everything is in order again
				else if (ros_control_enabled && flipper->Watchdog_ok() && flipper->Is_uptodate() && 
						 trajectory_controller_stopped == true)
				{
					start_trajectory_controller();
					trajectory_controller_stopped = false;
				}
				
				wait_for_connection_interval.sleep();  	// just use the same interval as before, it's not really critical here
			}
			
			if (flipper->Is_uptodate() == false) ROS_WARN("[Flipper hw iface] Timeout while waiting for new values from actor. Stopping controllers!");
			if (flipper->Watchdog_ok() == false) ROS_WARN("[Flipper hw iface] Watchdog timeout - stopping controllers!");
	
			if (!stop_all_controllers()) break;
		}	
		else break;
	}
	
	ROS_INFO("[Flipper hw iface] shutting down");
	controller_thread_running = false;
	
	if (pthread_join(controller_thread, NULL)) 
	{
		fprintf(stderr, "Error joining thread\n");
		return 2;
	}
		
	flipper->Stop();	
	delete flipper;
	
	return 0;
}

Flipper_hw_interface::Flipper_hw_interface(Flipper* flipper, float offset) 
{ 
	this->flipper = flipper;
	offsets.push_back(offset);
	
	ROS_INFO("[Flipper hw iface] Initializing flipper with 1 joint.\n");
	
	// initialize vector first, so that we can use addresses of members
	cmds.push_back(std::numeric_limits<double>::quiet_NaN());
	vels.push_back(std::numeric_limits<double>::quiet_NaN());
	poss.push_back(std::numeric_limits<double>::quiet_NaN());
	effs.push_back(std::numeric_limits<double>::quiet_NaN());
	
	last_valid_poss.push_back(std::numeric_limits<double>::quiet_NaN());
	received_nan_pos.push_back(false);
	
	suppress_writing = false;
	
	// connect and register the joint state interface
	hardware_interface::JointStateHandle* current_state_handle = 
		new hardware_interface::JointStateHandle(flipper->Get_segment_name(), 
												 &poss[0], 
												 &vels[0], 
												 &effs[0]);
	
	joint_state_handles.push_back(current_state_handle);
	jnt_state_interface.registerHandle(*current_state_handle);

	// connect and register the joint position interface
	hardware_interface::JointHandle* current_pos_handle = 
		new hardware_interface::JointHandle(jnt_state_interface.getHandle(flipper->Get_segment_name()), 
												 &cmds[0]);
	joint_handles.push_back(current_pos_handle);
	jnt_pos_interface.registerHandle(*current_pos_handle);		


	registerInterface(&jnt_state_interface);
	registerInterface(&jnt_pos_interface);
}

Flipper_hw_interface::~Flipper_hw_interface()
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

void Flipper_hw_interface::read()
{
	cmds[0] = std::numeric_limits<double>::quiet_NaN();
	
	// negative sign for manipulator->Get_position(i) to comply with argos model -- -1 means flippers pointing *up*
#ifdef FLIPPER_SIMULATION
#warning Flipper driver node is built to always report a fixed angle!!!
	double getpos = offsets[i];
#else
	double getpos = -flipper->Get_position() + offsets[0];
#endif
	//ROS_INFO("[%d] read position %f, offset %f, this results in %f", i, manipulator->Get_position(i), offsets[i], (float)getpos);
	
	if (getpos != getpos) 	// NaN check -- we never want to publish NaN values
	{
		getpos = last_valid_poss[0]; 	// publish last known good
		suppress_writing = true;
		
		if (received_nan_pos[0] == false)
		{
			ROS_WARN("[Flipper hw iface] Received NaN position from flipper! Setting this jointstate to the last known good value. Flipper movements will not work until valid values are received again!");
			received_nan_pos[0] = true;
		}
	}
	else 	// valid position
	{
		// map in range [-pi..+pi]
		while (getpos > M_PI) getpos -= 2*M_PI;
		while (getpos < -M_PI) getpos += 2*M_PI;
		
		// if position is valid, store it as the last valid
		last_valid_poss[0] = getpos;
		
		// is this the first valid position after a period of NaNs?
		if (received_nan_pos[0] == true)
		{
			received_nan_pos[0] = false;
			ROS_INFO("[Flipper hw iface] Flipper sent a valid position again (%f). Resuming to publish the current jointstate for this segment.", getpos);
			
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
				ROS_INFO("[Flipper hw iface] Note that other segments still lack valid positions, so the arm cannot be moved yet!");
			}
		}
	}
	
	poss[0] = getpos;
	effs[0] = flipper->Get_current();
	vels[0] = 0;
	//ROS_DEBUG("current flipper position is %f", i, poss[i]);
}

void Flipper_hw_interface::write()
{	
	if (suppress_writing == false)
	{
		if (cmds[0] == cmds[0]) 	// NaN check
		{
			// negative sign for cmds[i] to comply with argos model -- -1 means flippers pointing *up*
			// offsets NEEDS to be '+' here otherwise flippers will move as offset is changed
			double setpos = -cmds[0] + offsets[0];
			//ROS_INFO("[%d] setting to position %f with offset %f, which results in %f", i, cmds[i], offsets[i], (float)setpos);
			
			// make sure value is in [0..2pi]
			while (setpos < 0) setpos += 2*M_PI;
			while (setpos > 2*M_PI) setpos -= 2*M_PI;
			
#ifndef FLIPPER_SIMULATION
			flipper->Set_position(setpos);
#else
#warning Flipper driver node is built to never send actual values to actor!!!
#endif
		}
		else ROS_DEBUG("[Flipper hw iface] not setting anything - detected NaN");		
		//ROS_INFO("[Flipper hw iface] ## writing arm pos: %f %f", cmds[0], cmds[1]);
	}
}

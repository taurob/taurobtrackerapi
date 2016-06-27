# taurobtrackerapi

*taurob tracker ROS driver package*

 Author: Martin Schenk, taurob GmbH
 Date: 30 May 2016
 
 This repo contains drivers for the taurob tracker robot system.
 
 Copyright (c) 2016 taurob GmbH. All rights reserved.
 Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 
## Compilation

Simply place the packages in your ROS workspace, and compile using catkin_make.



## Available features

There are five basic components of our ROS package suite:

 * Base platform. Commands to move the robot, i.e., drive its tracks, as well as controlling the flippers, are available. The base platform also offers status information such as odometry, battery voltage, current consumption (motors), on-board temperature, accelerometer and gyroscope data.
 
 * Arm control. Each arm joint may be moved independently, and reports its current angle.
	
 * Claw control. The taurob gripper/claw also has a driver node that allows rotation and gripping, and reports grip force.

 * Command multiplexer. If you want to use ROS in addition to the Commander software, a multiplexer package is available to toggle between Commander and ROS. (Note: Not available with initial release - to be release soon.)

 * Emergency stop. For safety reasons, a special package offers emergency-stop functionality. This must run on a remote computer (i.e., the control station), not the robot itself.

	
WARNING: Using the emergency stop package as detailed in this readme is *mandatory* for operating the robot with ROS! If you need a different implementation of the emergency stop for your project, please contact taurob. 
WARNING: Do not attempt to run multiple instances of any node! The resulting actions are unforeseeable. When using ROS and Commander together, *always* use the provided multiplexer package!


## Package overview

You are now presented with an overview of the directory and package structure of our ROS installation. It is possible to add your packages to this workspace, or merge the taurob packages with your existing workspace. Details about connecting to the on-board PC (if applicable), and a system/IP map should have been delivered together with the robot. For the purposes of this readme, SSH access to the on-board PC is assumed.

The taurob workspace contains the following packages:


### taurob_arm_node - Communicates with the individual arm joints, and offers arm control via JointState messages. Note that you can only use either taurob_arm_node OR taurob_arm_ros_control. Configure which one you want to use in the bringup launchfile.

Publishes:

 * jointstate_status (sensor_msgs/JointState) - The current angle of each arm segment. Only the "position" member is defined! The joint names follow the scheme "arm_joint_n", where n is the joint number. The base joint is number 0, increasing with each subsequent joint.
	
Subscribes:

 * jointstate_cmd (sensor_msgs/JointState) - the desired angle for each arm joint. Only the "position" member is defined, which is the desired position in rad. You do not need to specify all joint positions, it suffices to define those you whish to change. The joint names follow the scheme "arm_joint_n", where n is the joint number. The base joint is number 0, increasing with each subsequent joint.
	
 * enable_control (std_msgs/Bool) - Used for the command multiplexer.
	
 * watchdog_feed (std_msgs/Bool) - Used for the emergency stop. If the node doesn't receive a watchdog message within a defined interval, it will stop any movement. Do not use for other purposes, and do not publish custom messages on this topic.
	

### taurob_arm_ros_control - Communicates with the individual arm joints, and integrates with ROS control. Note that you can only use either taurob_arm_node OR taurob_arm_ros_control. Configure which one you want to use in the bringup launchfile.

	
### taurob_base_node - Handles communication with the robot's base platform.

Publishes:
 * odom (nav_msgs/Odometry) - Odometry of the robot's tracks
	
 * imu_magnetometer (geometry_msgs/Vector3Stamped) - Raw value of the robot's built-in magnetometer. Currently not for public use.
	
 * imu_accelerometer (geometry_msgs/Vector3Stamped) - Raw value of the robot's built-in accelerometer. Currently not for public use.
	
 * imu_gyroscope (geometry_msgs/Vector3Stamped) - Raw value of the robot's built-in gyroscope. Currently not for public use.
	
 * supply_voltage (std_msgs/Float32) - Supply voltage of the robot's battery. Should always be between 23 and 28V. Note that the voltage will drop in high-load scenarios such as stair-climbing. Stop using the robot if the voltage drops below 23V permanently, and load its battery to avoid damage (deep discharge).
	
 * error_code (std_msgs/Byte) - An internal error code. Reads 0 if no error is present. If any other value is published, stop using the robot and contact taurob, describing the circumstances under which the error occured.
	
 * aux_bits (std_msgs/Byte) - Value of the robot's internal digital input pins. The assignment depends on the robot. If relevant to your project or robot, these values will be explained in a separate document.
	
 * remaining_optime (std_msgs/Float32) - The remainin operational time of the robot, calculated based on the supply voltage. This is for informal purposes only. Always use the supply voltage directly to determine if the battery is critically low.
	
Subscribes:
	
 * cmd_vel (geometry_msgs/Twist) - Steer the robot with the specified velocity in x and y directions (forward/backward, left/right). Only the "linear.x" (forward/backward movement) and "angular.z" (rotation around z axis) are defined and interpreted.
	
 * jointstate_cmd (sensor_msgs/JointState) - Control the robot's flippers using a JointState message with joint name "flipper_front". Only the position member is used, where 0 indicates the neutral pose. Note that the "neutral pose" for the flippers is the one they had when the robot was switched on.
	
 * light (std_msgs/Bool) - Sets the state of the robot's headlights.
	
 * bluelight (std_msgs/Bool) - Sets the state of the robot's blue signal lights.
	
 * enable_control (std_msgs/Bool) - Used for the command multiplexer.
		
 * watchdog_feed (std_msgs/Bool) - Used for the emergency stop. If the node doesn't receive a watchdog message within a defined interval, it will stop any movement. Do not use for other purposes, and do not publish custom messages on this topic. 
	

### taurob_claw_node - Offers claw control using a JointState message, and reports current angles and grip force.

Publishes:

 * jointstate_status (sensor_msgs/JointState) - The current angle of the rotation and grip joints. Only the "position" member is defined! Joint names are "claw_rotation" and "claw_grip".
	
 * claw_force (std_msgs/Float64) - The force measured by the sensor in the claw, as a scalar.
	
Subscribes:

 * jointstate_cmd (sensor_msgs/JointState) - the desired angle for rotation and grip joints. Only the "position" member is defined, which is the desired position in rad. You do not need to specify all joint positions, it suffices to define those you whish to change. The joint names are either (or both) "claw_rotation" and/or "claw_grip".
	
 * enable_control (std_msgs/Bool) - Used for the command multiplexer.
	
 * watchdog_feed (std_msgs/Bool) - Used for the emergency stop. If the node doesn't receive a watchdog message within a defined interval, it will stop any movement. Do not use for other purposes, and do not publish custom messages on this topic.


	
### taurob_bringup - Contains launchfile for system bringup.


### taurob_enable_switch - Multiplexer node to toggle between control by ROS and control by Commander. Receives toggle command by Commander via TCP, and publishes ROS message accordingly. (NOTE: not included in initial release. Will be published soon.)

Publishes:
	
 * enable_control (std_msgs/Bool) - True if ROS commands should become active, False if Commander is supposed to control the robot.
	
	

### taurob_watchdog - Watchdog server that listens for messages from an external watchdog client, and relays them to a ROS message.

Publishes:
	
 * watchdog_feed (std_msgs/Bool) - If published, signals that a watchdog pulse was received. As soon as that pulse is absent for more than 150ms, the taurob nodes will stop communicating with the robot, bringing it to a safe halted state.
	


### taurob_watchdog_client - Client program for the watchdog that remotely feeds the watchdog server. Is supposed to run on an external system, it has no dependencies to ROS and may be compiled and run as a stand-alone program outside the ROS workspace.




## Package configuration

The individual packages offer configuration options via ROS parameters. They can be set in the individual launchfiles, or a dedicated config file. The relevant files and options are as follows:


 * taurob_arm_node/config/tracker_arm.yaml: A dedicated configuration file for the arm. The only interesting parameter here is an offset, that can be configured for each arm joint individually (in rad). The file format is documented there.

 * taurob_base_node/launch/tracker_base.launch: The parameters are set in this launchfile. They are:	
	- publish_tf (bool): Whether or not the node should publish a tf from the <tf_from_frame> to the <tf_to_frame>, representing the odometry of the platform.	
	- tf_from_frame (string): Name of the tf source frame (if applicable).	
	- tf_to_frame (string): Name of the tf target frame (if applicable).	
	- base_ip (string): IP address of the robot's base platform ECU.	
	- base_port (int): Port number of the robot's base platform ECU.
	


Other parameters exist, but they are not relevant to the user. Do not change any parameters not mentioned in this manual.



## Starting the nodes

*IMPORTANT*: If your robot has an arm, it MUST be calibrated before using it with ROS! See the previous section "Package configuration". If you are not sure what to do, please contact taurob.


To bring up the driver nodes, use the tracker_bringup.launch file from the taurob_bringup package. The relevant workspace (~/rosws) is set up automatically (via entry in .bashrc). To launch all taurob driver nodes, use the following command:

$ roslaunch taurob_bringup tracker_bringup.launch

*IMPORTANT*: Once the driver nodes are up and running, the battery’s supply voltage will be published on the topic /supply_voltage. *You need to monitor that voltage during operation to prevent potential data loss and damage to the robot’s batteries!*
If the voltage drops below 22V, or remains below 23V for extended periods of time, please shut down everything, turn the robot off and let the battery charge. If you do not monitor the voltage, and it drops further down than that, components such as the WiFi AP or the on-board PC might simply power off, and the battery could be damaged (deep-discharge)! So to prevent loss of data or damage, make sure to always monitor the supply voltage.

It is not recommended to start nodes separately. Make sure that there is always only one instance of any taurob node running! Otherwise, the resulting actions of the robot are unforeseeable.

To make the robot operational, you also need to run a watchdog client on an external PC (i.e. the Control Station). If you use the watchdog client program contained in the taurob ROS package collection, you can launch it on your control station with

$ rosrun taurob_watchdog_client taurob_watchdog_client 10.0.0.3


or, if you use it witout ROS

$ taurob_watchdog_client 10.0.0.3


For more details, please refer to the user manual.



## Quick Reference/Cheatsheet for basic functions

You can use these commands to test the basic functions of the robot. Make sure you started the robot driver nodes beforehand (with the bringup launchfile, see above), and that you have the watchdog client running on an external PC as described above.

### Driving the robot 

CAREFUL: if you issue the next command, the robot *will not stop driving* until you issue another Twist message with zero velocity, or you stop the watchdog/hit the emergency stop!

$ rostopic pub /cmd_vel geometry_msgs/Twist -r 1 -- '[0.3, 0.0, 0.0]' '[0.0, 0.0, -0.1]'

where in this example, 0.3 is the linear speed in x axis (forward, in m/s), and -0.1 is the angular velocity (turning, in rad/s).
It is highly recommended to use the rqt "Robot Steering" plugin instead of terminal commands to test robot driving.


### Controlling the flippers

$ rostopic pub /jointstate_cmd sensor_msgs/JointState '{header: {seq: 0, stamp: {secs: 0, nsecs: 0, frame_id: "", name: ["flipper_front"], position: [0.3], velocity: [0.0], effort: [0.0]' --once

where 0.3 is the desired flipper angle in rad. Note that it might be more convenient to use the rqt Message Publisher to publish this message.


### Lights/Blue lights

$ rostopic pub /light std_msgs/Bool true
$ rostopic pub /bluelight std_msgs/Bool true



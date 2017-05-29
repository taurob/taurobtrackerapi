# taurobtrackerapi

This repository contains drivers for the taurob tracker robot system.

## Prerequisites

* The clocks of the control units inside the robot need to be synchronized via NTP with the computer that is running the ROS stack. Different Linux distributions provide different NTP server packages, below is a working example configuration for the [chrony](https://chrony.tuxfamily.org/) server:
```
server ntp.ubuntu.com iburst

bindaddress 10.0.0.100
allow 10.0.0.0/24

local stratum 10
```
Chrony can then be started with the following command: `chronyd -n -f chrony.conf`.

* The URDF model in this repository depends on the `hector_xacro_tools` package which can be found [here](https://github.com/tu-darmstadt-ros-pkg/hector_models).

## Compilation and setup

Simply place the packages in your ROS workspace, and compile it using `catkin build`.

## Available features

There are three basic components of our ROS package suite:

 * Base platform. Commands to move the robot, i.e., drive its tracks, are available. The base platform also offers status information such as odometry, battery voltage, current consumption (motors), and on-board temperature.
 
 * Arm control. Each arm joint may be moved independently, and reports its current angle.

 * Emergency stop. For safety reasons, a special package offers emergency-stop functionality. This must run on a remote computer (i.e., the control station), not the robot itself.

*WARNING:* Using the emergency stop package as detailed in this readme is *mandatory* for operating the robot with ROS! If you need a different implementation of the emergency stop for your project, please contact taurob. 

*WARNING:* Do not attempt to run multiple instances of any node! The resulting actions are unforeseeable. Also do not try to use ROS and the taurob Commander software together.

## Package overview

This repository contains the following packages:

### taurob_arm_node 

Communicates with the individual arm joints, and offers arm control via sensor_msgs/JointState messages.

*Publishes:*

 * `joint_states` (sensor_msgs/JointState) - The current angle of each arm segment. Only the "position" member is defined! The joint names follow the scheme "arm_joint_n", where n is the joint number. The base joint is number 0, increasing with each subsequent joint.

*Subscribes:*

 * `jointstate_cmd` (sensor_msgs/JointState) - The desired angle for each arm joint. Only the "position" member is defined, which is the desired position in rad. You do not need to specify all joint positions, it suffices to define those you whish to change. The joint names follow the scheme "arm_joint_n", where n is the joint number. The base joint is number 0, increasing with each subsequent joint.

 * `watchdog_feed` (std_msgs/Bool) - Used for the emergency stop. If the node does not receive a watchdog message within a defined interval, it will stop any movement. Do not use for other purposes, and do not publish custom messages on this topic.

### taurob_base_node 

Handles communication with the robot's base platform.

*Publishes:*
 * `odom` (nav_msgs/Odometry) - Odometry of the robot's tracks.

 * `supply_voltage` (std_msgs/Float32) - Supply voltage of the robot's battery. Should always be between 23 and 28V. Note that the voltage will drop in high-load scenarios such as stair-climbing. Stop using the robot if the voltage drops below 23V permanently, and load its battery to avoid damage (deep discharge).

 * `error_code` (std_msgs/Byte) - An internal error code. Reads 0 if no error is present. If any other value is published, stop using the robot and contact taurob, describing the circumstances under which the error occured.

 * `remaining_optime` (std_msgs/Float32) - The remainin operational time of the robot, calculated based on the supply voltage. This is for informal purposes only. Always use the supply voltage directly to determine if the battery is critically low.
	
*Subscribes:*
	
 * `cmd_vel` (geometry_msgs/Twist) - Steer the robot with the specified velocity in x and y directions (forward/backward, left/right). Only the "linear.x" (forward/backward movement) and "angular.z" (rotation around z axis) are defined and interpreted.
	
 * `jointstate_cmd` (sensor_msgs/JointState) - Control the robot's flippers using a JointState message with joint name "flipper_front". Only the position member is used, where 0 indicates the neutral pose. Note that the "neutral pose" for the flippers is the one they had when the robot was switched on.

 * `watchdog_feed` (std_msgs/Bool) - Used for the emergency stop. If the node doesn't receive a watchdog message within a defined interval, it will stop any movement. Do not use for other purposes, and do not publish custom messages on this topic. 

### taurob_flipper_node

Handles communication with the robot's flipper.

*Publishes:*

 * `joint_states` (sensor_msgs/JointState) - The current angle of the flipper. Only the "position" member is defined!

*Subscribes:*

 * `jointstate_cmd` (sensor_msgs/JointState) - The desired angle for the flipper.

 * `watchdog_feed` (std_msgs/Bool) - Used for the emergency stop. If the node does not receive a watchdog message within a defined interval, it will stop any movement. Do not use for other purposes, and do not publish custom messages on this topic.

### taurob_watchdog 

Watchdog server that listens for messages from an external watchdog client, and relays them to a ROS message.

*Publishes:*

 * `watchdog_feed` (std_msgs/Bool) - If published, signals that a watchdog pulse was received. As soon as that pulse is absent for more than 150ms, the taurob nodes will stop communicating with the robot, bringing it to a safe halted state.

### taurob_watchdog_client 

Client program for the watchdog that remotely feeds the watchdog server. Is supposed to run on an external system, it has no dependencies to ROS and may be compiled and run as a stand-alone program outside the ROS workspace.

## Package configuration

The individual packages offer configuration options via ROS parameters. They can be set in the individual launchfiles, or a dedicated config file. The relevant files and options are as follows:

 * taurob_arm_node/config/tracker_arm.yaml: A dedicated configuration file for the arm. The only interesting parameter here is an offset, that can be configured for each arm joint individually (in rad). The file format is documented there.

 * taurob_base_node/launch/tracker_base.launch: The parameters are set in this launchfile. They are:	
   - publish_tf (bool): Whether or not the node should publish a tf from the <tf_from_frame> to the <tf_to_frame>, representing the odometry of the platform.
   - tf_from_frame (string): Name of the tf source frame (if applicable).
   - tf_to_frame (string): Name of the tf target frame (if applicable).
   - base_ip (string): IP address of the robot's base platform ECU.
   - base_port (int): Port number of the robot's base platform ECU.

Other parameters exist, but they are not relevant to the user. Do not change any parameters not mentioned here.

## Starting the nodes

**IMPORTANT:** If your robot has an arm, it MUST be calibrated before using it with ROS! See the previous section "Package configuration". If you are not sure what to do, please contact taurob.

The taurob_tracker_description packages offers example launch files to start all robot components. It merely references the launch files in the individual packages, it is therefore recommended not to edit the launch files. To start a launch file, use the following command:

```
roslaunch taurob_tracker_description <name of the launch file>
```

Three launch files included:
 * tracker_model_rviz.launch: Loads the URDF model of the robot and makes it possible to view it in rviz. (No real robot is needed to start this file.)
 * tracker_passive.launch: Loads the URDF model and starts the ROS drivers, but does not start the watchdog. After this file has been started, the model in rviz should reflect the actual position of the robot, but no movement commands are send to the robot.
 * tracker_steer.launch: Loads the URDF model, starts the ROS drivers, the watchdog, and a ROS node to manipulate the values of the joint angles. *Caution:* The joint angles that are sent to the robot are set to the default value of '0' at start-up, so make sure the robot is in the initial pose before using this launch file, otherwise it will start moving!

Without a running watchdog client, the ROS drivers only read from the control units in the robot, but do not send any commands. To enable sending of commands, the watchdog client has to be started:
```
rosrun taurob_watchdog_client taurob_watchdog_client 127.0.0.1
```

**IMPORTANT:** Once the driver nodes are up and running, the battery’s supply voltage will be published on the topic /supply_voltage. You need to monitor that voltage during operation to prevent potential data loss and damage to the robot’s batteries!
If the voltage drops below 22V, or remains below 23V for extended periods of time, please shut down everything, turn the robot off and let the battery charge. If you do not monitor the voltage, and it drops further down than that, components such as the WiFi AP or the on-board PC might simply power off, and the battery could be damaged (deep-discharge)! So to prevent loss of data or damage, make sure to always monitor the supply voltage.

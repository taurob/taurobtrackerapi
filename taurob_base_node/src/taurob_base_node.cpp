/**************************************************************************//**
 *
 * @file taurob_base_node.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Node to integrate taurob base ECU functionality with ROS Control
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

#include <ros/ros.h>
#include <ros/param.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Byte.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <boost/tuple/tuple.hpp>

#include <libtaurob_base/libtaurob_base.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

using namespace ros;
using namespace std;

#define DEFAULT_GEAR_RATIO 30.0 // to be verified
#define DEFAULT_WHEEL_DIAMETER 0.250 // 0.37 for argos, 0.303 for tracker, 0.250 for mecanum wheels
#define DEFAULT_TRACK_WIDTH 0.55 // for tracker
#define DEFAULT_TURNING_GEOMETRY_FACTOR 1.476 	// by experimentation

#define DEFAULT_ODOM_LOW_COV 1.0
#define DEFAULT_ODOM_HIGH_COV 1000.0
#define DEFAULT_ODOM_TURN_HIGH_COV_TOLERANCE 0.08

double gear_ratio;
double wheel_diameter;
double track_width;
double turning_geometry_factor;


double angular_vel_z_covariance_threshold; // rad/s

double low_speed_linear_vel_variance_x; // m/s
double low_speed_linear_vel_variance_yz;

double high_speed_linear_vel_variance_x;
double high_speed_linear_vel_variance_yz;

double low_speed_angular_vel_variance_yaw; // rad/s
double low_speed_angular_vel_variance_roll_pitch;

double high_speed_angular_vel_variance_yaw;
double high_speed_angular_vel_variance_roll_pitch;


NodeHandle* nh;

std::string ecu_ip;
int ecu_port;

Subscriber sub_twist;
Subscriber sub_jointstate;
Subscriber sub_light;
Subscriber sub_bluelight;
Subscriber sub_watchdog;
Subscriber sub_enabler;
		
Publisher pub_twist;
Publisher pub_odom;
Publisher pub_jointstate;
Publisher pub_imu;
Publisher pub_imu_mag;
Publisher pub_imu_acc;
Publisher pub_imu_gyr;
Publisher pub_voltage;
Publisher pub_temperature;
Publisher pub_error_code;
Publisher pub_aux_bits;
Publisher pub_docked;
Publisher pub_remaining_optime;

Taurob_base* tb = 0;

double ecu_com_diag_min_freq = 4; 	// 1 msg every 250ms
double ecu_com_diag_max_freq = 40; 	// 1 msg every 25ms

diagnostic_updater::HeaderlessTopicDiagnostic* ecu_coms_diag = 0;
diagnostic_updater::Updater* updater;
diagnostic_updater::FunctionDiagnosticTask* temperature_checker;
diagnostic_updater::FunctionDiagnosticTask* voltage_checker;

float alpha = 0;
float x = 0;
float y = 0;
ros::Time last_rx_frame_time;
tf::TransformBroadcaster* tf_broadcaster;

string tf_from_frame;
string tf_to_frame;
bool publish_tf;
bool watchdog;
bool control_enabled;
string tracker_ip;
int tracker_port;
int protocol_version;

float supply_voltage = 0.0;
unsigned char error_code = 0;
float remaining_optime = 0.0;
float motor_driver_temperature = 0.0;
char dock_received = 0;


void publish_odometry_and_tf();
void publish_misc_data();
void publish_imu();
void publish_docked();

void sigint_handler(int sig)
{
    ros::shutdown();
}

double voltage_avg = 0;
const double VOLTAGE_AVG_ELEMENTS = 500;
const double REMAINING_V_TO_MIN_FACTOR = (4 * 60); 	// 4 hours, in minutes, with a full charge (1)
const double MAX_VOLTAGE = 26;
const double MIN_VOLTAGE = 22;
const int LOW_VOLTAGE_COUNT_THRESHOLD = 40; 		// we receive voltage with ~40 Hz, so 40 means ~1sec
const float LOW_VOLTAGE_THRESHOLD_NO_DRIVING = 22.0f;
const float LOW_VOLTAGE_THRESHOLD_WHILE_DRIVING = 20.0f;
int low_voltage_count = 0;

bool shutting_down = false;

// prepare system shutdown in case of low voltage
void prepare_shutdown()
{
	if (shutting_down == false)
	{
		::system("echo \"ATTENTION! THE ROBOT'S BATTERY IS VERY LOW!\" | wall");
		shutting_down = true;
	}
}

double calc_remaining_optime(float voltage)
{
	double ret = 0;
	
	if (tb != 0)
	{
		// update moving average
		if (voltage_avg == 0 && voltage != 0) 	// we just started up
		{
			ROS_INFO("initializing supply voltage averaging with %.1fV", voltage);
			voltage_avg = voltage;
		}
		else
		{
			voltage_avg -= (voltage_avg / VOLTAGE_AVG_ELEMENTS);
			voltage_avg += (voltage / VOLTAGE_AVG_ELEMENTS);
		}
		
		// calculate remaining time based on avg
		// min voltage is 22, max is 27
		double remaining_v = voltage_avg - MIN_VOLTAGE;
		if (remaining_v < 0) remaining_v = 0;
		if (remaining_v > (MAX_VOLTAGE - MIN_VOLTAGE) + 1) remaining_v = (MAX_VOLTAGE - MIN_VOLTAGE) + 1; 	 	// now we're in [0..(MAX_VOLTAGE - MIN_VOLTAGE) + 1]
		remaining_v /= (MAX_VOLTAGE - MIN_VOLTAGE); 	// map to [0..1.1]
		ret = remaining_v * REMAINING_V_TO_MIN_FACTOR;
	}
	
	return ret;
}

void check_temperature(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if (motor_driver_temperature > 70)
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Over 70 degrees");
	else if(motor_driver_temperature > 60)
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Over 60 degrees");
    else if(motor_driver_temperature > 50)
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Over 50 degrees");
    else
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Below 50 degrees");
		
   stat.add("Motor-driver temperature", motor_driver_temperature);
}

void check_voltage(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if (tb != 0)
	{
	    float supply_voltage = voltage_avg; 	// we could also use the unfiltered tb->Get_supply_voltage() here
	    boost::tuple<float, float> dist = tb->Get_motor_distance();
	    float threshold = ((get<0>(dist) > 0 || get<1>(dist) > 0) ? 
							    LOW_VOLTAGE_THRESHOLD_WHILE_DRIVING :
							    LOW_VOLTAGE_THRESHOLD_NO_DRIVING);
	    
	    if (supply_voltage > 10)
	    {
		if (supply_voltage < 22)
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Below 22.0V");
		else if (supply_voltage < 23)
			stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Below 23.0V");
		else
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Above 23.0V");
			
		stat.add("Supply voltage", voltage_avg);

		if (supply_voltage < threshold)
		{
			++low_voltage_count;
			if (low_voltage_count > LOW_VOLTAGE_COUNT_THRESHOLD)
			{
				prepare_shutdown();
			}
		}
		else if (low_voltage_count > 0)
		{
			--low_voltage_count;
		}
	    }	
	}
}

void On_frame_received()
{
	//ROS_INFO("A frame was received.\n");
	publish_odometry_and_tf();
	publish_imu();
	publish_misc_data();
	publish_docked();
		
    ecu_coms_diag->tick();
    updater->update();
}

void publish_docked()
{
	char auxbits = tb->Get_AUX_bits();
	if ((auxbits & 0x01) == 0) 	// in docking station
	{
		if (dock_received < 1)
		{
			dock_received++;
			std_msgs::Bool msg;
			msg.data = true;
			pub_docked.publish(msg);
		}
	}
	else
	{
		if (dock_received > 0)
		{
			dock_received--;
			std_msgs::Bool msg;
			msg.data = false;
			pub_docked.publish(msg);
		}
	}
}

void publish_misc_data()
{
	//ROS_INFO("publishing voltage %f", tb->Get_supply_voltage());
	std_msgs::Float32 fdata;

	motor_driver_temperature = calc_remaining_optime(tb->Get_supply_voltage());
    fdata.data = motor_driver_temperature;
    pub_remaining_optime.publish(fdata);
    
    supply_voltage = voltage_avg;
    fdata.data = supply_voltage;
    pub_voltage.publish(fdata);
	
	std_msgs::Byte bdata;

    error_code = tb->Get_error_code();
    bdata.data = error_code;
    pub_error_code.publish(bdata);
	
	bdata.data = tb->Get_AUX_bits();
	pub_aux_bits.publish(bdata);

    motor_driver_temperature = get<0>(tb->Get_temperatures());
    fdata.data = motor_driver_temperature;
    pub_temperature.publish(fdata);
}

void publish_imu()
{
	geometry_msgs::Vector3Stamped vec;
	vec.header.stamp = ros::Time::now();
	vec.header.frame_id = "base_link";
	
	// swap x, y and make z*=-1 to conform to ros standard
	tuple<float, float, float> data = tb->Get_gyro_data();
	vec.vector.x = get<1>(data);
	vec.vector.y = get<0>(data);
	vec.vector.z = -get<2>(data);
	pub_imu_gyr.publish(vec);
	
	data = tb->Get_acc_data();
	vec.vector.x = get<1>(data);
	vec.vector.y = get<0>(data);
	vec.vector.z = -get<2>(data);
	pub_imu_acc.publish(vec);
	
	data = tb->Get_mag_data();
	vec.vector.x = get<1>(data);
	vec.vector.y = get<0>(data);
	vec.vector.z = -get<2>(data);
	pub_imu_mag.publish(vec);	
}

void publish_odometry_and_tf()
{
	float dt = (ros::Time::fromBoost(tb->Get_last_frame_timestamp()) - last_rx_frame_time).toSec();
	last_rx_frame_time = ros::Time::now(); //ros::Time::fromBoost(tb->Get_last_frame_timestamp());
	
	// calculate odometry (x, y, alpha) from wheel distances and publish
	boost::tuple<float, float> dist = tb->Get_motor_distance();
	
	double distance_travelled_left_wheel = (get<0>(dist)
                * wheel_diameter * M_PI) / (gear_ratio * 24);
	double distance_travelled_right_wheel = (get<1>(dist)
			* wheel_diameter * M_PI) / (gear_ratio * 24);

	geometry_msgs::Twist drive_vector;
	drive_vector.linear.x = (distance_travelled_left_wheel
			- distance_travelled_right_wheel) / 2;
	drive_vector.linear.y = 0.0;
	drive_vector.linear.z = 0.0;

	drive_vector.angular.x = 0.0;
	drive_vector.angular.y = 0.0;
	drive_vector.angular.z = (double) (distance_travelled_left_wheel
			+ distance_travelled_right_wheel)
			/ (track_width * turning_geometry_factor);

	float dAlpha = drive_vector.angular.z;

  /*

	// calculate "covariance" -- whenever we turn, the error will be high, so set the covariance high
	double cov = odom_low_cov;
	if (dAlpha > odom_turn_high_cov_tolerance)
	{
		cov = odom_high_cov;
	}	
  */
  
        alpha = alpha + dAlpha;
	float dx = drive_vector.linear.x * cos(alpha);
	float dy = drive_vector.linear.x * sin(alpha);

	x = x + dx;
	y = y + dy;

	ROS_DEBUG("translation: x = %g, y = %g rotation: alpha = %g", x, y, alpha);
	
	// also, calculate velocities via dt
	float vx = dx / dt;
	float vy = dy / dt;
	float valpha = dAlpha / dt;

	// convert distance in drive_vector.linear to speed -> divide by publish rate
	drive_vector.linear.x /= dt;
	drive_vector.linear.y /= dt;

	// publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = last_rx_frame_time;
	odom.header.frame_id = tf_to_frame;

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(alpha);
	odom.pose.pose.orientation = odom_quat;
	
  // Set covariances for pose data to -1.0 to signify that it should not be used.
  // Position uncertainty grows unbounded over time, so should not write misleading
  // covariance data here.
  //for (size_t i = 0; i < 36; ++i){
  //  odom.pose.covariance[i] = 100000.0;
  //}
  odom.pose.covariance[0] = 100000.0;
  odom.pose.covariance[7] = 100000.0;
  odom.pose.covariance[14] = 100000.0;
  odom.pose.covariance[21] = 100000.0;
  odom.pose.covariance[28] = 100000.0;
  odom.pose.covariance[35] = 100000.0;

	// set velocity
	odom.child_frame_id = tf_from_frame;
	odom.twist.twist.linear.x = drive_vector.linear.x;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = valpha;
	
	// covariance
  if (std::abs(odom.twist.twist.angular.z) > angular_vel_z_covariance_threshold){
      odom.twist.covariance[0] = high_speed_linear_vel_variance_x;
      odom.twist.covariance[7] = high_speed_linear_vel_variance_yz;
      odom.twist.covariance[14] = high_speed_linear_vel_variance_yz;

      odom.twist.covariance[21] = high_speed_angular_vel_variance_roll_pitch;
      odom.twist.covariance[28] = high_speed_angular_vel_variance_roll_pitch;
      odom.twist.covariance[35] = high_speed_angular_vel_variance_yaw;
  }else{
      odom.twist.covariance[0] = low_speed_linear_vel_variance_x;
      odom.twist.covariance[7] = low_speed_linear_vel_variance_yz;
      odom.twist.covariance[14] = low_speed_linear_vel_variance_yz;

      odom.twist.covariance[21] = low_speed_angular_vel_variance_roll_pitch;
      odom.twist.covariance[28] = low_speed_angular_vel_variance_roll_pitch;
      odom.twist.covariance[35] = low_speed_angular_vel_variance_yaw;
  }

	pub_odom.publish(odom);
	
	// broadcast tf
	if (publish_tf == true)
	{
		tf::Quaternion rotation = tf::Quaternion(tf::Vector3(0, 0, 1.0), alpha); // rotation from Y axis
		tf::Vector3 translation = tf::Vector3(x, y, 0.0); // in meters
		tf::Transform transform = tf::Transform(rotation, translation);

		tf::StampedTransform stampedTransform = tf::StampedTransform(transform, last_rx_frame_time, tf_to_frame, tf_from_frame);

		tf_broadcaster->sendTransform(stampedTransform);
	}
}

void twist_cmd_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	//ROS_INFO("received twist");
	if (tb != 0)
	{
	//	ROS_INFO("sending twist");
		tb->Set_drive_command(make_tuple(msg->linear.x, msg->angular.z));
	}
}

void jointstate_cmd_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if (tb != 0)
	{
		// check if the joint state contains a joint called "gripper_front"
		std::vector<std::string> jointnames = msg->name;
		int index = 0;
		
		for (std::vector<string>::iterator it = jointnames.begin(); it != jointnames.end(); ++it)
		{
			const char* mit = (*it).c_str();
			if (strcmp(mit, "flipper_front") == 0)
			{
				// gripper_front was found -- get the set angle for it (position, in rad -> convert to degrees)
				float setpos = msg->position[index] * (180.0f/M_PI);
				ROS_DEBUG("setting flipper angle to %f", setpos);
				tb->Set_gripper_angle(setpos);
				
				// since there is nothing but grippers for now, break here
				break;
			}
			index++;
		}
	}
}

void light_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (tb != 0)
	{
		tb->Set_light(msg->data);
	}
}

void bluelight_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (tb != 0)
	{
		tb->Set_bluelight(msg->data);
	}
}

void watchdog_feed_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (tb != 0 && msg->data == true)
	{
		tb->Feed_watchdog();
	}
}

void enable_control_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (tb != 0)
	{
		ROS_INFO("setting control to %s", (msg->data ? (char*)"true" : (char*)"false"));
		tb->Set_pause_sending(!(msg->data)); 	// if control enabled, pause=false. if control disabled, pause=true.
	}
}


void get_parameters()
{
	ros::NodeHandle local_nh = NodeHandle(ros::this_node::getName());
	
	local_nh.param<bool>("publish_tf", publish_tf, false);
	local_nh.param<double>("wheel_diameter", wheel_diameter, DEFAULT_WHEEL_DIAMETER);
	local_nh.param<double>("track_width", track_width, DEFAULT_TRACK_WIDTH);
	local_nh.param<double>("gear_ratio", gear_ratio, DEFAULT_GEAR_RATIO);
	local_nh.param<double>("turning_geometry_factor", turning_geometry_factor, DEFAULT_TURNING_GEOMETRY_FACTOR);
  //local_nh.param<double>("odom_low_covariance", odom_low_cov, DEFAULT_ODOM_LOW_COV);
  //local_nh.param<double>("odom_high_covariance", odom_high_cov, DEFAULT_ODOM_HIGH_COV);
  //local_nh.param<double>("odom_turn_high_cov_tolerance", odom_turn_high_cov_tolerance, DEFAULT_ODOM_TURN_HIGH_COV_TOLERANCE);
	local_nh.param<string>("tf_from_frame", tf_from_frame, "/base_link");
	local_nh.param<string>("tf_to_frame", tf_to_frame, "/odom");
  local_nh.param<bool>("watchdog", watchdog, true);
  local_nh.param<bool>("control_enabled_at_startup", control_enabled, false);
  local_nh.param<string>("base_ip", tracker_ip, "10.0.0.2");
  local_nh.param<int>("base_port", tracker_port, 8080);
  local_nh.param<int>("protocol_version", protocol_version, 2); 	// default protocol is 2 (since that is the current one at the time of this writing)


  //covariance_treatment
  double tmp;
  local_nh.param<double>("angular_vel_z_covariance_threshold_deg", tmp, 1.0);
  angular_vel_z_covariance_threshold = tmp * (M_PI/180.0);

  local_nh.param<double>("low_speed_linear_vel_std_dev_x", tmp, 0.025);
  low_speed_linear_vel_variance_x = tmp * tmp;
  local_nh.param<double>("low_speed_linear_vel_std_dev_yz", tmp, 0.01);
  low_speed_linear_vel_variance_yz = tmp * tmp;

  local_nh.param<double>("high_speed_linear_vel_std_dev_x", tmp, 0.05);
  high_speed_linear_vel_variance_x = tmp * tmp;
  local_nh.param<double>("high_speed_linear_vel_std_dev_yz", tmp, 0.05);
  high_speed_linear_vel_variance_yz = tmp * tmp;

  local_nh.param<double>("low_speed_angular_vel_std_dev_yaw_deg", tmp, 3.0);
  low_speed_angular_vel_variance_yaw = ( (tmp * (M_PI/180.0)) * (tmp * (M_PI/180.0)) );
  local_nh.param<double>("low_speed_angular_vel_std_dev_roll_pitch_deg", tmp, 5.0);
  low_speed_angular_vel_variance_roll_pitch = ( (tmp * (M_PI/180.0)) * (tmp * (M_PI/180.0)) );

  local_nh.param<double>("high_speed_angular_vel_std_dev_yaw_deg", tmp, 120.0);
  high_speed_angular_vel_variance_yaw = ( (tmp * (M_PI/180.0)) * (tmp * (M_PI/180.0)) );
  local_nh.param<double>("high_speed_angular_vel_std_dev_roll_pitch_deg", tmp, 30.0);
  high_speed_angular_vel_variance_roll_pitch = ( (tmp * (M_PI/180.0)) * (tmp * (M_PI/180.0)) );

}

			
void init()
{
	signal(SIGINT, sigint_handler);
	
	get_parameters();
		
	sub_twist = nh->subscribe("cmd_vel", 1, &twist_cmd_callback);
	sub_jointstate = nh->subscribe("jointstate_cmd", 1, &jointstate_cmd_callback);
	sub_light = nh->subscribe("light", 1, &light_callback);
	sub_bluelight = nh->subscribe("bluelight", 1, &bluelight_callback);
	sub_watchdog = nh->subscribe("watchdog_feed", 1, &watchdog_feed_callback);
	sub_enabler = nh->subscribe("enable_control", 1, &enable_control_callback);
			
	// TODO: commented lines would be available, but still need to be published
	//pub_twist = nh->advertise<geometry_msgs::Twist>("twist_status", 1); 		// does this make sense when we have odom?
	pub_odom = nh->advertise<nav_msgs::Odometry>("odom", 1);
	//pub_jointstate = nh->advertise<sensor_msgs::JointState>("jointstate_status", 1);
	//pub_imu = nh->advertise<sensor_msgs::Imu>("imu_data", 1);
	pub_imu_mag = nh->advertise<geometry_msgs::Vector3Stamped>("imu_magnetometer", 1);
	pub_imu_acc = nh->advertise<geometry_msgs::Vector3Stamped>("imu_accelerometer", 1);
	pub_imu_gyr = nh->advertise<geometry_msgs::Vector3Stamped>("imu_gyroscope", 1);
	pub_voltage = nh->advertise<std_msgs::Float32>("supply_voltage", 1);
	pub_temperature = nh->advertise<std_msgs::Float32>("temperatures", 1);
	pub_error_code = nh->advertise<std_msgs::Byte>("error_code", 1);
	pub_aux_bits = nh->advertise<std_msgs::Byte>("aux_bits", 1);
    pub_remaining_optime = nh->advertise<std_msgs::Float32>("remaining_optime", 1);
	pub_docked = nh->advertise<std_msgs::Bool>("robot_docked", 1);

    updater->setHardwareID("taurob_base");

    temperature_checker = new diagnostic_updater::FunctionDiagnosticTask
							(
								"Motor-driver temperature", 
								boost::bind(&check_temperature, _1)
							);
	voltage_checker = new diagnostic_updater::FunctionDiagnosticTask
							(
								"Supply voltage",
								boost::bind(&check_voltage, _1)
							);
    updater->add(*temperature_checker);
    updater->add(*voltage_checker);

    ecu_coms_diag = new diagnostic_updater::HeaderlessTopicDiagnostic
							(
								"Base ECU Communication", 
								*updater, 
								diagnostic_updater::FrequencyStatusParam(&ecu_com_diag_min_freq, 
																		 &ecu_com_diag_max_freq, 
																		 0.1, 
																		 10)
							);
    ecu_coms_diag->addTask(temperature_checker);
    ecu_coms_diag->addTask(voltage_checker);

    updater->force_update();
	ROS_INFO("Base node initialization complete");
}


int main(int argc, char **argv) 
{
	ROS_INFO("starting base node");
	ros::init(argc, argv, "taurob_base_node");
	
	NodeHandle rosnh;
	nh = &rosnh;
	
	tf::TransformBroadcaster tf_bc;
	tf_broadcaster = &tf_bc;

    diagnostic_updater::Updater base_node_updater;
    updater = &base_node_updater;
	
	init();
	ROS_INFO("\ntaurob base node is running.\n");

	tb = new Taurob_base(tracker_ip, tracker_port, protocol_version, control_enabled);
	tb->Set_on_received_callback(&On_frame_received);
	tb->Set_watchdog_enabled(watchdog);
	tb->Run();

	while (ros::ok())
	{
		ros::spinOnce();
		updater->update();
		ros::Duration(0.02).sleep(); 	// 20ms
	}
	
	if (ecu_coms_diag != 0) delete ecu_coms_diag;
	if (temperature_checker != 0) delete temperature_checker;
	if (voltage_checker != 0) delete voltage_checker;
	
	tb->Stop();
	delete tb;
	tb = 0;
	
	ROS_INFO("Terminating.");	
	exit(0);
}


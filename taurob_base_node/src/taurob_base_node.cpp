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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <libtaurob_base/libtaurob_base.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

using namespace ros;
using namespace std;

#define DEFAULT_GEAR_RATIO                      25.0
#define DEFAULT_WHEEL_DIAMETER                  0.237
#define DEFAULT_TRACK_WIDTH                     0.55
#define DEFAULT_TURNING_GEOMETRY_FACTOR         1.476 	// by experimentation
#define DEFAULT_ODOM_LOW_COV                    1.0
#define DEFAULT_ODOM_HIGH_COV                   1000.0
#define DEFAULT_ODOM_TURN_HIGH_COV_TOLERANCE    0.08

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
double gear_ratio;
double wheel_diameter;
double track_width;
double turning_geometry_factor;

Subscriber sub_twist;
Subscriber sub_watchdog;
		
Publisher pub_twist;
Publisher pub_odom;
Publisher pub_voltage;
Publisher pub_temperature;
Publisher pub_dig_in;
Publisher pub_remaining_optime;
Publisher pub_current;
Publisher pub_measured_speed;
Publisher pub_cooldown_active;
Publisher pub_air_pressure;

Taurob_base* tb = 0;

double ecu_com_diag_min_freq = 4; 	// 1 msg every 250ms
double ecu_com_diag_max_freq = 40; 	// 1 msg every 25ms

diagnostic_updater::Updater* updater;
diagnostic_updater::FunctionDiagnosticTask* temperature_checker;
diagnostic_updater::FunctionDiagnosticTask* voltage_checker;
diagnostic_updater::FunctionDiagnosticTask* cooldown_checker;

float alpha = 0;
float x = 0;
float y = 0;

tf::TransformBroadcaster* tf_broadcaster;
string tf_from_frame;
string tf_to_frame;
bool publish_tf;
float remaining_optime = 0.0;
float motor_driver_temperature = 0.0;
float distance_travelled = 0;
char dock_received = 0;
bool last_cooldown_active = false;
double voltage_avg = 27.0;
const double VOLTAGE_AVG_ELEMENTS = 300;
const double REMAINING_V_TO_MIN_FACTOR = (3 * 30); 	// 1.5 hours, in minutes, with a full charge (1)
const double MAX_VOLTAGE = 25.5;
const double MIN_VOLTAGE = 21;
const int LOW_VOLTAGE_COUNT_THRESHOLD = 40; 		// we receive voltage with ~40 Hz, so 40 means ~1sec
const float LOW_VOLTAGE_THRESHOLD_NO_DRIVING = 21.0f;
const float LOW_VOLTAGE_THRESHOLD_WHILE_DRIVING = 19.5f;
int low_voltage_count = 0;
bool driving = false;
bool shutting_down = false;

void publish_odometry_and_tf(DRIVETRAIN_STATUS status);
void publish_misc_data(DRIVETRAIN_STATUS status);
void publish_dig_in_states(DRIVETRAIN_STATUS status);
int publish_misc_counter = 0;

void sigint_handler(int sig)
{
    ros::shutdown();
}

// prepare system shutdown in case of low voltage
void prepare_shutdown()
{
	if (shutting_down == false)
	{
		int result = ::system("echo \"ATTENTION! THE ROBOT'S BATTERY IS VERY LOW!\" | wall");
		shutting_down = true;
	}
}

double calc_remaining_optime(float voltage)
{
    double ret = 0;
    
    // only update voltage if we're not driving
    if (tb != 0 && driving == false)
    {
		// update moving average
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

    return ret;
}

void check_temperature(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if(motor_driver_temperature > 95)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Over 95 degrees");
    }
    else if(motor_driver_temperature > 85)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Over 85 degrees");
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
		
   stat.add("Motor-driver temperature", motor_driver_temperature);
}

void check_cooldown(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if (last_cooldown_active)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Cooldown active");
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }

    stat.add("Base motor cooldown idle", !last_cooldown_active);
}

void check_voltage(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    float threshold = LOW_VOLTAGE_THRESHOLD_NO_DRIVING;

	if (tb != 0)
	{
        if (distance_travelled > 0)
        {
            threshold = LOW_VOLTAGE_THRESHOLD_WHILE_DRIVING;
        }
	    
        if (voltage_avg > 10)
	    {
            if (voltage_avg < 22)
				stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Below 22.0V");
            else if (voltage_avg < 23)
				stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Below 23.0V");
			else
                stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
			
			stat.add("Supply voltage", voltage_avg);

            if (voltage_avg < threshold)
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

void On_frame_received(DRIVETRAIN_STATUS status)
{
    distance_travelled = status.distance_travelled_forward;
    motor_driver_temperature = status.temperature;

    publish_odometry_and_tf(status);
    publish_dig_in_states(status);

    if (publish_misc_counter < 10)
    {
        publish_misc_counter++;
    }
    else
    {
        publish_misc_data(status);
        publish_misc_counter = 0;
    }
		
    updater->update();
}

void publish_dig_in_states(DRIVETRAIN_STATUS status)
{
    std_msgs::Byte msg;
    msg.data = status.dig_in_1_state + (status.dig_in_2_state << 1);
    pub_dig_in.publish(msg);
}

void publish_misc_data(DRIVETRAIN_STATUS status)
{
	std_msgs::Float32 fdata;
    std_msgs::Bool bdata;

    fdata.data = calc_remaining_optime(status.voltage);
    pub_remaining_optime.publish(fdata);
    
    fdata.data = voltage_avg;
    pub_voltage.publish(fdata);

    fdata.data = status.temperature;
    pub_temperature.publish(fdata);

    fdata.data = status.left_motor_current + status.right_motor_current;
    pub_current.publish(fdata);

    fdata.data = status.driving_speed;
    pub_measured_speed.publish(fdata);

    fdata.data = status.air_pressure;
    pub_air_pressure.publish(fdata);

    if (last_cooldown_active != status.cooldown_active)
    {
        bdata.data = status.cooldown_active;
        pub_cooldown_active.publish(bdata);
    }

    last_cooldown_active = status.cooldown_active;

}

void publish_odometry_and_tf(DRIVETRAIN_STATUS status)
{
    ros::Time current_ros_time = ros::Time::now();

    alpha = alpha + status.angle_turned;

    float dx = status.distance_travelled_forward * cos(alpha);
    float dy = status.distance_travelled_forward * sin(alpha);

	x = x + dx;
	y = y + dy;

	// publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_ros_time;
	odom.header.frame_id = tf_to_frame;

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(alpha);
	odom.pose.pose.orientation = odom_quat;
	
    odom.pose.covariance[0] = 100000.0;
    odom.pose.covariance[7] = 100000.0;
    odom.pose.covariance[14] = 100000.0;
    odom.pose.covariance[21] = 100000.0;
    odom.pose.covariance[28] = 100000.0;
    odom.pose.covariance[35] = 100000.0;

	// set velocity
	odom.child_frame_id = tf_from_frame;
    odom.twist.twist.linear.x = status.driving_speed;
	odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = status.turning_speed;
	
	// covariance
    if (std::abs(odom.twist.twist.angular.z) > angular_vel_z_covariance_threshold)
    {
      odom.twist.covariance[0] = high_speed_linear_vel_variance_x;
      odom.twist.covariance[7] = high_speed_linear_vel_variance_yz;
      odom.twist.covariance[14] = high_speed_linear_vel_variance_yz;

      odom.twist.covariance[21] = high_speed_angular_vel_variance_roll_pitch;
      odom.twist.covariance[28] = high_speed_angular_vel_variance_roll_pitch;
      odom.twist.covariance[35] = high_speed_angular_vel_variance_yaw;
    }
    else
    {
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

        tf::StampedTransform stampedTransform = tf::StampedTransform(transform, current_ros_time, tf_to_frame, tf_from_frame);

		tf_broadcaster->sendTransform(stampedTransform);
	}
}

void twist_cmd_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if (tb != 0)
    {
        tb->Set_drive_command(msg->linear.x, msg->angular.z);
    }

    if (msg->linear.x != 0 || msg->angular.z != 0)
    {
        driving = true;
    }
    else
    {
        driving = false;
    }
}

void watchdog_feed_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (tb != 0 && msg->data == true)
	{
		tb->Feed_watchdog();
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
    local_nh.param<string>("tf_from_frame", tf_from_frame, "/base_link");
    local_nh.param<string>("tf_to_frame", tf_to_frame, "/odom");
    local_nh.param<string>("base_ip", ecu_ip, "10.0.0.2");
    local_nh.param<int>("base_port", ecu_port, 8080);

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
    sub_watchdog = nh->subscribe("watchdog_feed", 1, &watchdog_feed_callback);
		    
    pub_odom = nh->advertise<nav_msgs::Odometry>("odom", 1);
    pub_voltage = nh->advertise<std_msgs::Float32>("supply_voltage", 1);
    pub_temperature = nh->advertise<std_msgs::Float32>("temperatures", 1);
    pub_remaining_optime = nh->advertise<std_msgs::Float32>("remaining_optime", 1);
    pub_dig_in = nh->advertise<std_msgs::Byte>("base_node_dig_in", 1, true);
    pub_current = nh->advertise<std_msgs::Float32>("motor_current", 1);
    pub_measured_speed = nh->advertise<std_msgs::Float32>("measured_speed", 1);
    pub_cooldown_active = nh->advertise<std_msgs::Bool>("cooldown_active", 1);
    pub_air_pressure = nh->advertise<std_msgs::Float32>("air_pressure", 1);

    updater->setHardwareID("taurob_base");

    temperature_checker = new diagnostic_updater::FunctionDiagnosticTask
			    (
                    "Motor-driver Temperature",
				    boost::bind(&check_temperature, _1)
			    );
    voltage_checker = new diagnostic_updater::FunctionDiagnosticTask
			    (
                    "Supply Voltage",
				    boost::bind(&check_voltage, _1)
			    );
    cooldown_checker = new diagnostic_updater::FunctionDiagnosticTask
			    (
                    "Motors Cooldown",
                    boost::bind(&check_cooldown, _1)
			    );
    updater->add(*temperature_checker);
    updater->add(*voltage_checker);
    updater->add(*cooldown_checker);

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

    tb = new Taurob_base(ecu_ip, ecu_port, gear_ratio, wheel_diameter, track_width, turning_geometry_factor);
	tb->Set_on_received_callback(&On_frame_received);
	tb->Run();

    while (ros::ok())
    {
	    ros::spinOnce();
	    updater->update();
	    ros::Duration(0.02).sleep(); 	// 20ms
    }
    
    if (temperature_checker != 0) delete temperature_checker;
    if (voltage_checker != 0) delete voltage_checker;
    if (cooldown_checker != 0) delete cooldown_checker;
    
    tb->Stop();
    delete tb;
    tb = 0;
    
    ROS_INFO("Terminating.");	
    exit(0);
}


/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "teleop_twist_joy/teleop_twist_joy.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/JointTrajectoryControllerState.h"


namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void arm_joint_statesCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
    void flipper_joint_statesCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);

    ros::Subscriber joy_sub;
    ros::Subscriber arm_joint_states_sub_;
    ros::Subscriber flipper_joint_states_sub_;
    ros::Publisher cmd_vel_pub;
    ros::Publisher jointstate_pub;
    ros::Publisher arm_joints_pub_;
    ros::Publisher flipper_joints_pub_;

    int enable_button;
    int enable_turbo_button;
    int axis_linear;
    int axis_angular;
    int axis_flipper;
    int axis_arm_joint3_;
    int axis_arm_joint4_;
    int button_flipper_front_dn;
    int button_flipper_front_up;
    int button_flipper_rear_dn;
    int button_flipper_rear_up;
    double scale_linear;
    double scale_linear_turbo;
    double scale_angular;
    double scale_flipper;
    double scale_arm_joint3_;
    double scale_arm_joint4_;
    double scale_flipper_front;
    double scale_flipper_rear;
    std::vector<std::string> arm_joint_names_;
    std::vector<std::string> flipper_joint_names_;
    trajectory_msgs::JointTrajectoryPoint current_arm_joints_;
    trajectory_msgs::JointTrajectoryPoint current_flipper_joints_;

    bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
    pimpl_ = new Impl;

    pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    pimpl_->arm_joints_pub_= nh->advertise<trajectory_msgs::JointTrajectory>("/argonaut_robot/arm_traj_controller/command", 1);
    pimpl_->flipper_joints_pub_= nh->advertise<trajectory_msgs::JointTrajectory>("/argonaut_robot/flipper/flipper_traj_controller/command", 1);
    pimpl_->jointstate_pub = nh->advertise<sensor_msgs::JointState>("jointstate_cmd", 1);
    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);
    pimpl_->arm_joint_states_sub_ = nh->subscribe<control_msgs::JointTrajectoryControllerState>("/argonaut_robot/arm_traj_controller/state", 1, &TeleopTwistJoy::Impl::arm_joint_statesCallback, pimpl_);
    pimpl_->flipper_joint_states_sub_ = nh->subscribe<control_msgs::JointTrajectoryControllerState>("/argonaut_robot/flipper/flipper_traj_controller/state", 1, &TeleopTwistJoy::Impl::flipper_joint_statesCallback, pimpl_);

    nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
    nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

    nh_param->param<int>("axis_linear", pimpl_->axis_linear, 1);
    nh_param->param<double>("scale_linear", pimpl_->scale_linear, 0.5);
    nh_param->param<double>("scale_linear_turbo", pimpl_->scale_linear_turbo, 1.0);

    nh_param->param<int>("axis_angular", pimpl_->axis_angular, 0);
    nh_param->param<double>("scale_angular", pimpl_->scale_angular, 1.0);

    nh_param->param<int>("axis_flipper", pimpl_->axis_flipper, 2);
    nh_param->param<double>("scale_flipper", pimpl_->scale_flipper, 0.8);

    nh_param->param<int>("axis_arm_joint3", pimpl_->axis_arm_joint3_, 2);
    nh_param->param<double>("scale_arm_joint3", pimpl_->scale_arm_joint3_, 0.6);

    nh_param->param<int>("axis_arm_joint4", pimpl_->axis_arm_joint4_, 3);
    nh_param->param<double>("scale_arm_joint4", pimpl_->scale_arm_joint4_, 0.6);
    
    nh_param->param<int>("button_flipper_front_dn", pimpl_->button_flipper_front_dn, 1);
    nh_param->param<int>("button_flipper_front_up", pimpl_->button_flipper_front_up, 2);
    nh_param->param<int>("button_flipper_rear_dn", pimpl_->button_flipper_rear_dn, 3);
    nh_param->param<int>("button_flipper_rear_up", pimpl_->button_flipper_rear_up, 4);
    nh_param->param<double>("scale_flipper_front", pimpl_->scale_flipper_front, 0.6);
    nh_param->param<double>("scale_flipper_rear", pimpl_->scale_flipper_rear, 0.6);

    ROS_INFO_NAMED("TeleopTwistJoy", "Using axis %i for linear and axis %i for angular.",
                   pimpl_->axis_linear, pimpl_->axis_angular);
    ROS_INFO_NAMED("TeleopTwistJoy", "Teleop on button %i at scale %f linear, scale %f angular.",
                   pimpl_->enable_button, pimpl_->scale_linear, pimpl_->scale_angular);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
                        "Turbo on button %i at scale %f linear.", pimpl_->enable_turbo_button, pimpl_->scale_linear_turbo);

    pimpl_->sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    // Initializes with zeros by default.
    geometry_msgs::Twist cmd_vel_msg;
    sensor_msgs::JointState jsm;
    trajectory_msgs::JointTrajectory arm_traj;
    trajectory_msgs::JointTrajectory flipper_traj;
    trajectory_msgs::JointTrajectoryPoint arm_desired_joint_states;
    trajectory_msgs::JointTrajectoryPoint flipper_desired_joint_states;
    
    if (enable_turbo_button >= 0 && joy_msg->buttons.size() > enable_turbo_button && joy_msg->buttons[enable_turbo_button])
    {
        cmd_vel_msg.linear.x = joy_msg->axes[axis_linear] * scale_linear_turbo;
        cmd_vel_msg.angular.z = joy_msg->axes[axis_angular] * scale_angular;
        cmd_vel_pub.publish(cmd_vel_msg);

        jsm.header.frame_id = "flippers_front";
        jsm.header.stamp = ros::Time::now();
        jsm.name.push_back("flippers_front");
        jsm.position.push_back(joy_msg->axes[axis_flipper] * scale_flipper);
        jointstate_pub.publish(jsm);

        ros::Duration dur(0.5);
        for(int i=0; i< arm_joint_names_.size(); i++){
            arm_traj.joint_names.push_back(arm_joint_names_[i]);
            if(i==3){
                arm_desired_joint_states.positions.push_back(current_arm_joints_.positions.at(i) +  joy_msg->axes[axis_arm_joint3_] * scale_arm_joint3_);
            }else{
                if(i==4){
                    arm_desired_joint_states.positions.push_back(current_arm_joints_.positions.at(i) +  joy_msg->axes[axis_arm_joint4_] * scale_arm_joint4_);
                }else{
                    arm_desired_joint_states.positions.push_back(current_arm_joints_.positions.at(i));
                }
            }
        }
                
        for(int i=0; i< flipper_joint_names_.size(); i++){
            flipper_traj.joint_names.push_back(flipper_joint_names_[i]);
            if (i == 0)
            {
                flipper_desired_joint_states.positions.push_back(
                        current_flipper_joints_.positions.at(i) + 
                        joy_msg->buttons[button_flipper_front_up] * -scale_flipper_front + 
                        joy_msg->buttons[button_flipper_front_dn] * scale_flipper_front);
            }
            else
            {
                flipper_desired_joint_states.positions.push_back(
                        current_flipper_joints_.positions.at(i) + 
                        joy_msg->buttons[button_flipper_rear_up] * -scale_flipper_rear + 
                        joy_msg->buttons[button_flipper_rear_dn] * scale_flipper_rear);
            }
        }

        arm_desired_joint_states.time_from_start=dur;
        arm_traj.header.stamp = ros::Time::now();
        arm_traj.points.push_back(arm_desired_joint_states);
        arm_joints_pub_.publish(arm_traj);
        
        flipper_desired_joint_states.time_from_start=dur;
        flipper_traj.header.stamp = ros::Time::now();
        flipper_traj.points.push_back(arm_desired_joint_states);
        flipper_joints_pub_.publish(flipper_traj);

        sent_disable_msg = false;
    }
    else if (joy_msg->buttons.size() > enable_button && joy_msg->buttons[enable_button])
    {
        cmd_vel_msg.linear.x = joy_msg->axes[axis_linear] * scale_linear;
        cmd_vel_msg.angular.z = joy_msg->axes[axis_angular] * scale_angular;
        cmd_vel_pub.publish(cmd_vel_msg);

        jsm.header.frame_id = "flippers_front";
        jsm.header.stamp = ros::Time::now();
        jsm.name.push_back("flippers_front");
        jsm.position.push_back(joy_msg->axes[axis_flipper] * scale_flipper);
        jointstate_pub.publish(jsm);

        ros::Duration dur(0.5);
        for(int i=0; i< arm_joint_names_.size(); i++){
            arm_traj.joint_names.push_back(arm_joint_names_[i]);
            if(i==3){
                arm_desired_joint_states.positions.push_back(current_arm_joints_.positions.at(i) +  joy_msg->axes[axis_arm_joint3_] * scale_arm_joint3_);
            }else{
                if(i==4){
                    arm_desired_joint_states.positions.push_back(current_arm_joints_.positions.at(i) +  joy_msg->axes[axis_arm_joint4_] * scale_arm_joint4_);
                }else{
                    arm_desired_joint_states.positions.push_back(current_arm_joints_.positions.at(i));
                }
            }
        }
        
        for(int i=0; i< flipper_joint_names_.size(); i++){
            flipper_traj.joint_names.push_back(flipper_joint_names_[i]);
            if (i == 0)
            {
                flipper_desired_joint_states.positions.push_back(
                        current_flipper_joints_.positions.at(i) + 
                        joy_msg->buttons[button_flipper_front_up] * -scale_flipper_front + 
                        joy_msg->buttons[button_flipper_front_dn] * scale_flipper_front);
            }
            else
            {
                flipper_desired_joint_states.positions.push_back(
                        current_flipper_joints_.positions.at(i) + 
                        joy_msg->buttons[button_flipper_rear_up] * -scale_flipper_rear + 
                        joy_msg->buttons[button_flipper_rear_dn] * scale_flipper_rear);
            }
        }

        arm_desired_joint_states.time_from_start=dur;
        arm_traj.header.stamp= ros::Time::now();
        arm_traj.points.push_back(arm_desired_joint_states);
        arm_joints_pub_.publish(arm_traj);
              
        flipper_desired_joint_states.time_from_start=dur;
        flipper_traj.header.stamp = ros::Time::now();
        flipper_traj.points.push_back(flipper_desired_joint_states);
        flipper_joints_pub_.publish(flipper_traj);

        sent_disable_msg = false;
    }
    else
    {
        // When enable button is released, immediately send a single no-motion command
        // in order to stop the robot.
        if (!sent_disable_msg)
        {
            cmd_vel_pub.publish(cmd_vel_msg);

            for(int i=0; i< arm_joint_names_.size(); i++){
                    arm_desired_joint_states.positions.push_back(current_arm_joints_.positions.at(i));
                    arm_traj.joint_names.push_back(arm_joint_names_[i]);
            }
            
            for(int i=0; i< flipper_joint_names_.size(); i++){
                    flipper_desired_joint_states.positions.push_back(current_flipper_joints_.positions.at(i));
                    flipper_traj.joint_names.push_back(flipper_joint_names_[i]);
            }

            ros::Duration dur(0.5);
            arm_desired_joint_states.time_from_start=dur;
            arm_traj.header.stamp= ros::Time::now();
            arm_traj.points.push_back(arm_desired_joint_states);
            arm_joints_pub_.publish(arm_traj);

            flipper_desired_joint_states.time_from_start=dur;
            flipper_traj.header.stamp= ros::Time::now();
            flipper_traj.points.push_back(flipper_desired_joint_states);
            flipper_joints_pub_.publish(flipper_traj);
            
            sent_disable_msg = true;
        }
    }
}

void TeleopTwistJoy::Impl::arm_joint_statesCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    arm_joint_names_=msg->joint_names;
    current_arm_joints_= msg->actual;
}

void TeleopTwistJoy::Impl::flipper_joint_statesCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    flipper_joint_names_=msg->joint_names;
    current_flipper_joints_= msg->actual;
}

}  // namespace teleop_twist_joy

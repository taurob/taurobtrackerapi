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
#include "teleop_twist_joy/simple_inverse_kinematics.h"


namespace teleop_twist_joy
{


static double rad2deg(double radian)
{
    return (radian / (2 * M_PI)) * 360.0;
}

static double deg2rad(double deg)
{
    return (deg / 360.0) * (2 * M_PI);
}

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void arm_joint_statesCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
    void sensor_head_joint_statesCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
    void flipper_joint_statesCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);

    ros::Subscriber joy_sub;
    ros::Subscriber arm_joint_states_sub_;
    ros::Subscriber sensor_head_joint_states_sub_;
    ros::Subscriber flipper_joint_states_sub_;
    ros::Publisher cmd_vel_pub;
    ros::Publisher arm_joints_pub_;
    ros::Publisher sensor_head_joints_pub_;
    ros::Publisher flipper_joints_pub_;

    int enable_button;
    int enable_turbo_button;
    int axis_linear;
    int axis_angular;
    int axis_arm_yaw_head_;
    int axis_arm_pitch_head_;
    int axis_arm_endpoint_up_down_;
    int axis_arm_endpoint_forward_backward_;
    int button_flipper_front_dn;
    int button_flipper_front_up;
    int button_rotate_arm_left;
    int button_rotate_arm_right;
    double scale_linear;
    double scale_linear_turbo;
    double scale_angular;
    double scale_arm_yaw_head_;
    double scale_arm_pitch_head_;
    double scale_endpoint_up_down_;
    double scale_endpoint_forward_backward_;
    double scale_flipper_front;
    double scale_rotate_arm;
    std::vector<std::string> arm_joint_names_;
    std::vector<std::string> sensor_head_joint_names_;
    std::vector<std::string> flipper_joint_names_;
    trajectory_msgs::JointTrajectoryPoint current_arm_joints_;
    trajectory_msgs::JointTrajectoryPoint current_sensor_head_joints_;
    trajectory_msgs::JointTrajectoryPoint current_flipper_joints_;
    ModArm_angles current_get_angles;
    ModArm_angles current_set_angles;
    bool joint_control_active;
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
    pimpl_->arm_joints_pub_= nh->advertise<trajectory_msgs::JointTrajectory>("/arm_control/arm_traj_controller/command", 1);
    pimpl_->sensor_head_joints_pub_= nh->advertise<trajectory_msgs::JointTrajectory>("/arm_control/sensor_head_traj_controller/command", 1);
    pimpl_->flipper_joints_pub_= nh->advertise<trajectory_msgs::JointTrajectory>("/flipper_control/flipper_traj_controller/command", 1);
    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);
    pimpl_->arm_joint_states_sub_ = nh->subscribe<control_msgs::JointTrajectoryControllerState>("/arm_control/arm_traj_controller/state", 1, &TeleopTwistJoy::Impl::arm_joint_statesCallback, pimpl_);
    pimpl_->sensor_head_joint_states_sub_ = nh->subscribe<control_msgs::JointTrajectoryControllerState>("/arm_control/sensor_head_traj_controller/state", 1, &TeleopTwistJoy::Impl::sensor_head_joint_statesCallback, pimpl_);
    pimpl_->flipper_joint_states_sub_ = nh->subscribe<control_msgs::JointTrajectoryControllerState>("/flipper_control/flipper_traj_controller/state", 1, &TeleopTwistJoy::Impl::flipper_joint_statesCallback, pimpl_);

    nh_param->param<int>("enable_button", pimpl_->enable_button, 5);
    nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

    nh_param->param<int>("axis_linear", pimpl_->axis_linear, 4);
    nh_param->param<double>("scale_linear", pimpl_->scale_linear, 0.2);
    nh_param->param<double>("scale_linear_turbo", pimpl_->scale_linear_turbo, 0.0);

    nh_param->param<int>("axis_angular", pimpl_->axis_angular, 3);
    nh_param->param<double>("scale_angular", pimpl_->scale_angular, 0.2);

    nh_param->param<int>("axis_arm_yaw_head", pimpl_->axis_arm_yaw_head_, 0);
    nh_param->param<double>("scale_arm_yaw_head", pimpl_->scale_arm_yaw_head_, 0.1);

    nh_param->param<int>("axis_arm_pitch_head", pimpl_->axis_arm_pitch_head_, 1);
    nh_param->param<double>("scale_arm_pitch_head", pimpl_->scale_arm_pitch_head_, 0.01);

    nh_param->param<int>("axis_arm_endpoint_up_down", pimpl_->axis_arm_endpoint_up_down_, 7);
    nh_param->param<double>("scale_endpoint_up_down", pimpl_->scale_endpoint_up_down_, 0.01);

    nh_param->param<int>("axis_arm_endpoint_forward_backward", pimpl_->axis_arm_endpoint_forward_backward_, 6);
    nh_param->param<double>("scale_endpoint_forward_backward", pimpl_->scale_endpoint_forward_backward_, 0.01);
    
    nh_param->param<int>("button_flipper_front_dn", pimpl_->button_flipper_front_dn, 3);
    nh_param->param<int>("button_flipper_front_up", pimpl_->button_flipper_front_up, 0);
    nh_param->param<double>("scale_flipper_front", pimpl_->scale_flipper_front, 0.3);

    nh_param->param<int>("button_rotate_arm_left", pimpl_->button_rotate_arm_left, 2);
    nh_param->param<int>("button_rotate_arm_right", pimpl_->button_rotate_arm_right, 1);
    nh_param->param<double>("scale_rotate_arm", pimpl_->scale_rotate_arm, 0.2);

    ROS_INFO_NAMED("TeleopTwistJoy", "Using axis %i for linear and axis %i for angular.",
                   pimpl_->axis_linear, pimpl_->axis_angular);
    ROS_INFO_NAMED("TeleopTwistJoy", "Teleop on button %i at scale %f linear, scale %f angular.",
                   pimpl_->enable_button, pimpl_->scale_linear, pimpl_->scale_angular);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
                        "Turbo on button %i at scale %f linear.", pimpl_->enable_turbo_button, pimpl_->scale_linear_turbo);
    pimpl_->joint_control_active = false;
    pimpl_->current_get_angles = ModArm_angles();
    pimpl_->current_set_angles = ModArm_angles();
    pimpl_->sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    End_effector_coordinates delta = End_effector_coordinates();
    ros::Duration dur(0.5);

    // Initializes with zeros by default.
    geometry_msgs::Twist cmd_vel_msg;
    trajectory_msgs::JointTrajectory arm_traj;
    trajectory_msgs::JointTrajectory sensor_head_traj;
    trajectory_msgs::JointTrajectory flipper_traj;
    trajectory_msgs::JointTrajectoryPoint arm_desired_joint_states;
    trajectory_msgs::JointTrajectoryPoint sensor_head_desired_joint_states;
    trajectory_msgs::JointTrajectoryPoint flipper_desired_joint_states;
    
    /*if enable or turbo button is pressed */
    if ((enable_turbo_button >= 0 && joy_msg->buttons.size() > enable_turbo_button && joy_msg->buttons[enable_turbo_button]) ||
        (joy_msg->buttons.size() > enable_button && joy_msg->buttons[enable_button]))
    {
        /* publish cmd_vel */
        if (joy_msg->buttons[enable_turbo_button])
        {
            cmd_vel_msg.linear.x = joy_msg->axes[axis_linear] * scale_linear_turbo;
        }
        else
        {
            cmd_vel_msg.linear.x = joy_msg->axes[axis_linear] * scale_linear;
        }

        cmd_vel_msg.angular.z = joy_msg->axes[axis_angular] * scale_angular;
        cmd_vel_pub.publish(cmd_vel_msg);

        /* publish arm angles */
        delta.X += rad2deg(joy_msg->axes[axis_arm_endpoint_forward_backward_] * scale_endpoint_forward_backward_);
        delta.Y += rad2deg(joy_msg->axes[axis_arm_endpoint_up_down_] * scale_endpoint_up_down_);
        delta.Phi = rad2deg(joy_msg->axes[axis_arm_pitch_head_] * scale_arm_pitch_head_);

        if ((arm_joint_names_.size() == 3) &&
            (sensor_head_joint_names_.size() == 2))
        {
            current_get_angles.Joint_angle_1 = fmod(360 + rad2deg(current_arm_joints_.positions.at(1)), 360);
            current_get_angles.Joint_angle_2 = fmod(360 + rad2deg(current_arm_joints_.positions.at(2)), 360);
            current_get_angles.Joint_angle_3 = fmod(360 * 2 - rad2deg(current_sensor_head_joints_.positions.at(1)), 360);
        }

        /* if arm is controlled */
        if (((delta.X != 0) || (delta.Y != 0) || (delta.Phi != 0) || (joy_msg->axes[axis_arm_yaw_head_] != 0) ||
             (joy_msg->buttons[button_rotate_arm_left] != 0) || (joy_msg->buttons[button_rotate_arm_right] != 0)) &&
             (arm_joint_names_.size() == 3) &&
             (sensor_head_joint_names_.size() == 2))
        {            
            End_effector_coordinates current_end_effector_coordinates = End_effector_coordinates();
            End_effector_coordinates desiredEndEffectorCoordinates = End_effector_coordinates();

            if (joint_control_active == false)
            {
                /* perform forward kinematics of current angles */
                current_end_effector_coordinates = ModArm_kinematic::Get_end_effector_coordinates(current_get_angles);
                joint_control_active = true;
            }
            else
            {
                current_end_effector_coordinates = ModArm_kinematic::Get_end_effector_coordinates(current_set_angles);
            }


            /* calculate desired end effector coordinates from camera perspective */
            desiredEndEffectorCoordinates.X = current_end_effector_coordinates.X - (delta.X * cos((current_end_effector_coordinates.Phi * M_PI) / 180)) + (delta.Y * sin((current_end_effector_coordinates.Phi * M_PI) / 180));
            desiredEndEffectorCoordinates.Y = current_end_effector_coordinates.Y + (delta.X * sin((current_end_effector_coordinates.Phi * M_PI) / 180)) + (delta.Y * cos((current_end_effector_coordinates.Phi * M_PI) / 180));
            desiredEndEffectorCoordinates.Phi = fmod(360 + current_end_effector_coordinates.Phi + delta.Phi, 360);

            /* set joint 2,3,4 angle */
            current_set_angles = ModArm_kinematic::Get_ModArm_angles(desiredEndEffectorCoordinates, current_get_angles);

            for (int i=0; i< arm_joint_names_.size(); i++)
            {
                arm_traj.joint_names.push_back(arm_joint_names_[i]);
            }

            arm_desired_joint_states.positions.push_back(current_arm_joints_.positions.at(0) +
                                                         joy_msg->buttons[button_rotate_arm_left] * -scale_rotate_arm +
                                                         joy_msg->buttons[button_rotate_arm_right] * scale_rotate_arm);
            arm_desired_joint_states.positions.push_back(deg2rad(current_set_angles.Joint_angle_1));
            arm_desired_joint_states.positions.push_back(deg2rad(current_set_angles.Joint_angle_2));

            for (int i=0; i< sensor_head_joint_names_.size(); i++)
            {
                sensor_head_traj.joint_names.push_back(sensor_head_joint_names_[i]);
            }

            sensor_head_desired_joint_states.positions.push_back(current_sensor_head_joints_.positions.at(0) + joy_msg->axes[axis_arm_yaw_head_] * scale_arm_yaw_head_);

            double corrected_joint3_angle = 360 - current_set_angles.Joint_angle_3;

            if (corrected_joint3_angle > 270)
            {
                corrected_joint3_angle -= 360;
            }

            sensor_head_desired_joint_states.positions.push_back(deg2rad(corrected_joint3_angle));

            arm_desired_joint_states.time_from_start=dur;
            arm_traj.points.push_back(arm_desired_joint_states);
            arm_joints_pub_.publish(arm_traj);

            sensor_head_desired_joint_states.time_from_start=dur;
            sensor_head_traj.points.push_back(sensor_head_desired_joint_states);
            sensor_head_joints_pub_.publish(sensor_head_traj);
        }
        else
        {
            joint_control_active = false;

            for (int i=0; i< arm_joint_names_.size(); i++)
            {
                arm_desired_joint_states.positions.push_back(current_arm_joints_.positions.at(i));
                arm_traj.joint_names.push_back(arm_joint_names_[i]);
            }

            ros::Duration dur(0.5);
            arm_desired_joint_states.time_from_start=dur;
            arm_traj.points.push_back(arm_desired_joint_states);
            arm_joints_pub_.publish(arm_traj);
        }
                
        for (int i=0; i< flipper_joint_names_.size(); i++)
        {
            flipper_traj.joint_names.push_back(flipper_joint_names_[i]);

            if (i == 0)
            {
                flipper_desired_joint_states.positions.push_back(
                        current_flipper_joints_.positions.at(i) + 
                        joy_msg->buttons[button_flipper_front_up] * -scale_flipper_front + 
                        joy_msg->buttons[button_flipper_front_dn] * scale_flipper_front);
            }         
        }       

        flipper_desired_joint_states.time_from_start=dur;
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

            for (int i=0; i< arm_joint_names_.size(); i++)
            {
                arm_desired_joint_states.positions.push_back(current_arm_joints_.positions.at(i));
                arm_traj.joint_names.push_back(arm_joint_names_[i]);
            }
            
            for (int i=0; i< flipper_joint_names_.size(); i++)
            {
                flipper_desired_joint_states.positions.push_back(current_flipper_joints_.positions.at(i));
                flipper_traj.joint_names.push_back(flipper_joint_names_[i]);
            }

            ros::Duration dur(0.5);
            arm_desired_joint_states.time_from_start=dur;
            arm_traj.points.push_back(arm_desired_joint_states);
            arm_joints_pub_.publish(arm_traj);

            flipper_desired_joint_states.time_from_start=dur;
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

void TeleopTwistJoy::Impl::sensor_head_joint_statesCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    sensor_head_joint_names_=msg->joint_names;
    current_sensor_head_joints_= msg->actual;
}

void TeleopTwistJoy::Impl::flipper_joint_statesCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    flipper_joint_names_=msg->joint_names;
    current_flipper_joints_= msg->actual;
}

}  // namespace teleop_twist_joy

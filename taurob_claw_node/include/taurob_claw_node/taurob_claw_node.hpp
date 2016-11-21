/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Christian Rose
 *                      Team Hector,
 *                      Technische Universität Darmstadt
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Technische Universität Darmstadt nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef taurob_claw_node_hpp___
#define taurob_claw_node_hpp___

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <hardware_interface/robot_hw.h>

#include <libtaurob_claw/libtaurob_claw.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <ros/callback_queue.h>

using namespace ros;
using namespace std;

namespace taurob_claw_node
{

class TaurobClawNode : public hardware_interface::RobotHW
{
public:
    TaurobClawNode(ros::NodeHandle &nh, boost::shared_ptr<Claw> &claw);
    void init();
    void cleanup();

    void read(ros::Time time, ros::Duration period);
    void write(ros::Time time, ros::Duration period);

protected:

  void watchdog_feed_callback(const std_msgs::Bool::ConstPtr& msg);
  void enable_control_callback(const std_msgs::Bool::ConstPtr& msg);

public:

  bool publish_tf;
  bool watchdog;
  double rot_factor;
  double rot_offset;
  double grip_factor;
  double grip_offset;

private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    ros::NodeHandle nh_;

    Subscriber sub_watchdog, sub_enabler;

    boost::shared_ptr<Claw> claw;

public:
    double joint_position_rotation;
    double joint_velocity_rotation;
    double joint_effort_rotation;
    double joint_pos_cmd_rotation;

    double joint_position_grip;
    double joint_velocity_grip;
    double joint_effort_grip;
    double joint_pos_cmd_grip;

};

}

#endif

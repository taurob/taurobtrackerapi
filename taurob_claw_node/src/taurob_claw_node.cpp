#include "taurob_claw_node/taurob_claw_node.hpp"

#include <controller_manager/controller_manager.h>

#include <ros/param.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <sstream>




#include <boost/tuple/tuple.hpp>
#include <XmlRpcValue.h>

using namespace ros;
using namespace std;

const string JOINT_ROTAION = "arm_joint_4";
const string JOINT_GRIP = "gripper_joint_0";


namespace taurob_claw_node
{

  TaurobClawNode::TaurobClawNode(ros::NodeHandle &nh, boost::shared_ptr<Claw> &claw) : nh_(nh), claw(claw)
  {
    ROS_INFO("TaurobClawNode()");
  }

  void TaurobClawNode::watchdog_feed_callback(const std_msgs::Bool::ConstPtr& msg)
  {
    if (msg->data == true)
    {
      claw->Feed_watchdog();
    }
  }

  void TaurobClawNode::enable_control_callback(const std_msgs::Bool::ConstPtr& msg)
  {
    ROS_INFO("setting control to %s", (msg->data ? (char*)"true" : (char*)"false"));
    claw->Set_pause_sending(!(msg->data)); 	// if control enabled, pause=false. if control disabled, pause=true.
  }


  void TaurobClawNode::init()
  {
    ROS_INFO("reading parameters...");

    // get parameters -- try to, at least
    try
    {
      nh_.param<bool>("watchdog", watchdog, true);
      ROS_INFO("Safeguard (Watchdog or Command RX Timeout): %s", (char*)(watchdog ? "Watchdog" : "Command RX Timeout"));
      nh_.param<double>("rot_factor", rot_factor, 1.0);
      nh_.param<double>("rot_offset", rot_offset, 0.0);
      nh_.param<double>("grip_factor", grip_factor, 1.0);
      nh_.param<double>("grip_offset", grip_offset, 0.0);
      ROS_INFO("Factor: rot: %f, grip: %f", rot_factor, grip_factor);
      ROS_INFO("Offsets: rot: %f, grip: %f", rot_offset, grip_offset);
    }
    catch(...)
    {
      ROS_ERROR("No configuration found -- won't be doing anything.");
    }

    //sub_jointstate = nh_->subscribe("jointstate_cmd", 1, &jointstate_cmd_callback);
    sub_watchdog = nh_.subscribe("watchdog_feed", 1, &TaurobClawNode::watchdog_feed_callback, this);
    sub_enabler = nh_.subscribe("enable_control", 1, &TaurobClawNode::enable_control_callback, this);
    //pub_jointstate = nh_->advertise<sensor_msgs::JointState>("jointstate_status", 1);
    //pub_force = nh_.advertise<std_msgs::Float64>("claw_force", 1);

    hardware_interface::JointStateHandle state_handle_rot(JOINT_ROTAION, &joint_position_rotation, &joint_velocity_rotation, &joint_effort_rotation);
    joint_state_interface_.registerHandle(state_handle_rot);

    hardware_interface::JointHandle pos_handle_rot(joint_state_interface_.getHandle(JOINT_ROTAION), &joint_pos_cmd_rotation);
    position_joint_interface_.registerHandle(pos_handle_rot);

    hardware_interface::JointStateHandle state_handle_grip(JOINT_GRIP, &joint_position_grip, &joint_velocity_grip, &joint_effort_grip);
    joint_state_interface_.registerHandle(state_handle_grip);

    hardware_interface::JointHandle pos_handle_grip(joint_state_interface_.getHandle(JOINT_GRIP), &joint_pos_cmd_grip);
    position_joint_interface_.registerHandle(pos_handle_grip);

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    nh_.param<double>("rot_init_pos", joint_pos_cmd_rotation, 0.0);

    nh_.param<double>("grip_init_pos", joint_pos_cmd_grip, 0.0);
    ROS_INFO("Initialization complete");
  }

  void TaurobClawNode::cleanup(){

  }

  void TaurobClawNode::read(ros::Time time, ros::Duration period){
    //Done in On_claw_receive()
  }

  void TaurobClawNode::write(ros::Time time, ros::Duration period){
    ROS_DEBUG_THROTTLE(1,"rotation_to_command: %f", joint_pos_cmd_rotation);
    double rotation = rot_factor*joint_pos_cmd_rotation - rot_offset;
    claw->Set_rotation(rotation);
    ROS_DEBUG_THROTTLE(1,"rotation_commanded: %f", rotation);

    ROS_DEBUG_THROTTLE(1,"gripper_to_command: %f", joint_pos_cmd_grip);
    double grip = grip_factor*joint_pos_cmd_grip - grip_offset;
    claw->Set_grip(grip);
    ROS_DEBUG_THROTTLE(1,"gripper_commanded: %f", grip);

  }

}

std::string ip_address;
int port;

bool control_enabled;

boost::shared_ptr<taurob_claw_node::TaurobClawNode> claw_node;
boost::shared_ptr<Claw> claw;

void On_claw_receive()
{
  ROS_DEBUG("Claw received; rot angle is %f, grip angle is %f", claw->Get_rotation(), claw->Get_grip());

  double rot = claw->Get_rotation() ;

  ROS_DEBUG_THROTTLE(1, "rot input: %f", rot);
  rot = (rot + claw_node->rot_offset)/claw_node->rot_factor;
  ROS_DEBUG_THROTTLE(1, "rot output: %f", rot);
  while (rot > M_PI) rot -= 2*M_PI;
  while (rot < -M_PI) rot += 2*M_PI;

  claw_node->joint_position_rotation = rot;

  double grip = claw->Get_grip();
  while (grip > M_PI) grip -= 2*M_PI;
  while (grip < -M_PI) grip += 2*M_PI;

  ROS_DEBUG_THROTTLE(1, "grip input: %f", grip);
  grip = (grip + claw_node->grip_offset)/claw_node->grip_factor;
  ROS_DEBUG_THROTTLE(1, "grip output: %f", grip);

  claw_node->joint_position_grip = grip;

  claw_node->joint_effort_grip = claw->Get_force();

}


int main(int argc, char** argv){

  try{
    ROS_INFO("starting");
    ros::init(argc, argv, "taurob_claw_node");

    ros::NodeHandle pnh("~");


    pnh.param<std::string>("ip_address", ip_address, "10.0.0.50");
    ROS_INFO("IP address: %s", ip_address.c_str());

    pnh.param<int>("port", port, 1235);
    ROS_INFO("Port: %d", port);

    pnh.param<bool>("control_enabled_at_startup", control_enabled, true);
    ROS_INFO("Robot control enabled at startup: %s", (char*)(control_enabled ? "true" : "false"));


    claw.reset(new Claw (ip_address, port, control_enabled));

    claw_node.reset(new taurob_claw_node::TaurobClawNode(pnh, claw));
    claw_node->init();

    claw->Set_on_received_callback(&On_claw_receive);
    claw->Set_watchdog_enabled(claw_node->watchdog);
    claw->Run();

    ROS_DEBUG("ControllerManager - setup");
    controller_manager::ControllerManager cm(&(*claw_node), pnh);
    ROS_DEBUG("ControllerManager - done");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate loop_rate(50);

    ros::Time last_time = ros::Time::now();

    ROS_DEBUG("pre loop");
    while (ros::ok())
    {
      //ROS_INFO("in main loop");
      loop_rate.sleep();

      ros::Time current_time = ros::Time::now();
      ros::Duration elapsed_time = current_time - last_time;
      last_time = current_time;

      //ROS_INFO("before read");
      claw_node->read(current_time, elapsed_time);
      //ROS_INFO("after read");

      //ROS_INFO("before cm.update");
      cm.update(current_time, elapsed_time);
      //ROS_INFO("after cm.update");

      //ROS_INFO("before write");
      claw_node->write(current_time, elapsed_time);
      //ROS_INFO("after write");
    }

    claw_node->cleanup();

    claw->Stop();
  }
  catch(...)
  {
    ROS_ERROR("Unhandled exception!");
    return -1;
  }

  return 0;
}

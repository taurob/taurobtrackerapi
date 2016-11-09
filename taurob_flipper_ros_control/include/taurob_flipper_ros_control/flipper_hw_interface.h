#ifndef FLIPPER_HW_INTERFACE_H_
#define FLIPPER_HW_INTERFACE_H_

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <libtaurob_arm/libtaurob_arm.h>

#include <string>
#include <sstream>
#include <vector>

class Flipper_hw_interface : public hardware_interface::RobotHW
{
	public:
		Flipper_hw_interface(Flipper* man, float offset);
		~Flipper_hw_interface();
		
		void read();
		void write();

	private:
		template <typename T>
		std::string to_string(T value)
		{
			std::ostringstream os ;
			os << value ;
			return os.str() ;
		}

		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;
		
		std::vector<float> offsets;
		std::vector<double> cmds;
		std::vector<double> poss;
		std::vector<double> vels;
		std::vector<double> effs;
		
		// if we receive NaN (invalid) position from actor, we still write out the last valid position
		// so other nodes get a valid jointstate. however, to avoid dangerous side-effects, we suppress 
		// writing to the actor while we get a NaN position
		std::vector<double> last_valid_poss;
		std::vector<bool> received_nan_pos; 	// just used to write appropriate warnings to console without spamming it
		bool suppress_writing; 
		
		std::vector<hardware_interface::JointStateHandle*> joint_state_handles;
		std::vector<hardware_interface::JointHandle*> joint_handles;
		
		Flipper* flipper;
};

#endif

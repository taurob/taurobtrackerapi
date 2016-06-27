/**************************************************************************//**
 *
 * @file arm_hw_interface.h
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

#ifndef ARM_HW_INTERFACE_H_
#define ARM_HW_INTERFACE_H_

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include <libtaurob_arm/libtaurob_arm.h>

#include <string>
#include <sstream>
#include <vector>

class Arm_hw_interface : public hardware_interface::RobotHW
{
	public:
		Arm_hw_interface(Arm* man, std::vector<float> offsets);
		~Arm_hw_interface();
		
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
		
		Arm* manipulator;
};

#endif

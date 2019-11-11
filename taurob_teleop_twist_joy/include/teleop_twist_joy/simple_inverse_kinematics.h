/**************************************************************************
 *
 * @file simple_inverse_kinematics.h
 * @author Lukas Silberbauer, taurob GmbH
 * @date 02 March 2017
 * @brief Simple inverse kinematics for the taurob ModArm
 *
 *
 *  Copyright (c) 2017 taurob GmbH. All rights reserved.
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

#ifndef SIMPLE_INVERSE_KINEMATICS_H
#define SIMPLE_INVERSE_KINEMATICS_H


namespace teleop_twist_joy
{

    struct ModArm_angles
    {
       double Joint_angle_1;  //!< modarm joint 1 angle in degrees, 0 means Arm A is horizontally aligned with the base platform,
                             ///  90 degrees means the modarm stands perpendicular on the base platform
       double Joint_angle_2;  //!< modarm joint 2 angle in degrees, 0 means Arm B is horizontally aligned with Arm A,
                             /// positive angle means the Arm B goes up if Arm A is horizontolly aligned with the base platform.
       double Joint_angle_3;  //!< modarm joint 3 angle in degrees, 0 means the camera is looking straight forward if
                             /// the arm ist completly folded

       ModArm_angles()
       {
           Joint_angle_1 = 0;
           Joint_angle_2 = 0;
           Joint_angle_3 = 0;
       }
    };

    struct End_effector_coordinates
    {
        double X;     //!< x coordinate of end effector
        double Y;     //!< y coordinate of end effector
        double Phi;   //!< phi is the angle of the end effector relative to horizontal

        End_effector_coordinates()
        {
            X = 0;
            Y = 0;
            Phi = 0;
        }
    };

    class ModArm_kinematic
    {
        public:
            /** Performs inverse kinematics to obtain modarm angles */
            static ModArm_angles Get_ModArm_angles(End_effector_coordinates future_endpoint, ModArm_angles current_angles);

            /** Performs forward kinematics to obtain end effector coordinates */
            static End_effector_coordinates Get_end_effector_coordinates(ModArm_angles angles);

        private:

            static constexpr double ELEMENT_A_LENGTH = 551;
            static constexpr double ELEMENT_B_LENGTH = 435;

            /** Performs forward kinematics to obtain joint 2 coordinates */
            static End_effector_coordinates Get_joint_2_coordinates(ModArm_angles angles);
    };
}

#endif  // SIMPLE_INVERSE_KINEMATICS_H

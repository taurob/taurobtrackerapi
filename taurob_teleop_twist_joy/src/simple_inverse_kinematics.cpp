/**************************************************************************
 *
 * @file simple_inverse_kinematics.c
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

#include "teleop_twist_joy/simple_inverse_kinematics.h"
#include <math.h>
#include <stdlib.h>

double math_round(double value, int digits)
{
  return floor(value * pow(10, digits) + 0.5) / pow(10, digits);
}

namespace teleop_twist_joy
{
    ModArm_angles ModArm_kinematic::Get_ModArm_angles(End_effector_coordinates future_endpoint, ModArm_angles current_angles)
    {
        /* inverse kinematics is done via 2 circles, one originating from joint 1 with radius ELEMENT_A_LENGTH
         * and one originating from joint 3 with radius ELEMENT_B_LENGTH. Joint 2 must lie on the intersection
         * of these two circles.
         */
        ModArm_angles futureAngles = ModArm_angles();
        End_effector_coordinates future_joint_2 = End_effector_coordinates();
        End_effector_coordinates current_joint_2 = Get_joint_2_coordinates(current_angles);

        /* first check if effectorX equals zero, which leads to a special treatment, because of a division by zero */
        if (math_round(future_endpoint.X, 5) == 0) // sufficiently close to zero, to prevent overflow in else branch
        {
            future_joint_2.Y = (pow(ELEMENT_A_LENGTH, 2) - pow(ELEMENT_B_LENGTH, 2) + pow(future_endpoint.Y, 2)) / (2 * future_endpoint.Y);

            double determinant = pow(ELEMENT_A_LENGTH, 2) - pow(future_joint_2.Y, 2);

            if (determinant < 0)
            {
                /* the two circles don't intersect, so the effector coordinates must lie outside of the
                 * modarm's reach
                 */
                future_joint_2.X = current_joint_2.X;
                future_joint_2.Y = current_joint_2.Y;
            }
            else
            {
                /* for the x-coordinate the solution is abiguous, always choose nearest solution */
                double x_1 = sqrt(determinant);
                double x_2 = -x_1;

                if (abs(x_1 - current_joint_2.X) < abs(x_2 - current_joint_2.X))
                {
                    future_joint_2.X = x_1;
                }
                else
                {
                    future_joint_2.X = x_2;
                }
            }
        }
        else
        {
            double k1 = (pow(ELEMENT_A_LENGTH, 2) - pow(ELEMENT_B_LENGTH, 2) + pow(future_endpoint.X, 2) + pow(future_endpoint.Y, 2)) / (2 * future_endpoint.X);
            double k2 = future_endpoint.Y / future_endpoint.X;

            double determinant = (pow(ELEMENT_A_LENGTH, 2) * pow(k2, 2)) - pow(k1, 2) + pow(ELEMENT_A_LENGTH, 2);

            if (determinant < 0)
            {
                /* the two circles don't intersect, so the effector coordinates must lie outside of the
                 * modarm's reach
                 */

                double distance2origin = sqrt(pow(future_endpoint.X, 2) + pow(future_endpoint.Y, 2));

                /* if the end effector coordinates lie within same quadrant as element a, stretch the modarm to its maximum
                 * in order to approximate the given coordinates
                 */
                if ((future_endpoint.Y >= 0) && (distance2origin > ELEMENT_A_LENGTH))
                {
                    /* equations:
                     * (1) (a+b)^2 = x^2 + y^2
                     * (2) y = kx + d
                     */

                    /* for the x-coordinate the solution is abiguous, always choose nearest solution */
                    double x_1 = sqrt(pow(ELEMENT_A_LENGTH + ELEMENT_B_LENGTH, 2) / (1 + (pow(future_endpoint.Y, 2) / pow(future_endpoint.X, 2))));
                    double x_2 = -x_1;

                    if (abs(x_1 - future_endpoint.X) < abs(x_2 - future_endpoint.X))
                    {
                        future_endpoint.X = x_1;
                    }
                    else
                    {
                        future_endpoint.X = x_2;
                    }

                    future_endpoint.Y = (future_endpoint.Y / future_endpoint.X) * future_endpoint.X;

                    future_joint_2.X = future_endpoint.X * ELEMENT_A_LENGTH / (ELEMENT_A_LENGTH + ELEMENT_B_LENGTH);
                    future_joint_2.Y = future_endpoint.Y * ELEMENT_A_LENGTH / (ELEMENT_A_LENGTH + ELEMENT_B_LENGTH);
                }
                else if ((future_endpoint.Y < 0) && (distance2origin > ELEMENT_A_LENGTH))
                {
                    /* move arm a to the bottom and approximate to the given coordinates by moving arm b */
                    if (current_joint_2.X >= 0)
                    {
                        future_joint_2.X = ELEMENT_A_LENGTH;
                    }
                    else
                    {
                        future_joint_2.X = -ELEMENT_A_LENGTH;
                    }
                    future_joint_2.Y = 0;

                    double vector_x = future_endpoint.X - future_joint_2.X;
                    double vector_y = future_endpoint.Y - future_joint_2.Y;

                    future_endpoint.X = future_joint_2.X + ((vector_x * ELEMENT_B_LENGTH) / sqrt(pow(vector_x, 2) + pow(vector_y, 2)));
                    future_endpoint.Y = future_joint_2.Y + ((vector_y * ELEMENT_B_LENGTH) / sqrt(pow(vector_x, 2) + pow(vector_y, 2)));
                }
                else
                {
                    /* modarm can't reach end coordinates in its folded state, try to approximate
                     * end effector coordinates within reach of element b
                     */
                    future_joint_2.X = current_joint_2.X;
                    future_joint_2.Y = current_joint_2.Y;

                    double vectorX = future_endpoint.X - future_joint_2.X;
                    double vectorY = future_endpoint.Y - future_joint_2.Y;

                    future_endpoint.X = future_joint_2.X + ((vectorX * ELEMENT_B_LENGTH) / sqrt(pow(vectorX, 2) + pow(vectorY, 2)));
                    future_endpoint.Y = future_joint_2.Y + ((vectorY * ELEMENT_B_LENGTH) / sqrt(pow(vectorX, 2) + pow(vectorY, 2)));
                }
            }
            else
            {
                double future_joint_2_y_1 = (k1 * k2 + sqrt(determinant)) / (pow(k2, 2) + 1);
                double future_joint_2_y_2 = (k1 * k2 - sqrt(determinant)) / (pow(k2, 2) + 1);

                double future_joint_2_x_1 = k1 - k2 * future_joint_2_y_1;
                double future_joint_2_x_2 = k1 - k2 * future_joint_2_y_2;

                /* determine which solution to use by taking the current position of joint 3 into
                 * account
                 */
                double distance_1 = sqrt(pow(current_joint_2.X - future_joint_2_x_1, 2) + pow(current_joint_2.Y - future_joint_2_y_1, 2));
                double distance_2 = sqrt(pow(current_joint_2.X - future_joint_2_x_2, 2) + pow(current_joint_2.Y - future_joint_2_y_2, 2));

                if ((distance_1 <= distance_2) && (future_joint_2_y_1 >= 0))
                {
                    future_joint_2.Y = future_joint_2_y_1;
                    future_joint_2.X = future_joint_2_x_1;
                }
                else if ((distance_2 < distance_1) && (future_joint_2_y_2 >= 0))
                {
                    future_joint_2.Y = future_joint_2_y_2;
                    future_joint_2.X = future_joint_2_x_2;
                }
                else
                {
                    /* modarm tries to reach a point outside its operation space, try to approximate solution */

                    /* move element A to left boundary */
                    if (current_joint_2.X >= 0)
                    {
                        future_joint_2.X = ELEMENT_A_LENGTH;
                        future_joint_2.Y = 0;

                        double vectorX = future_endpoint.X - future_joint_2.X;
                        double vectorY = future_endpoint.Y - future_joint_2.Y;

                        future_endpoint.X = future_joint_2.X + ((vectorX * ELEMENT_B_LENGTH) / sqrt(pow(vectorX, 2) + pow(vectorY, 2)));
                        future_endpoint.Y = future_joint_2.Y + ((vectorY * ELEMENT_B_LENGTH) / sqrt(pow(vectorX, 2) + pow(vectorY, 2)));
                    }
                    else
                    {
                        /* move element A to right boundary */
                        future_joint_2.X = -ELEMENT_A_LENGTH;
                        future_joint_2.Y = 0;

                        double vectorX = future_endpoint.X - future_joint_2.X;
                        double vectorY = future_endpoint.Y - future_joint_2.Y;

                        future_endpoint.X = future_joint_2.X + ((vectorX * ELEMENT_B_LENGTH) / sqrt(pow(vectorX, 2) + pow(vectorY, 2)));
                        future_endpoint.Y = future_joint_2.Y + ((vectorY * ELEMENT_B_LENGTH) / sqrt(pow(vectorX, 2) + pow(vectorY, 2)));
                    }
                }
            }
        }

        /* JointAngle1 should not have negative values, so ignore negative solution of the following equation */
        futureAngles.Joint_angle_1 = fmod(360 + (acos(future_joint_2.X / ELEMENT_A_LENGTH) * 180) / M_PI, 360);

        double acos_var = (future_joint_2.X - future_endpoint.X) / ELEMENT_B_LENGTH;

        if (acos_var > 1)
        {
            acos_var = 1;
        }

        double partialAngle = (acos(acos_var) * 180) / M_PI;

        if (future_joint_2.Y > future_endpoint.Y)
        {
            partialAngle *= -1; // if joint 2 is higher than the end effector, partial angle must be negative
        }

        futureAngles.Joint_angle_2 = fmod(partialAngle + futureAngles.Joint_angle_1 + 360, 360);

        if ((current_angles.Joint_angle_2 < 90) &&
            (futureAngles.Joint_angle_2 > 270) && (current_angles.Joint_angle_2 <= 360))
        {
            /* a rotation through zero was detected, prevent this */
            futureAngles.Joint_angle_2 = 0;
        }

        if ((current_angles.Joint_angle_2 > 270) && (current_angles.Joint_angle_2 <= 360) &&
             (futureAngles.Joint_angle_2 < 90))
        {
            /* a rotation through zero was detected, prevent this */
            futureAngles.Joint_angle_2 = 360;
        }

        futureAngles.Joint_angle_3 = fmod((future_endpoint.Phi - (futureAngles.Joint_angle_2 - futureAngles.Joint_angle_1)) + 360, 360);

        /* check boundaries of JointAngle3 */
#if 0
        if (futureAngles.Joint_angle_3 < 25)
        {
            futureAngles.Joint_angle_3 = 25;
        }
        else if (futureAngles.Joint_angle_3 > 340)
        {
            futureAngles.Joint_angle_3 = 340;
        }
#endif

        return futureAngles;
    }

    End_effector_coordinates ModArm_kinematic::Get_end_effector_coordinates(ModArm_angles angles)
    {
        End_effector_coordinates myEndEffectorCoordinates = End_effector_coordinates();

        End_effector_coordinates future_joint_2 = End_effector_coordinates();

        future_joint_2.X = ELEMENT_A_LENGTH * cos((angles.Joint_angle_1 * M_PI) / 180);
        future_joint_2.Y = ELEMENT_A_LENGTH * sin((angles.Joint_angle_1 * M_PI) / 180);

        myEndEffectorCoordinates.X = future_joint_2.X - (ELEMENT_B_LENGTH * cos(((angles.Joint_angle_1 - angles.Joint_angle_2) * M_PI) / 180));
        myEndEffectorCoordinates.Y = future_joint_2.Y - (ELEMENT_B_LENGTH * sin(((angles.Joint_angle_1 - angles.Joint_angle_2) * M_PI) / 180));

        myEndEffectorCoordinates.Phi = fmod((angles.Joint_angle_2 - angles.Joint_angle_1) + angles.Joint_angle_3 + 360, 360);

        return myEndEffectorCoordinates;
    }

    End_effector_coordinates ModArm_kinematic::Get_joint_2_coordinates(ModArm_angles angles)
    {
        End_effector_coordinates myEndEffectorCoordinates = End_effector_coordinates();

        myEndEffectorCoordinates.X = ELEMENT_A_LENGTH * cos((angles.Joint_angle_1 * M_PI) / 180);
        myEndEffectorCoordinates.Y = ELEMENT_A_LENGTH * sin((angles.Joint_angle_1 * M_PI) / 180);

        return myEndEffectorCoordinates;
    }
}

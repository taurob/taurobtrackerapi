/**************************************************************************//**
 *
 * @file taurob_flipper_node.h
 * @author taurob GmbH
 * @brief A node to control the flipper of the taurob tracker.
 *
 *  Copyright (c) 2017 taurob GmbH. All rights reserved.
 *  Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ******************************************************************************/

#include "libtaurob_flipper/libtaurob_flipper.h"
#include <boost/function.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#define JOINT_NAME "flipper_front"

template <typename T>
class FlipperSubscriber
{
public:
    typedef boost::function<void(Flipper&, const typename T::ConstPtr&)> callback_type;

    FlipperSubscriber(
        Flipper& flipper,
        ros::NodeHandle& node_handle,
        std::string topic_name,
        callback_type callback_function)
        : flipper_(flipper)
        , callback_function_(callback_function)
        , subscriber_(node_handle.subscribe(topic_name, 1, &FlipperSubscriber::callback, this))
    {
    }

private:
    void callback(const typename T::ConstPtr& msg)
    {
        if (callback_function_) {
            callback_function_(flipper_, msg);
        }
    }

    Flipper& flipper_;
    callback_type callback_function_;
    ros::Subscriber subscriber_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "flipper_hw_interface");
    ros::NodeHandle pnh("~");

    std::string host;
    if (!pnh.getParam("ip_address", host)) {
        ROS_FATAL("'ip_address' parameter not found");
        return 1;
    }

    int port;
    if (!pnh.getParam("port", port)) {
        ROS_FATAL("'port' parameter not found");
        return 1;
    }

    if (port < 1 || 65535 < port) {
        ROS_FATAL("'port' parameter outside of valid range");
        return 1;
    }

    ROS_INFO_STREAM(
        "taurob_flipper_ros_control_node, remote address: '"
        << host << ':' << port << '\'');

    Flipper flipper(host, port);

    ros::NodeHandle nh;

    FlipperSubscriber<std_msgs::Bool> watchdog_feeder(flipper, nh, "watchdog_feed",
        [](Flipper& flipper, const std_msgs::Bool::ConstPtr& msg)
        {
            if (msg->data)
            {
                flipper.Feed_watchdog();
            }
        });

    FlipperSubscriber<sensor_msgs::JointState> angle_feeder(flipper, nh, "jointstate_cmd",
        [](Flipper& flipper, const sensor_msgs::JointState::ConstPtr& msg)
        {
            for (size_t i = 0; i < msg->name.size(); i++)
            {
                if (msg->name[i] != JOINT_NAME) {
                    continue;
                }

                FlipperCommand fc;
                fc.angle = msg->position[i];
                flipper.Set(fc);
            }
        });

    ros::Publisher pub_jointstate = nh.advertise<sensor_msgs::JointState>("jointstate_status", 1);

    flipper.Run();

    ros::Duration interval(0.1);
    for (;;) {
        if (!ros::ok())
            break;

        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        js.name.push_back(JOINT_NAME);

        FlipperStatus flipper_status = flipper.Get();

        js.position.push_back(flipper_status.angle);
        js.velocity.push_back(flipper_status.speed);
        js.effort.push_back(0);
        pub_jointstate.publish(js);

        ros::spinOnce();
        interval.sleep();
    }
}

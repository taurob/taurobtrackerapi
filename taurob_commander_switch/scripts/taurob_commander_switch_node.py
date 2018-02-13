#!/usr/bin/env python

import rospy
import select
import socket
import sys
import threading

import base_frames
import flipper_frames

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import taurob_commander_switch.srv

# Port numbers where we wait for U-ECU command frames.
BASE_PORT = 8080
FLIPPER_PORT = 8090


# Data used to fake the base U-ECU.
base_ecu_lock = threading.Lock()
base_ecu = {
    'endpoint': None,
    'socket': None,
    'seqno': None
}

# Data used to fake the flipper U-ECU
flipper_ecu_lock = threading.Lock()
flipper_ecu = {
    'endpoint': None,
    'socket': None,
    'seqno': None
}


class Switch:
    '''Class that either forwards ROS or Commander set-values
    to the actual driver of the robot.'''

    def __init__(self):
        # We use internal locking because the methods of this
        # class are both called from the main thread and rospy.
        self._lock = threading.Lock()

        # If true, we forward the ROS messages,
        # otherwise the commands from Commander.
        self._forwarding_ros_messages = True

        # The topics where the rest of the ROS stack publishes its commands.
        rospy.Subscriber("cmd_vel", Twist, self._cmd_val_callback)
        rospy.Subscriber("jointstate_cmd", JointState, self._jointstate_cmd_callback)

        # The remapped input topics of the robot drivers.
        self._pub_cmd_vel = rospy.Publisher('cmd_vel_switched', Twist, queue_size=2)
        self._pub_jointstate_cmd = rospy.Publisher('jointstate_cmd_switched', JointState, queue_size=2)

    def _cmd_val_callback(self, msg):
        with self._lock:
            if not self._forwarding_ros_messages:
                # Currently we forward Commander messages,
                # drop ROS message.
                return

            self._pub_cmd_vel.publish(msg)

    def _jointstate_cmd_callback(self, msg):
        with self._lock:
            if not self._forwarding_ros_messages:
                # Currently we forward Commander messages,
                # drop ROS message.
                return

            self._pub_jointstate_cmd.publish(msg)

    def switch(self, new_input):
        '''Changes the input that is forwarded.'''

        with self._lock:
            if new_input == 'commander':
                self._forwarding_ros_messages = False
            elif new_input == 'ros':
                self._forwarding_ros_messages = True

    def commander_base_values(self, linear, angular):
        with self._lock:
            if self._forwarding_ros_messages:
                # Currently we forward ROS messages,
                # drop Commander message.
                return

            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self._pub_cmd_vel.publish(msg)
     

    def commander_flipper_value(self, angle):
        with self._lock:
            if self._forwarding_ros_messages:
                # Currently we forward ROS messages,
                # drop Commander message.
                return

            msg = JointState()
            msg.header = Header()
            msg.name = ['flipper_front']
            msg.position = [angle]
            self._pub_jointstate_cmd.publish(msg)


class Service:
    '''Provides a ROS service to switch between ROS and Commander.'''

    def __init__(self, switch, possible_inputs):
        self._switch = switch
        self._inputs = possible_inputs

        self._current_input = self._inputs[0]

        self._state_service = rospy.Service(
        			'commander_switch_state',
        			taurob_commander_switch.srv.State,
        			self.state_service_callback)
        self._change_service = rospy.Service(
        			'commander_switch_change',
        			taurob_commander_switch.srv.Change,
        			self.change_service_callback)

    def change_input(self, new_input):
        if not new_input in self._inputs:
            sys.stderr.write('invalid input "{}", ignoring\n'.format(new_input))
            return

        if self._current_input == new_input:
            # Nothing to do.
            return

        self._current_input = new_input
        sys.stdout.write('input set to "{}"\n'.format(self._current_input))
        self._switch.switch(self._current_input)


    def state_service_callback(self, req):
        return taurob_commander_switch.srv.StateResponse(self._current_input)

    def change_service_callback(self, req):
        self.change_input(req.str)
        return taurob_commander_switch.srv.ChangeResponse()


def joint_states_callback(msg):
    '''Sends a flipper status frame back to Commander.'''

    try:
        i = msg.name.index('flipper_front')
    except:
        # The flipper is not included in the message.
        return

    with flipper_ecu_lock:
        if flipper_ecu['socket'] is None or \
           flipper_ecu['endpoint'] is None or \
           flipper_ecu['seqno'] is None:
            # Not yet ready to send something back to Commander.
            return
 
        data = {
            'seqno': flipper_ecu['seqno'],
            'angle': msg.position[i]
        }
        frame = flipper_frames.pack_status(data)
        flipper_ecu['socket'].sendto(frame, flipper_ecu['endpoint'])


if __name__ == '__main__':
    try:
        rospy.init_node('taurob_commander_switch')

        switch = Switch()
        service = Service(switch, ('ros', 'commander'))

        sock_base = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_base.bind(('0.0.0.0', BASE_PORT))
        with base_ecu_lock:
            base_ecu['socket'] = sock_base

        sock_flipper = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_flipper.bind(('0.0.0.0', FLIPPER_PORT))
        with flipper_ecu_lock:
            flipper_ecu['socket'] = sock_flipper


        rospy.Subscriber("joint_states", JointState, joint_states_callback)

        while True:
            r, w, e = select.select([sock_base, sock_flipper], [], [])

            if sock_base in r:
                data, addr = sock_base.recvfrom(1024)

                try:
                    cmd = base_frames.unpack_command(data)
                except Exception as e:
                    sys.stderr.write('error unpacking base frame: {}\n'.format(e))
                    continue

                with base_ecu_lock:
                    base_ecu['endpoint'] = addr
                    base_ecu['seqno'] = cmd['seqno']

                switch.commander_base_values(cmd['linear'], cmd['angular'])

            if sock_flipper in r:
                data, addr = sock_flipper.recvfrom(1024)

                try:
                    cmd = flipper_frames.unpack_command(data)
                except Exception as e:
                    sys.stderr.write('error unpacking flipper frame: {}\n'.format(e))
                    continue

                with flipper_ecu_lock:
                    flipper_ecu['endpoint'] = addr
                    flipper_ecu['seqno'] = cmd['seqno']

                if cmd['active']:
                    switch.commander_flipper_value(cmd['angle'])

    except rospy.ROSInterruptException:
        pass

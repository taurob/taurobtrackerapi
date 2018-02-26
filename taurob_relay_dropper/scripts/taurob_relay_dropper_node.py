#!/usr/bin/env python

from PyCRC.CRC16 import CRC16

import rospy
import threading
import socket
import struct

import std_srvs.srv


RELAY_DROPPER_ENDPOINT = ('10.0.0.41', 9000)


# Data used to communicate between threads.
triggered_lock = threading.Lock()
triggered = False


def service_callback(req):
    print('triggered')

    global triggered_lock
    global triggered

    with triggered_lock:
        triggered = True

    return std_srvs.srv.EmptyResponse()


def pack_command_frame(seq_no, active):
    bitfield = 0x40 if active else 0x00

    fmt = '!BBBBHH'
    data = struct.pack(fmt, 0, 1, seq_no, bitfield, 0, 0)

    crc = CRC16(modbus_flag=True).calculate(data)
    data += struct.pack('!H', crc)

    return data


if __name__ == '__main__':
    try:
        rospy.init_node('taurob_relay_dropper_node')

        service = rospy.Service(
       	    'drop_relay',
            std_srvs.srv.Empty,
            service_callback)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        seq_no = 0

        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            with triggered_lock:
                active_frame = triggered
                triggered = False

            if active_frame:
                print('sending active command frame')

            data = pack_command_frame(seq_no, active_frame)
            seq_no = (seq_no + 1) % 256

            sock.sendto(data, RELAY_DROPPER_ENDPOINT)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass

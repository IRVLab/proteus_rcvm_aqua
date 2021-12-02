#! /usr/bin/python3

import sys, math, threading, signal
from time import sleep
from math import pi

from aquacore.msg import AutopilotModes
from rcvm_autopilot_client import RCVMPilotClient

import rospy
from tf.transformations import euler_from_quaternion
import roslaunch
import rosnode 

from timeout import Timeout

from proteus import Kineme


rospy.init_node('aqua_rcvm_server', argv=None, anonyomous=True)

params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST
pc = RCVMPilotClient(params)


if __name__ == '__main__':
    rospy.loginfo('Initializing the Aqua8 RCVM server')
    # Load proteus language definition file
    

    # Select kineme language definition file
    # Process kineme definition file into kineme objects
    # Setup topic listener/service calls

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

else:
    pass
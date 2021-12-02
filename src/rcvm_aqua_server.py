#! /usr/bin/python3

import sys, math, threading, signal
from time import sleep
from math import pi

from aquacore.msg import AutopilotModes
from rcvm_autopilot_client import RCVMPilotClient

import rospy
from rosnode import get_node_names
from tf.transformations import euler_from_quaternion
import roslaunch
import rosnode 

from timeout import Timeout

import xml.etree.ElementTree as ET
from proteus.kineme import Kineme, KNode
from proteus.srv import SymbolTrigger, SymbolDirectional

# params = {}
# params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST
# pc = RCVMPilotClient(params)

symbols = None
kinemes = None

def execute_kineme(req, id, call_type):
    return True

if __name__ == '__main__':
    rospy.init_node('aqua_rcvm_server', argv=None, anonymous=True)
    rospy.loginfo('Initializing the Aqua8 RCVM server')

    #Check if PROTEUS language server is up
    rospy.loginfo('Checking PROTEUS language server...')
    lang_server_active = False
    nodes = get_node_names()
    rospy.logdebug(nodes)
    for n in nodes:
        if n.split('/')[-1] == 'proteus_language_server':
            lang_server_active = True
            break
    if not lang_server_active:
        rospy.logerr("This RCVM implementation requires the PROTEUS language server to be active.")
        sys.exit(1)
    else:
        rospy.loginfo('PROTEUS language server OK!')

    # Find kineme language definition file
    rospy.loginfo("Loading RCVM vector information...")
    rcvm_info = rospy.get_param('vectors/out/RCVM')
    rcvm_def_file = rcvm_info['definition_file']

    # Find symbol definitions
    rospy.loginfo("Loading symbol information...")
    symbols = rospy.get_param('symbols/out')
    rospy.loginfo(symbols)
    
    # Process kineme definition file into kineme objects
    rospy.loginfo("Loading Kineme definitions from kineme definition file.")
    kinemes = list()

    #Load XML file
    tree = ET.parse(rcvm_def_file)
    root = tree.getroot()
    for kdef in root:
        k = Kineme()
        k.parse_from_xml(kdef)
        kinemes.append(k)

    # Check for symbol matchup and print summary.
    

    # Setup service calls
    rospy.Service('rcvm/Affirmative', SymbolTrigger, lambda req: execute_kineme(req, 'affirmative','trigger'))

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

else:
    pass
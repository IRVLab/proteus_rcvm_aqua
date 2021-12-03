#! /usr/bin/python3

import sys, math, threading, signal
from time import sleep
from math import pi

from aquacore.msg import AutopilotModes
from rcvm_autopilot_client import RCVMPilotClient

import rospy
from rosnode import get_node_names
from rospy import service
from tf.transformations import euler_from_quaternion
import roslaunch
import rosnode 

from timeout import Timeout

import xml.etree.ElementTree as ET
from proteus.kineme import Kineme, KNode
from proteus.srv import SymbolTrigger, SymbolDirectional, SymbolTarget

# params = {}
# params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST
# rpc = RCVMPilotClient(params)

rcvm_params = None
symbols = None
kinemes = None

# Farms out the execution of the kineme to the appropriate function
def service_cb(req, id, call_type):
    if call_type == 'trigger':
        return execute_trigger(req, id)
    elif call_type == 'directional':
        return execute_directional(req, id)
    elif call_type == 'target':
        return execute_target(req, id)
    else:
        return False

# Executes kinemes which are trigger called, meaning that there's no information in the service call.
def execute_trigger(req, id):
    kineme = kinemes[id]
    for knode in kineme.knodes:
        rpc.do_relative_angle_change((knode.orientation.roll, knode.orientation.pitch, knode.orientation.yaw), rpc.current_depth, knode.position.x, knode.position.y, knode.duration.seconds, threshold=rcvm_params['angle_diff_threshold'])
    
# Executes kinemes which are directional called, meaning that there's directional information in the service call.
def execute_directional(req, id):
    return False

# Executes kinemes which are target called, meaning that there is a need to connect the kineme to a target.
def execute_target(req, id):
    return False


if __name__ == '__main__':
    rospy.init_node('aqua_rcvm_server', argv=None, anonymous=True)
    rospy.loginfo('Initializing the Aqua8 RCVM server')

    rcvm_params = rospy.get_param('rcvm/', 'relative')

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
    
    # Process kineme definition file into kineme objects
    rospy.loginfo("Loading Kineme definitions from kineme definition file.")
    kinemes = dict()

    #Load XML file
    tree = ET.parse(rcvm_def_file)
    root = tree.getroot()
    for kdef in root:
        k = Kineme()
        k.parse_from_xml(kdef)
        kinemes[k.id] = k

    # Check for symbol matchup.
    for s in symbols:
        for key in kinemes:
            k = kinemes[key]
            if s == k.id:
                rospy.loginfo("Found match beteween symbol %s and kineme %s, associating data."%(s, k.id))
                rospy.logdebug("Call type: %s"%(symbols.get(s).get('call_type')))
                k.set_call_type(symbols.get(s).get('call_type'))
                break

    # Setup service calls
    for key, kineme in kinemes.items():
        service_class = None
        if kineme.call_type == 'trigger':
            service_class = SymbolTrigger
        elif kineme.call_type == 'directional':
            service_class = SymbolDirectional
        elif kineme.call_type == 'target':
            service_class = SymbolTarget
        else:
            rospy.logwarn("Unexpected kineme call type %s"%(kineme.call_type))

        rospy.Service('rcvm/'+kineme.name, service_class, lambda req: service_cb(req, key, kineme.call_type))

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

else:
    pass
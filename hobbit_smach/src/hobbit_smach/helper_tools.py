#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = 'Markus Bajones'
PKG = 'hobbit_smach'
NAME = 'helper_node'
DEBUG = False
RETRY = 10
state = True

import rospy
from hobbit_msgs.srv import ChargeCheck, ChargeCheckResponse, SetCloserState, GetCloserState
from mira_msgs.msg import BatteryState


def handle_charge_check(req):
    response = False
    rospy.loginfo("Charge check called")
    if DEBUG:
        return ChargeCheckResponse(response)
    for x in xrange(1, RETRY):
        rospy.sleep(1.0)
        try:
            msg = rospy.wait_for_message(
                '/battery_state',
                BatteryState,
                timeout=5
                )
            if msg.charging:
                response = True
                break
        except rospy.ROSException as e:
                response = False
    return ChargeCheckResponse(response)

def set_closer_state(req):
    global state
    response = False
    rospy.loginfo("Set Come closer check called")
    state = req.state
    rospy.loginfo("Set state to "+str(state))
    return True

def set_closer_state(req):
    global state
    response = True
    rospy.loginfo("Get Come closer check called")
    rospy.loginfo("Get state is "+str(state))
    return state

def charge_check_server():
    rospy.init_node('hobbit_helper_node')
    s = rospy.Service('/hobbit/charge_check', ChargeCheck, handle_charge_check)
    s1 = rospy.Service('/came_closer/set_closer_state', SetCloserState, set_closer_state)
    s2 = rospy.Service('/came_closer/set_closer_state', GetCloserState, get_closer_state)

    rospy.loginfo('Node started')
    rospy.spin()
    rospy.loginfo('Node stopped')

if __name__ == '__main__':
    charge_check_server()

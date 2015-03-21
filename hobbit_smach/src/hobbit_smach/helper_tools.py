#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = 'Markus Bajones'
PKG = 'hobbit_smach'
NAME = 'helper_node'
DEBUG = False
RETRY = 10
state = False
dock_state = False
move_state = False
arm_state = False

import rospy
from hobbit_msgs.srv import SetCloserState, GetCloserState, SetDockState, GetDockState, SetMoveState, GetMoveState
from hobbit_msgs.srv import GetArmState, SetArmState
from mira_msgs.msg import BatteryState

def set_closer_state(req):
    global state
    response = False
    rospy.loginfo("Set Come closer check called")
    state = req.state
    rospy.loginfo("Set state to "+str(state))
    return True

def get_closer_state(req):
    global state
    response = True
    rospy.loginfo("Get Come closer check called")
    rospy.loginfo("Get state is "+str(state))
    return state

def set_dock_state(req):
    global dock_state
    response = False
    rospy.loginfo("Set dock check called")
    dock_state = req.state
    rospy.loginfo("Set state to "+str(dock_state))
    return True

def get_dock_state(req):
    global dock_state
    response = True
    rospy.loginfo("Get dock check called")
    rospy.loginfo("Get state is "+str(dock_state))
    return dock_state

def set_move_state(req):
    global move_state
    response = False
    rospy.loginfo("Set move check called")
    move_state = req.state
    rospy.loginfo("Set state to "+str(move_state))
    return True

def get_move_state(req):
    global move_state
    response = True
    rospy.loginfo("Get move check called")
    rospy.loginfo("Get state is "+str(move_state))
    return move_state

def set_arm_state(req):
    global arm_state
    response = False
    rospy.loginfo("Set arm check called")
    arm_state = req.state
    rospy.loginfo("Set state to "+str(arm_state))
    return True

def get_arm_state(req):
    global arm_state
    response = True
    rospy.loginfo("Get arm check called")
    rospy.loginfo("Get state is "+str(arm_state))
    return arm_state

def set_dock_state_true(msg):
    global dock_state
    if msg.charging:
        #rospy.loginfo("Set docked to TRUE")
        dock_state = True
     

def charge_check_server():
    rospy.init_node('hobbit_helper_node')
    s1 = rospy.Service('/came_closer/set_closer_state', SetCloserState, set_closer_state)
    s2 = rospy.Service('/came_closer/get_closer_state', GetCloserState, get_closer_state)
    s3 = rospy.Service('/docking/set_dock_state', SetDockState, set_dock_state)
    s4 = rospy.Service('/docking/get_dock_state', GetDockState, get_dock_state)
    s5 = rospy.Service('/moving/set_move_state', SetMoveState, set_move_state)
    s6 = rospy.Service('/moving/get_move_state', GetMoveState, get_move_state)
    s7 = rospy.Service('/arm/set_move_state', SetArmState, set_arm_state)
    s8 = rospy.Service('/arm/get_move_state', GetArmState, get_arm_state)
    rospy.Subscriber('battery_state', BatteryState, set_dock_state_true)

    rospy.loginfo('Node started')
    rospy.spin()
    rospy.loginfo('Node stopped')

if __name__ == '__main__':
    charge_check_server()

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
user_away = False

import rospy
from hobbit_msgs.srv import SetCloserState, GetCloserState, SetDockState, GetDockState, SetMoveState, GetMoveState, ChargeCheck, ChargeCheckResponse
from hobbit_msgs.srv import GetArmState, SetArmState, GetAwayState, SetAwayState
from hobbit_msgs.msg import Event, Command
from mira_msgs.msg import BatteryState
import hobbit_smach.arm_move_import as arm_move

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

def set_away_state(req):
    global user_away
    rospy.loginfo("Set user away state called with request: "+str(req.state))
    user_away = req.state
    rospy.loginfo("Set state to "+str(user_away))
    return True

def get_away_state(req):
    global user_away
    rospy.loginfo("Get user away state called")
    rospy.loginfo("Get state is "+str(user_away))
    return user_away

def set_dock_state_true(msg):
    global dock_state
    if msg.charging:
        #rospy.loginfo("Set docked to TRUE")
        dock_state = True

def events_to_commands(msg):
    rospy.loginfo("i heard: "+str(msg.event))
    pub = rospy.Publisher('/Command', Command, queue_size=10)
    command = Command()
    if msg.event.lower() == 'A_learn'.lower():
        command.command = 'C_LEARN'
    elif msg.event.lower() == 'A_follow'.lower():
        command.command = 'C_FOLLOW'
    elif msg.event.lower() == 'A_sleep'.lower():
        command.command = 'C_SLEEP6'
    elif msg.event.lower() in ['A_stop'.lower(), 'A_cancel'.lower()]:
        command.command = 'C_STOP'
    elif msg.event.lower() == 'A_help'.lower():
        command.command = 'C_HELP'
    elif msg.event.lower() in ['A_pickup'.lower(), 'A_cleanup'.lower()]:
        command.command = 'C_PICKUP'
    elif msg.event.lower() == 'A_welldone'.lower():
        command.command = 'C_REWARD'
    elif msg.event.lower() == 'A_surprise'.lower():
        command.command = 'C_SURPRISE'
    elif msg.event.lower() == 'A_recharge'.lower():
        command.command = 'C_RECHARGE'
    else:
        rospy.loginfo('Event ignored')
        return
    rospy.loginfo("will publish"+str(command))
    pub.publish(command)


def charge_check_server():
    rospy.init_node('hobbit_helper_node')
    s = rospy.Service('/hobbit/charge_check', ChargeCheck, handle_charge_check)
    s1 = rospy.Service('/came_closer/set_closer_state', SetCloserState, set_closer_state)
    s2 = rospy.Service('/came_closer/get_closer_state', GetCloserState, get_closer_state)
    s3 = rospy.Service('/docking/set_dock_state', SetDockState, set_dock_state)
    s4 = rospy.Service('/docking/get_dock_state', GetDockState, get_dock_state)
    s5 = rospy.Service('/moving/set_move_state', SetMoveState, set_move_state)
    s6 = rospy.Service('/moving/get_move_state', GetMoveState, get_move_state)
    s7 = rospy.Service('/arm/set_move_state', SetArmState, set_arm_state)
    s8 = rospy.Service('/arm/get_move_state', GetArmState, get_arm_state)
    s9 = rospy.Service('/user/set_away_state', SetAwayState, set_away_state)
    s10 = rospy.Service('/user/get_away_state', GetAwayState, get_away_state)
    rospy.Subscriber('battery_state', BatteryState, set_dock_state_true)
    rospy.Subscriber('Event', Event, events_to_commands)

    rospy.loginfo('Node started')
    rospy.spin()
    rospy.loginfo('Node stopped')

if __name__ == '__main__':
    charge_check_server()

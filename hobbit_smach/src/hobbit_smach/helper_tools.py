#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = 'Markus Bajones'
PKG = 'hobbit_smach'
NAME = 'helper_node'
DEBUG = False

import rospy
from hobbit_msgs.srv import ChargeCheck, ChargeCheckResponse
from mira_msgs.msg import BatteryState


def handle_charge_check(req):
    if DEBUG:
        print("Charge check called")
        return ChargeCheckResponse(True)
    try:
        msg = rospy.wait_for_message(
            '/battery_state',
            BatteryState,
            timeout=5
            )
        if msg.charging:
            return ChargeCheckResponse(True)
        else:
            return ChargeCheckResponse(False)
    except rospy.ROSException as e:
            return ChargeCheckResponse(False)


def charge_check_server():
    rospy.init_node('hobbit_helper_node')
    s = rospy.Service('/hobbit/charge_check', ChargeCheck, handle_charge_check)
    rospy.loginfo('Node started')
    rospy.spin()
    rospy.loginfo('Node stopped')

if __name__ == '__main__':
    charge_check_server()

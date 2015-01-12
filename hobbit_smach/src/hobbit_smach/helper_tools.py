#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = 'Markus Bajones'
PKG = 'hobbit_smach'
NAME = 'helper_node'
DEBUG = False
RETRY = 10

import rospy
from hobbit_msgs.srv import ChargeCheck, ChargeCheckResponse
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



def charge_check_server():
    rospy.init_node('hobbit_helper_node')
    s = rospy.Service('/hobbit/charge_check', ChargeCheck, handle_charge_check)
    rospy.loginfo('Node started')
    rospy.spin()
    rospy.loginfo('Node stopped')

if __name__ == '__main__':
    charge_check_server()

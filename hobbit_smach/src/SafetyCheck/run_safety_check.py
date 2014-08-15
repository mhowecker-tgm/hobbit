#!/usr/bin/env python

NAME = 'run_safety_check'
PKG = 'hobbit_smach'
import roslib
roslib.load_manifest(PKG)
import rospy
import actionlib

from hobbit_msgs.msg import GeneralHobbitAction, GeneralHobbitGoal
from std_msgs.msg import String


def start_action():
    print "start client"
    client = actionlib.SimpleActionClient(
        'safety_check', GeneralHobbitAction)
    client.wait_for_server()
    print('connected to server')
    par = []
    goal = GeneralHobbitGoal(command=String('safety_check'),
                             previous_state=String('call_hobbit'),
                             parameters=par)
    print('safety_check')
    client.send_goal(goal)
    print('waiting for result')
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
        try:
            rospy.init_node(NAME)
            print('node %s started' % NAME)
            result = start_action()
            print('Result: ', result)
        except rospy.ROSInterruptException:
            print('program interrupted before completion')

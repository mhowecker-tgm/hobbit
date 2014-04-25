#!/usr/bin/env python

NAME = 'run_learn_object'
PKG = 'hobbit_smach'
import roslib
roslib.load_manifest(PKG)
import rospy
import actionlib

#import hobbit_msgs.msg
from hobbit_msgs.msg import GeneralHobbitAction, GeneralHobbitGoal
from std_msgs.msg import String


def start_action():
    print "start client"
    client = actionlib.SimpleActionClient(
        'learn_object', GeneralHobbitAction)
    client.wait_for_server()
    print('connected to server')
    par = []
    goal = GeneralHobbitGoal(command=String('learn_object'),
                             previous_state=String('call_hobbit'),
                             parameters=par)
    print(goal)
    print(type(goal))
    print('learn_object')
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

#!/usr/bin/env python

NAME = 'run_goto'
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
        'goto', GeneralHobbitAction)
    client.wait_for_server()
    print('connected to server')
    par = []
    par.append(String('ShortCorridor'))
    par.append(String('table'))
    goal = GeneralHobbitGoal(command=String('goto'),
                             previous_state=String('call_hobbit'),
                             parameters=par)
    print(goal)
    print(type(goal))
    print('goto')
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

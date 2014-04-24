#!/usr/bin/env python

NAME = 'run_emergency'
import roslib
roslib.load_manifest('hobbit_smach')
import rospy
import actionlib

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction, GeneralHobbitGoal


def start_action():
    print "start client"
    client = actionlib.SimpleActionClient(
        'emergency_patrol', GeneralHobbitAction)
    client.wait_for_server()
    print "connected to server"
    par = []
    par.append(String('user_not_responding'))
    goal = GeneralHobbitGoal(
        command=String('Emergency'),
        parameters=par)
    print goal
    print "send goal"
    client.send_goal(goal)
    print "waiting for result"
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
        try:
            rospy.init_node(NAME)
            result = start_action()
            print('node %s started' % NAME)
            print ('Result:', result)
        except rospy.ROSInterruptException:
            print('program interrupted before completion')

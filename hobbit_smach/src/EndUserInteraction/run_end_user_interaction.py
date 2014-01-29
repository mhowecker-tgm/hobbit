#!/usr/bin/env python

import roslib; roslib.load_manifest('hobbit_msgs')
import rospy
import actionlib

import hobbit_msgs.msg
from std_msgs.msg import String

NAME = 'run_end_user_interaction'

def start_action():
    print "start client"
    client = actionlib.SimpleActionClient('end_user_interaction', hobbit_msgs.msg.EndUserInteractionAction)
    client.wait_for_server()
    print "connected to server"
    goal = hobbit_msgs.msg.EndUserInteractionGoal(
            command=String('end_user_interaction'))
    print goal
    print "send goal"
    client.send_goal(goal)
    print "waiting for result"
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
        try:
            rospy.init_node('NAME')
            print 'node',NAME,'started'
            result = start_action()
            print "Result: ", result
        except rospy.ROSInterruptException:
             print "program interrupted before completion"

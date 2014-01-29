#!/usr/bin/env python

PKG = 'hobbit_msgs'
NAME = 'runBringObject'

import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib

import hobbit_msgs.msg
from std_msgs.msg import String

def start_action():
    print "start client"
    client = actionlib.SimpleActionClient('bring_object', hobbit_msgs.msg.BringObjectAction)
    client.wait_for_server()
    print "connected to server"
    goal = hobbit_msgs.msg.BringObjectGoal(
            object_name=String('mug'))
    print goal
    print "send goal"
    client.send_goal(goal)
    print "waiting for result"
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
        try:
            rospy.init_node(NAME)
            print 'node '+NAME+ ' started'
            result = start_action()
            print "Result: ", result
        except rospy.ROSInterruptException:
             print "program interrupted before completion"

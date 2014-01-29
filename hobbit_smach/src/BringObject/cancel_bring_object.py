#!/usr/bin/env python

import roslib; roslib.load_manifest('hobbit_msgs')
import rospy
import actionlib

import hobbit_msgs.msg

NAME = 'BringObject'

def start_action():
    print "start client"
    client = actionlib.SimpleActionClient('bring_object', hobbit_msgs.msg.BringObjectAction)
    client.wait_for_server()
    print "connected to server"
    print 'sending cancel'
    client.cancel_all_goals()

    return

if __name__ == '__main__':
        try:
            rospy.init_node('NAME')
            print 'node',NAME,'started'
            start_action()
        except rospy.ROSInterruptException:
             print "program interrupted before completion"

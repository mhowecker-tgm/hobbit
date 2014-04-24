#!/usr/bin/env python

import roslib; roslib.load_manifest('hobbit_smach')
import rospy
import actionlib

import hobbit_msgs.msg

NAME = 'cancel_end_user_interaction'

def start_action():
    print "start client"
    client = actionlib.SimpleActionClient('end_user_interaction', hobbit_msgs.msg..EndUserInteractionAction)
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

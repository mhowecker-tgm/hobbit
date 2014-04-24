#!/usr/bin/env python

NAME = 'cancel_emergency'

import roslib
roslib.load_manifest('hobbit_smach')
import rospy
import actionlib
from hobbit_msgs.msg import GeneralHobbitAction


def start_action():
    print "start client"
    client = actionlib.SimpleActionClient('emergency',
                                          GeneralHobbitAction)
    client.wait_for_server()
    print "connected to server"
    print 'sending cancel'
    client.cancel_all_goals()

    return

if __name__ == '__main__':
        try:
            rospy.init_node('NAME')
            print('node %s started' % NAME)
            start_action()
        except rospy.ROSInterruptException:
            print ('program interrupted before completion')

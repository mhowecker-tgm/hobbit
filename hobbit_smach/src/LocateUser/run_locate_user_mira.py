#!/usr/bin/env python

import roslib; roslib.load_manifest('hobbit_smach')
import rospy
import actionlib

from hobbit_msgs.msg import GeneralHobbitAction, GeneralHobbitActionGoal
from std_msgs.msg import String

def start_action():
    print "start client"
    client = actionlib.SimpleActionClient('locate_user', GeneralHobbitAction)
    client.wait_for_server()
    print "connected to server"
    goal = hobbit_msgs.msg.GeneralHobbitActionGoal(
            command=String('locateUser'))
    print goal
    print "send goal"
    client.send_goal(goal)
    print "waiting for result"
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
        try:
            rospy.init_node('runLocate')
            print "node runLocate started"
            result = start_action()
            print "Result: ", result
        except rospy.ROSInterruptException:
             print "program interrupted before completion"

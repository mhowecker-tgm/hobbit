#!/usr/bin/python
'''
Created on 10.03.2015

this node is for shutting down the PLC

author: david fischinger
'''

PKG = 'hobbit_smach'

import roslib
roslib.load_manifest(PKG)
import rospy
import ArmControllerFunctions
from std_msgs.msg import String, Bool
import actionlib
import hobbit_msgs.msg
import ast


class ArmActionClient():

    last_feedback = None

    def __init__(self):
        #self.arm_client_pub = rospy.Publisher("/arm/commands", String)
        self.client = actionlib.SimpleActionClient("arm_action_server", hobbit_msgs.msg.ArmServerAction)   #"arm_action_server" has to be nodename of ArmActionServer(.py)
 


    # use this method with a String cmd to send goal to the ArmActionServer and return its result value
    def arm_action_client(self, cmd):

        # Waits until the action server has started up and started
        # listening for goals.
        print "ArmActionClient: wait_for_server()"
        self.client.wait_for_server()
        print "server found!"

        # Creates a goal to send to the action server.
        goal = hobbit_msgs.msg.ArmServerGoal(command=cmd)

        # Sends the goal to the action server.
        self.client.send_goal(goal, feedback_cb=self.feedback_cb)

        print "wait for result"
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
	

        # Prints out the result of executing the action
        returnval = self.client.get_result()  # 
        print "cmd: ", cmd
        #print "ArmActionClient.py: class ArmActionClient: function arm_action_client (=> sends/receives actionlib data from/to server): returnval.result.data: ", returnval.result.data
        return returnval
        
    def feedback_cb(self, feedback):
        print "feedback_cb executed!"
        #print "feedback type: ", (type) (feedback)
        self.last_feedback = feedback
        print "==========> feedback: ", self.last_feedback 


    #_____________________Command_____________________# (not used yet)
    

    def SetShutdown(self):
        result = self.arm_action_client(String('SetShutdown'))
        return result.result.data
    

if __name__ == '__main__':
   
    #   === SHUT DOWN PLC  ===

    # Node name
    rospy.init_node('arm_action_client_shut_down_plc')
    print "node arm_action_client_shut_down_plc started"
    arm_client = ArmActionClient()
    cmd = String ("SetShutdown")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res
    

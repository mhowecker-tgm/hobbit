#!/usr/bin/python
'''
Created on 28.11.2014

author: david fischinger
'''

PKG = 'arm_simulation'

import roslib
roslib.load_manifest(PKG)
import rospy
from std_msgs.msg import String, Bool
import actionlib
import hobbit_msgs.msg


class GraspTrajectoryActionClient():

    last_feedback = None

    def __init__(self):
        #self.arm_client_pub = rospy.Publisher("/arm/commands", String)
        self.client = actionlib.SimpleActionClient("grasp_trajectory_action_server", hobbit_msgs.msg.GraspTrajectoryServerAction)   #"grasp_trajectory_action_server" has to be nodename of GraspTrajectoryActionServer(.py)
 


    # use this method with a String cmd to send goal to the GraspTrajectoryActionServer and return its result value
    def grasp_trajectory_action_client(self, cmd):

        # Waits until the action server has started up and started
        # listening for goals.
        print "GraspTrajectoryActionClient: wait_for_server()"
        self.client.wait_for_server()
        print "server found!"

        #print "ArmActionClient: create goal"
        # Creates a goal to send to the action server.
        goal = hobbit_msgs.msg.GraspTrajectoryServerGoal(command=cmd)

        #print "GraspTrajectoryActionClient: send goal"
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




if __name__ == '__main__':
   
    #   === T E S T  ===

    # Node name
    rospy.init_node('grasp_trajectory_action_client')
    print "node grasp_trajectory_action_client test started"
    grasp_trajectory_client = GraspTrajectoryActionClient()
    #cmd = String ("0.32 -0.34 0.05 0 0 0 0 0 0") #dummy command/input  old
    cmd = String ("81 0.04 -0.45 0.127266 0.04 -0.51 0.127266 0 0 1 0.04 -0.48 0.127266 45") #input (=> = output from calc_grasppoints_svm_action_server)
    res = grasp_trajectory_client.grasp_trajectory_action_client(cmd)
    print "Result: ", res
 

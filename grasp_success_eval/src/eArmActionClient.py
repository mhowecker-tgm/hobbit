#!/usr/bin/python
'''
Created on 27.1.2015

author: Ester Martinez-Martin
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

class eArmActionClient():
    def __init__(self):
        self.client = actionlib.SimpleActionClient("arm_action_server", hobbit_msgs.msg.ArmServerAction)
 
    # use this method with a String cmd to send goal to the ArmActionServer and return its result value
    def arm_action_client(self, cmd):
        self.client.wait_for_server()
        goal = hobbit_msgs.msg.ArmServerGoal(command=cmd)
        self.client.send_goal(goal, feedback_cb=self.feedback_cb)
        self.client.wait_for_result()
        returnval = self.client.get_result() 
        return returnval
        
    def feedback_cb(self, feedback):
        self.last_feedback = feedback

    def GetGripperIsClosed(self):
        result = self.arm_action_client(String('GetGripperIsClosed'))
        return result.result.data

if __name__ == '__main__':
  try:
    pub = rospy.Publisher('gripperInfo', String, queue_size=1)
    pub2 = rospy.Publisher('checkGripper', String, queue_size=1)
    
    rospy.init_node('arm_action_client_publisher')
    arm_client = eArmActionClient()
    
    n = 0

    while True:
      cmd = String ("GetGripperIsClosed")
      res = arm_client.arm_action_client(cmd)
      pub.publish(str(res))

      pub2.publish(str("True"));

  except rospy.ROSInterruptException:
      print "Arm Action Client interrupted!"

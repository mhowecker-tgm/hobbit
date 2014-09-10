#!/usr/bin/python

#
# 	this server node subscribes to the ROS topic /arm/commands
#	and executes (sends commands to arm) the whished behavior
#       In addition this node has is use as an actionlib server for the arm
#
#	Created on 07.01.2014
#	@author: HofmannS, ROSified by D. Fischinger (17.6.2014)


PKG = 'hobbit_smach'

import roslib
roslib.load_manifest(PKG)
import rospy
from ArmControllerClientFunctions import ArmClientFunctions
from std_msgs.msg import String
import actionlib
from hobbit_msgs.msg import *




class ArmActionServerROS(object):

    # create messages that are used to publish feedback/result
    _feedback = hobbit_msgs.msg.ArmServerFeedback()
    _result   = hobbit_msgs.msg.ArmServerResult()


    def __init__(self): 

	#arm client instance (can only exist once)
        self.ArmClient = ArmClientFunctions('192.168.2.190')
	#subscriber for executing arm movements triggerd by message on topic
	self.armCommands_sub = rospy.Subscriber("/arm/commands", String, self.arm_execute)

	#action server stuff
	self._action_name = "armactionlibtopic"
	self._as = actionlib.SimpleActionServer(self._action_name, hobbit_msgs.msg.ArmServerAction, execute_cb=self.execute_cb, auto_start = False)
	self._as.start()
	print "ArmServer was started"

    #arm commands triggered by ActionServer
    def execute_cb(self, goal):

	#get command from goal
        strdata = str(goal.command)
        print "\nGRASP COMMAND received by the ArmActionServer:\n", strdata
	input = strdata.split()        
     	cmd = input[0]

	''' GET Functions '''
	if cmd == 'GetArmState':
            self._feedback.feedback = self.ArmClient.GetArmState()
	    print "ArmActionServer: feedback: ",self._feedback.feedback
	    self._result.result = True
	elif cmd == 'GetActualPosition':
	    print self.ArmClient.GetActualPosition()
	elif cmd == 'GetArmAtHomePos':
	    print self.ArmClient.GetArmAtHomePos()
	elif cmd == 'GetArmAtLearningPos':
	    print self.ArmClient.GetArmAtLearningPos() 
	elif cmd == 'GetArmAtTrayPos':
	    print self.ArmClient.GetArmAtTrayPos() 
	elif cmd == 'GetArmAtTurntablePos':
	    print self.ArmClient.GetArmAtTurntablePos() 
	elif cmd == 'GetTurntableAtCCWPos':
	    print self.ArmClient.GetTurntableAtCCWPos()
	elif cmd == 'GetTurntableAtCWPos':
	    print self.ArmClient.GetTurntableAtCWPos()
	elif cmd == 'GetArmHasError':
	    print self.ArmClient.GetArmHasError()                        
	elif cmd == 'GetArmHasStopped':
	    print self.ArmClient.GetArmHasStopped()
	elif cmd == 'GetArmInPositionArea':
	    print self.ArmClient.GetArmInPositionArea()
	elif cmd == 'GetArmInTargetPos':
	    print self.ArmClient.GetArmInTargetPos()
	elif cmd == 'GetArmIsEnabled':
	    print self.ArmClient.GetArmIsEnabled() 
	elif cmd == 'GetArmIsHomed':
	    print self.ArmClient.GetArmIsHomed()
	elif cmd == 'GetArmIsMoving':
	    print self.ArmClient.GetArmIsMoving()
	elif cmd == 'GetArmSoftLimitMax':
	    print self.ArmClient.GetArmSoftLimitMax()
	elif cmd == 'GetArmSoftLimitMin':
	    print self.ArmClient.GetArmSoftLimitMin()
	elif cmd == 'GetGripperIsClosed':
            print self.ArmClient.GetGripperIsClosed()         
	# SET Functions
	elif cmd == 'SetMoveToHomePos':
	    print self.ArmClient.SetMoveToHomePos()
	elif cmd == 'SetMoveToLearningPos':
	    print self.ArmClient.SetMoveToLearningPos()
	elif cmd == 'SetMoveToTrayPos':
	    print self.ArmClient.SetMoveToTrayPos()
	elif cmd == 'SetMoveToPreGraspFromFloorPos':
	    print self.ArmClient.SetMoveToPreGraspFromFloorPos()
	elif cmd == 'SetStoreTurntable':
	    print self.ArmClient.SetStoreTurntable()
	elif cmd == 'SetTurnTurntableCW':
	    print self.ArmClient.SetTurnTurntableCW()
	elif cmd == 'SetTurnTurntableCCW':
	    print self.ArmClient.SetTurnTurntableCCW()
	elif cmd == 'SetStartArmReference':
	    print self.ArmClient.SetStartArmReference()
	elif cmd == 'SetDisableArm':
	    print self.ArmClient.SetDisableArm()
	elif cmd == 'SetEnableArm':
	    print self.ArmClient.SetEnableArm()
	elif cmd == 'SetOpenGripper':
	    print self.ArmClient.SetOpenGripper()
	elif cmd == 'SetCloseGripper':
	    print self.ArmClient.SetCloseGripper()
	elif cmd == 'SetResetArm':
	    print self.ArmClient.SetResetArm()
	elif cmd == 'SetStopArmMove':
	    print self.ArmClient.SetStopArmMove()
	elif cmd == 'SetAbsolutePos':
	    print self.ArmClient.SetAbsolutePos(float(input[1]),float(input[2]),float(input[3]),float(input[4]),float(input[5]),float(input[6])) #(90, 0, 50, 0, 110, 0)
	elif cmd == 'SetStartMove':
	    print self.ArmClient.SetStartMove(float(input[1]))   #(10) #10 Grad/Sec
	# For Interpolation Mode    #no check of logic!!
	elif cmd == 'SetClearPosBuffer':
	    self.ArmClient.SetClearPosBuffer()
	elif cmd == 'SetPositionsForInterpolation':
	    self.ArmClient.SetPositionsForInterpolation(float(input[1]),float(input[2]),float(input[3]),float(input[4]),float(input[5]),float(input[6]))    #(90, 86, 70, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 70, 65, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 60, 60, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 50, 60, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 40, 55, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 30, 55, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 20, 50, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 0, 50, 0, 110, 0)
	elif cmd == 'SetPositionsForInterpolationReady':
	    self.ArmClient.SetPositionsForInterpolationReady()
	elif cmd == 'SetStartInterpolation':
	    self.ArmClient.SetStartInterpolation()
	elif cmd == 'help':
	    self.help()

	#publish feedback
	self._as.publish_feedback(self._feedback)
	self._as.set_succeeded(self._result)

        # helper variables
        #r = rospy.Rate(1)
        #success = True


    #arm commands triggered by topic
    def arm_execute(self, data):
        
        strdata = str(data.data)
        print "\nGRASP COMMAND received:\n", strdata
	input = strdata.split()        
     	cmd = input[0]

	''' GET Functions '''
	if cmd == 'GetArmState':
            print self.ArmClient.GetArmState()
	elif cmd == 'GetActualPosition':
	    print self.ArmClient.GetActualPosition()
	elif cmd == 'GetArmAtHomePos':
	    print self.ArmClient.GetArmAtHomePos()
	elif cmd == 'GetArmAtLearningPos':
	    print self.ArmClient.GetArmAtLearningPos() 
	elif cmd == 'GetArmAtTrayPos':
	    print self.ArmClient.GetArmAtTrayPos() 
	elif cmd == 'GetArmAtTurntablePos':
	    print self.ArmClient.GetArmAtTurntablePos() 
	elif cmd == 'GetTurntableAtCCWPos':
	    print self.ArmClient.GetTurntableAtCCWPos()
	elif cmd == 'GetTurntableAtCWPos':
	    print self.ArmClient.GetTurntableAtCWPos()
	elif cmd == 'GetArmHasError':
	    print self.ArmClient.GetArmHasError()                        
	elif cmd == 'GetArmHasStopped':
	    print self.ArmClient.GetArmHasStopped()
	elif cmd == 'GetArmInPositionArea':
	    print self.ArmClient.GetArmInPositionArea()
	elif cmd == 'GetArmInTargetPos':
	    print self.ArmClient.GetArmInTargetPos()
	elif cmd == 'GetArmIsEnabled':
	    print self.ArmClient.GetArmIsEnabled() 
	elif cmd == 'GetArmIsHomed':
	    print self.ArmClient.GetArmIsHomed()
	elif cmd == 'GetArmIsMoving':
	    print self.ArmClient.GetArmIsMoving()
	elif cmd == 'GetArmSoftLimitMax':
	    print self.ArmClient.GetArmSoftLimitMax()
	elif cmd == 'GetArmSoftLimitMin':
	    print self.ArmClient.GetArmSoftLimitMin()
	elif cmd == 'GetGripperIsClosed':
            print self.ArmClient.GetGripperIsClosed()         
	# SET Functions
	elif cmd == 'SetMoveToHomePos':
	    print self.ArmClient.SetMoveToHomePos()
	elif cmd == 'SetMoveToLearningPos':
	    print self.ArmClient.SetMoveToLearningPos()
	elif cmd == 'SetMoveToTrayPos':
	    print self.ArmClient.SetMoveToTrayPos()
	elif cmd == 'SetMoveToPreGraspFromFloorPos':
	    print self.ArmClient.SetMoveToPreGraspFromFloorPos()
	elif cmd == 'SetStoreTurntable':
	    print self.ArmClient.SetStoreTurntable()
	elif cmd == 'SetTurnTurntableCW':
	    print self.ArmClient.SetTurnTurntableCW()
	elif cmd == 'SetTurnTurntableCCW':
	    print self.ArmClient.SetTurnTurntableCCW()
	elif cmd == 'SetStartArmReference':
	    print self.ArmClient.SetStartArmReference()
	elif cmd == 'SetDisableArm':
	    print self.ArmClient.SetDisableArm()
	elif cmd == 'SetEnableArm':
	    print self.ArmClient.SetEnableArm()
	elif cmd == 'SetOpenGripper':
	    print self.ArmClient.SetOpenGripper()
	elif cmd == 'SetCloseGripper':
	    print self.ArmClient.SetCloseGripper()
	elif cmd == 'SetResetArm':
	    print self.ArmClient.SetResetArm()
	elif cmd == 'SetStopArmMove':
	    print self.ArmClient.SetStopArmMove()
	elif cmd == 'SetAbsolutePos':
	    print self.ArmClient.SetAbsolutePos(float(input[1]),float(input[2]),float(input[3]),float(input[4]),float(input[5]),float(input[6])) #(90, 0, 50, 0, 110, 0)
	elif cmd == 'SetStartMove':
	    print self.ArmClient.SetStartMove(float(input[1]))   #(10) #10 Grad/Sec
	# For Interpolation Mode    #no check of logic!!
	elif cmd == 'SetClearPosBuffer':
	    self.ArmClient.SetClearPosBuffer()
	elif cmd == 'SetPositionsForInterpolation':
	    self.ArmClient.SetPositionsForInterpolation(float(input[1]),float(input[2]),float(input[3]),float(input[4]),float(input[5]),float(input[6]))    #(90, 86, 70, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 70, 65, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 60, 60, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 50, 60, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 40, 55, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 30, 55, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 20, 50, 0, 110, 0)
	    #self.ArmClient.SetPositionsForInterpolation(90, 0, 50, 0, 110, 0)
	elif cmd == 'SetPositionsForInterpolationReady':
	    self.ArmClient.SetPositionsForInterpolationReady()
	elif cmd == 'SetStartInterpolation':
	    self.ArmClient.SetStartInterpolation()
	elif cmd == 'help':
	    self.help()



    def help(self): 
         
        print " ===   GET Functions === "
	print 'GetArmState'
	print 'GetActualPosition'
	print 'GetArmAtHomePos'
	print 'GetArmAtLearningPos'
	print 'GetArmAtTrayPos'
	print 'GetArmAtTurntablePos'
	print 'GetTurntableAtCCWPos'
	print 'GetTurntableAtCWPos'
	print 'GetArmHasError'
	print 'GetArmHasStopped'
	print 'GetArmInPositionArea'
	print 'GetArmInTargetPos'
	print 'GetArmIsEnabled'
	print 'GetArmIsHomed'
	print 'GetArmIsMoving'
	print 'GetArmSoftLimitMax'
	print 'GetArmSoftLimitMin'
	print 'GetGripperIsClosed'
        print "\n === SET Functions ===\n"
	print 'SetMoveToHomePos'
	print 'SetMoveToLearningPos'
	print 'SetMoveToTrayPos'
	print 'SetMoveToPreGraspFromFloorPos'
	print 'SetStoreTurntable'
	print 'SetTurnTurntableCW'
	print 'SetTurnTurntableCCW'
	print 'SetStartArmReference'
	print 'SetDisableArm'
	print 'SetEnableArm'
	print 'SetOpenGripper'
	print 'SetCloseGripper'
	print 'SetResetArm'
	print 'SetStopArmMove'
	print 'SetAbsolutePos p1 p2 p3 p4 p5 p6'
	print 'SetStartMove v1'
	print '\nFor Interpolation Mode:  '
	print 'SetClearPosBuffer'
	print 'SetPositionsForInterpolation p1 p2 p3 p4 p5 p6'
	print 'SetPositionsForInterpolationReady'
	print 'SetStartInterpolation'
	    

if __name__ == '__main__':
   
    # Node name
    rospy.init_node('arm_action_server_ros')
    print "ROS node arm_action_server started"
    #armservertopic = "armactionlibtopic"
    armclient = ArmActionServerROS()

    rospy.spin()

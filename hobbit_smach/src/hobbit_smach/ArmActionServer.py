#!/usr/bin/python

#
# this server node subscribes to the ROS topic /arm/commands
#    and executes (sends commands to arm) the desired behavior
#       In addition this node is used as an actionlib server for the arm
#
#    Created on 07.01.2014
#    @author: HofmannS, A. Flatscher, ROSified by D. Fischinger (17.6.2014)


PKG = 'hobbit_smach'

import roslib
roslib.load_manifest(PKG)
import rospy
from ArmControllerClientFunctions import ArmClientFunctions
from std_msgs.msg import String, Bool
import actionlib
import hobbit_msgs.msg
import time


def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")


class ArmActionServerROS(object):

    # create messages that are used to publish feedback/result
    _feedback = hobbit_msgs.msg.ArmServerFeedback()
    _result   = hobbit_msgs.msg.ArmServerResult()
    _stop_arm = False


    def __init__(self, name): 

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, hobbit_msgs.msg.ArmServerAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        print "ArmServer was started"

        #arm client instance (can only exist once)
        self.ArmClient = ArmClientFunctions('192.168.2.190')
        #subscriber for executing arm movements triggerd by message on topic
        self.armCommands_sub = rospy.Subscriber("/arm/commands", String, self.arm_execute)


    #check if Arm in target position
    def CheckTargetPos(self, feedback, checkpos, timeout):	
        if feedback[1]=='COMMAND_OK':
            for i in range(0,timeout):
                if self._stop_arm:
                    print "ArmActionServer: CheckTargetPos: stop function due to stop arm command on topic /arm/commands"
                    break #arm was actively stopped by command StopArm over topic /arm/commands
                time.sleep(0.5)
                print "loop round: ", i+1
                armInTargetPos = checkpos()
                print "armInTargetPos: ",armInTargetPos
                if armInTargetPos:
                    return Bool(True)
                    break
            return Bool(False)
        else:
            return Bool(False)

    #check if Arm in target position (result of status is negated)
    def CheckNegatedTargetPos(self, feedback, checkpos, timeout):	
        if feedback[1]=='COMMAND_OK':
            for i in range(0,timeout):
                time.sleep(0.5)
                print "loop round: ", i+1
                armInTargetPos = not checkpos()
                print "armInTargetPos: ",armInTargetPos
                if armInTargetPos:
                    return Bool(True)
                    break
            return Bool(False)
        else:
            return Bool(False)


    #arm commands triggered by ActionServer
    def execute_cb(self, goal):
        _stop_arm = False #default value if arm schould be stopped
        self._result.result = Bool(False) #default value for result
        #get command from goal
        isSetFunction = False
        strdata = str(goal.command.data)
        print "\nGRASP COMMAND received by the ArmActionServer: >> ", strdata
        input = strdata.split()        
        cmd = input[0]
        ''' GET Functions '''
        #return False/True if appropriate, otherwise True if command was executed
        if cmd == 'GetArmState':
            self._feedback.feedback.data = str(self.ArmClient.GetArmState())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            print "ArmActionServer, feedback type: ", (type) (self.ArmClient.GetArmState())
    
            self._result.result = Bool(True)
        elif cmd == 'GetActualPosition':
            self._feedback.feedback.data = str(self.ArmClient.GetActualPosition())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = Bool(True)
        elif cmd == 'GetArmAtHomePos':
            self._feedback.feedback.data = str(self.ArmClient.GetArmAtHomePos())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetArmAtLearningPos':
            self._feedback.feedback.data = str(self.ArmClient.GetArmAtLearningPos())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetArmAtTrayPos':
            self._feedback.feedback.data = str(self.ArmClient.GetArmAtTrayPos())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetArmAtTurntablePos':
            self._feedback.feedback.data = str(self.ArmClient.GetArmAtTurntablePos())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetTurntableAtCCWPos':
            self._feedback.feedback.data = str(self.ArmClient.GetTurntableAtCCWPos())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetTurntableAtCWPos':
            self._feedback.feedback.data = str(self.ArmClient.GetTurntableAtCWPos())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetArmHasError':
            self._feedback.feedback.data = str(self.ArmClient.GetArmHasError())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)                       
        elif cmd == 'GetArmHasStopped':
            self._feedback.feedback.data = str(self.ArmClient.GetArmHasStopped())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetArmInPositionArea':
            self._feedback.feedback.data = str(self.ArmClient.GetArmInPositionArea())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetArmInTargetPos':
            self._feedback.feedback.data = str(self.ArmClient.GetArmInTargetPos())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetArmIsEnabled':
            self._feedback.feedback.data = str(self.ArmClient.GetArmIsEnabled())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetArmIsHomed':
            self._feedback.feedback.data = str(self.ArmClient.GetArmIsHomed())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetArmIsMoving':
            self._feedback.feedback.data = str(self.ArmClient.GetArmIsMoving())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetArmSoftLimitMax':
            self._feedback.feedback.data = str(self.ArmClient.GetArmSoftLimitMax())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetArmSoftLimitMin':
            self._feedback.feedback.data = str(self.ArmClient.GetArmSoftLimitMin())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)
        elif cmd == 'GetGripperIsClosed':
            self._feedback.feedback.data = str(self.ArmClient.GetGripperIsClosed())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)     
        elif cmd == 'GetArmAtCandlePos':
            self._feedback.feedback.data = str(self.ArmClient.GetArmAtCandlePos())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)     
        elif cmd == 'GetArmAtPreGraspFromFloorPos':
            self._feedback.feedback.data = str(self.ArmClient.GetArmAtPreGraspFromFloorPos())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result.data = str2bool(self._feedback.feedback.data)     
    
    
        # SET Functions
        elif cmd == 'SetMoveToHomePos':
            feedback = self.ArmClient.SetMoveToHomePos()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data        
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmAtHomePos, 20)        
        elif cmd == 'SetMoveToCandlePos':
            feedback = self.ArmClient.SetMoveToCandlePos()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data        
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmAtCandlePos, 60)        
        elif cmd == 'SetMoveToLearningPos':
            feedback = self.ArmClient.SetMoveToLearningPos()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmAtLearningPos, 60)
        elif cmd == 'SetMoveToTrayPos':
            feedback = self.ArmClient.SetMoveToTrayPos()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmAtHomePos, 60)
        elif cmd == 'SetMoveToPreGraspFromFloorPos':
            feedback = self.ArmClient.SetMoveToPreGraspFromFloorPos()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmAtPreGraspFromFloorPos, 20)
        elif cmd == 'SetMoveToPreGraspFromTablePos':
            feedback = self.ArmClient.SetMoveToPreGraspFromTablePos()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmAtPreGraspFromTablePos, 60)
        elif cmd == 'SetStoreTurntable':
            feedback = self.ArmClient.SetStoreTurntable()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmAtHomePos, 70)
        elif cmd == 'SetTurnTurntableCW':
            feedback = self.ArmClient.SetTurnTurntableCW()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetTurntableAtCWPos, 60)        
        elif cmd == 'SetTurnTurntableCCW':
            feedback = self.ArmClient.SetTurnTurntableCCW()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetTurntableAtCCWPos, 60)
        elif cmd == 'SetStartArmReference':
            feedback = self.ArmClient.SetStartArmReference()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmAtHomePos, 120)
        elif cmd == 'SetDisableArm':
            feedback = self.ArmClient.SetDisableArm()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = Bool(feedback[1]=='COMMAND_OK')
        elif cmd == 'SetEnableArm':
            feedback = self.ArmClient.SetEnableArm()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = Bool(feedback[1]=='COMMAND_OK')
        elif cmd == 'SetOpenGripper':
            feedback = self.ArmClient.SetOpenGripper()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = self.CheckNegatedTargetPos(feedback, self.ArmClient.GetGripperIsClosed, 10) 
        elif cmd == 'SetCloseGripper':
            feedback = self.ArmClient.SetCloseGripper()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data        
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetGripperIsClosed, 10) 
        elif cmd == 'SetResetArm':
            feedback = self.ArmClient.SetResetArm()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = Bool(feedback[1]=='COMMAND_OK')
        elif cmd == 'SetStopArmMove':
            feedback = self.ArmClient.SetStopArmMove()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = Bool(feedback[1]=='COMMAND_OK')
        elif cmd == 'SetAbsolutePos':
            feedback = self.ArmClient.SetAbsolutePos(float(input[1]),float(input[2]),float(input[3]),float(input[4]),float(input[5]),float(input[6])) #(90, 0, 50, 0, 110, 0)
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = Bool(feedback[1]=='COMMAND_OK')
        elif cmd == 'SetStartMove':
            feedback = self.ArmClient.SetStartMove(float(input[1]))   #(10) #10 Grad/Sec
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data        
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmInTargetPos, 60) 
    
    
        # For Interpolation Mode    #no check of logic!!
        elif cmd == 'SetClearPosBuffer':
            feedback = self.ArmClient.SetClearPosBuffer()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            #@Andreas: how to check success? -> if feedback is COMMAND_OK, it has been received in PLC, therefor it will be executed
            #@David: maybe a timeout should be implemented to prevent server from hanging?
            self._result.result = Bool(feedback[1]=='COMMAND_OK')
        elif cmd == 'SetPositionsForInterpolation':
            #(90, 86, 70, 0, 110, 0)
            feedback = self.ArmClient.SetPositionsForInterpolation(float(input[1]),float(input[2]),float(input[3]),float(input[4]),float(input[5]),float(input[6]))
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = Bool(feedback[1]=='COMMAND_OK')
            #@David PLC returns COMMAND_OK if successful or POSITION_BUFFER_FULL if the positions could no be added -> ClearPosBuffer to execute
            #self.ArmClient.SetPositionsForInterpolation(90, 70, 65, 0, 110, 0)
            #self.ArmClient.SetPositionsForInterpolation(90, 60, 60, 0, 110, 0)
            #self.ArmClient.SetPositionsForInterpolation(90, 50, 60, 0, 110, 0)
            #self.ArmClient.SetPositionsForInterpolation(90, 40, 55, 0, 110, 0)
            #self.ArmClient.SetPositionsForInterpolation(90, 30, 55, 0, 110, 0)
            #self.ArmClient.SetPositionsForInterpolation(90, 20, 50, 0, 110, 0)
            #self.ArmClient.SetPositionsForInterpolation(90, 0, 50, 0, 110, 0)
        elif cmd == 'SetPositionsForInterpolationReady':
            feedback = self.ArmClient.SetPositionsForInterpolationReady()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = Bool(feedback[1]=='COMMAND_OK')
            #@David PLC returns COMMAND_OK if successful
        elif cmd == 'SetStartInterpolation':
            feedback = self.ArmClient.SetStartInterpolation()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data        
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmInTargetPos, 60) 
            #@David PLC returns COMMAND_OK if successful or NO_POSITIONS, ARM_HAS_ERROR, ... if not
    
    
    
        #    =======================================================        =======================================================
    
    
        #new (16.10.2014): executes whole trajectory for grasping [from floor] including closing gripper and going to back to start position of arm/hand
        # input: trajectory in the form "SetExecuteGrasp trajectory_waypoint1_value_joint1 tp1_vj2 ... tp1_vj_number of joints ... trajectory_waypoint_n ...
        elif cmd == 'SetExecuteGrasp':
            nr_j  = 6    # number of joints for arm
            nr_wp = (len(input)-1)/6  #number of waypoint for trajectory
         
            #MOVE ARM to Position above grasp Position
            feedback = self.ArmClient.SetAbsolutePos(float(input[1]),float(input[2]),float(input[3]),float(input[4]),float(input[5]),float(input[6])) #(90, 0, 50, 0, 110, 0)
            print self.ArmClient.SetStartMove(10.0)   #(10) #10 Grad/Sec
            print "sleep 1 second"
            rospy.sleep(1)
            #MOVE ARM TO GRASP POSITION
            #set waypoints for moving arm to target position
            for x in range(0, nr_wp):
                feedback = self.ArmClient.SetPositionsForInterpolation(float(input[6*x+1]),float(input[6*x+2]),float(input[6*x+3]),float(input[6*x+4]),float(input[6*x+5]),float(input[6*x+6]))
                self._feedback.feedback.data = str(feedback)
                self._as.publish_feedback(self._feedback)
            # set interpolation ready
            feedback = self.ArmClient.SetPositionsForInterpolationReady()
            self._feedback.feedback.data = str(feedback)
            self._as.publish_feedback(self._feedback)
            #print "ArmActionServer, feedback: ", self._feedback.feedback.data
            # set start interpolation (execute movement)
            feedback = self.ArmClient.SetStartInterpolation()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._as.publish_feedback(self._feedback)
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmInTargetPos, 60)  # wait until arm has reached final grasp position
    
            #close gripper
            feedback = self.ArmClient.SetCloseGripper()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback for closing gripper: ", self._feedback.feedback.data  
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetGripperIsClosed, 10)
            
            print "wait 4 seconds"
            rospy.sleep(4)  #wait <=================================================================== waiting!!!!!!
            
            # MOVE ARM BACK (reverted trajectory)
            # set waypoints for moving arm to target position
            for x in reversed(range(0, nr_wp)):
                feedback = self.ArmClient.SetPositionsForInterpolation(float(input[6*x+1]),float(input[6*x+2]),float(input[6*x+3]),float(input[6*x+4]),float(input[6*x+5]),float(input[6*x+6]))
                self._feedback.feedback.data = str(feedback)
                self._as.publish_feedback(self._feedback)
            # set interpolation ready
            feedback = self.ArmClient.SetPositionsForInterpolationReady()
            self._feedback.feedback.data = str(feedback)
            self._as.publish_feedback(self._feedback)
            #print "ArmActionServer, feedback: ", self._feedback.feedback.data
            # set start interpolation (execute movement)
            feedback = self.ArmClient.SetStartInterpolation()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._as.publish_feedback(self._feedback)
            #move arm MANUALLY (fixed values ->DANGEROUS!!!) to pregraspfromfloor position
            print "sleep 5 seconds before arm moves MANUALLY (fixed values ->DANGEROUS!!!) to pregraspfromfloor position"
            rospy.sleep(5)
            feedback = self.ArmClient.SetAbsolutePos(69.71,31.39,96.31,122.2,109.74,0) #(90, 0, 50, 0, 110, 0)
            print self.ArmClient.SetStartMove(4.0)   #(10) #10 Grad/Sec
            
            
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmInTargetPos, 60)  # wait until arm has reached its initial pre-grasp position
            #=======================================================        =======================================================
        #dddd
        elif cmd == 'SetMoveToCheckGraspFromFloorPosition':
            #move arm MANUALLY (fixed values ->DANGEROUS!!!) to pregraspfromfloor position
            feedback = self.ArmClient.SetAbsolutePos(63.69,31.39,96.31,122.2,67.79,0) #(90, 0, 50, 0, 110, 0)
            print "start moving arm to check-if-grasp-from-floor-was-successful-position MANUALLY (hard coded joint values)"
            print self.ArmClient.SetStartMove(5.0)   #(10) #10 Grad/Sec
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmInTargetPos, 60)  # wait until arm has reached its initial pre-grasp position
                   
        elif cmd == 'SetMoveToPreGraspFromFloorPosManually':
            #move arm MANUALLY (fixed values ->DANGEROUS!!!) to pregraspfromfloor position
            feedback = self.ArmClient.SetAbsolutePos(69.71,31.39,96.31,122.2,109.74,0) #(90, 0, 50, 0, 110, 0)
            print "start moving arm to pre-grasp-from-floor-position MANUALLY (hard coded joint values)"
            print self.ArmClient.SetStartMove(5.0)   #(10) #10 Grad/Sec
            self._result.result = self.CheckTargetPos(feedback, self.ArmClient.GetArmInTargetPos, 60)  # wait until arm has reached its initial pre-grasp position
            
    
        elif cmd == 'help':
            self.help()
        else:
            print "=====================> ArmActionServer.py: COMMAND was not found!!!!!"
    
    
        #if arm was stopped
        if self._stop_arm:
            print "david: arm was stopped manually, new command send to plc"
            feedback = self.ArmClient.SetStopArmMove()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = Bool(feedback[1]=='COMMAND_OK')
            self._stop_arm = False
    
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
        elif cmd == 'SetMoveToCheckGraspFromFloorPosition': #df 5.2.2015
            print self.ArmClient.SetMoveToCheckGraspFromFloorPosition()
        elif cmd == 'SetMoveToPreGraspFromFloorPosManually':
            print self.ArmClient.SetMoveToPreGraspFromFloorPosManually()
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
        #arm stop function
        elif cmd == 'StopArm':
            self._stop_arm = True
            print "StopArm command received via topic /arm/commands"
        elif cmd == 'help':
            self.help()
    


    def help(self): 
         
        print " ===   GET Functions === "
        print 'GetArmState'
        print 'GetActualPosition'
        print 'GetArmAtHomePos'
        print 'GetArmAtLearningPos'
        print 'GetArmAtTrayPos'    
        print 'GetArmAtPreGraspFromFloorPos'
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
        print 'SetMoveToCheckGraspFromFloorPosition'
        print 'SetMoveToPreGraspFromFloorPosManually'
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
    rospy.init_node('arm_action_server')
    print "ROS node arm_action_server started"
    #armservertopic = "armactionlibtopic"
    armclient = ArmActionServerROS(rospy.get_name())

    rospy.spin()

#!/usr/bin/python
'''
Created on 10.09.2014

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

	print "ArmActionClient: create goal"
	# Creates a goal to send to the action server.
	goal = hobbit_msgs.msg.ArmServerGoal(command=cmd)

	print "ArmActionClient: send goal"
	# Sends the goal to the action server.
	self.client.send_goal(goal, feedback_cb=self.feedback_cb)

	print "wait for result"
	# Waits for the server to finish performing the action.
	self.client.wait_for_result()
	

	# Prints out the result of executing the action
	returnval = self.client.get_result()  # 
	print "cmd: ", cmd
	print "ArmActionClient.py: class ArmActionClient: function arm_action_client (=> sends/receives actionlib data from/to server): returnval.result.data: ", returnval.result.data
	return returnval
        
    def feedback_cb(self, feedback):
	print "feedback_cb executed!"
	#print "feedback type: ", (type) (feedback)
	self.last_feedback = feedback
	print "==========> feedback: ", self.last_feedback 



    #_____________________SET Commands_____________________#
    
    #as (action server client)
    def SetAbsolutePos(self,Value1, Value2, Value3, Value4, Value5, Value6):
        result = self.arm_action_client(String('SetAbsolutePos {Val1} {Val2} {Val3} {Val4} {Val5} {Val6}'.format(Val1=round(Value1,2),Val2=round(Value2,2),Val3=round(Value3,2),Val4=round(Value4,2),Val5=round(Value5,2),Val6=round(Value6,2))))
    
   #as
    def SetStartInterpolation(self):
        result = self.arm_action_client(String('SetStartInterpolation'))
	return result.result.data
        
    
    #as
    def SetPositionsForInterpolation(self,Value1, Value2, Value3, Value4, Value5, Value6):
        result = self.arm_action_client(String('SetPositionsForInterpolation {Val1} {Val2} {Val3} {Val4} {Val5} {Val6}'.format(Val1=Value1,Val2=Value2,Val3=Value3,Val4=Value4,Val5=Value5,Val6=Value6)))
	return result.result.data
    
    #as
    def SetPositionsForInterpolationReady(self):
        result = self.arm_action_client(String('SetPositionsForInterpolationReady'))
	return result.result.data        
    
    #as
    def SetStartMove(self, Velocity):
        # Velocity in [deg/s]; Maximum Velocity = 20
        result = self.arm_action_client(String('SetStartMove {Val1}'.format(Val1=round(Velocity,2))))
	return result.result.data

    #as   
    def SetMoveToHomePos(self):
        result = self.arm_action_client(String('SetMoveToHomePos'))
	return result.result.data

    #as  
    def SetMoveToLearningPos(self):
        result = self.arm_action_client(String('SetMoveToLearningPos'))
   	return result.result.data
    
    #as
    def SetMoveToTrayPos(self):
        result = self.arm_action_client(String('SetMoveToTrayPos'))
	return result.result.data
    
    #as
    def SetMoveToPreGraspFromFloorPos(self):
        result = self.arm_action_client(String('SetMoveToPreGraspFromFloorPos'))
	return result.result.data
    
    #as
    def SetMoveToPreGraspFromTablePos(self):
        result = self.arm_action_client(String('SetMoveToPreGraspFromTablePos'))
	return result.result.data
    
    #as
    def SetStoreTurntable(self):
        result = self.arm_action_client(String('SetStoreTurntable'))
	return result.result.data
    
    #as
    def SetMoveToCandlePos(self):
        result = self.arm_action_client(String('SetMoveToCandlePos'))
	return result.result.data
    
    #as
    def SetTurnTurntableCW(self):
        result = self.arm_action_client(String('SetTurnTurntableCW'))
	return result.result.data
    
    #as
    def SetTurnTurntableCCW(self):
        result = self.arm_action_client(String('SetTurnTurntableCCW'))
	return result.result.data
    
    #as
    def SetStartArmReference(self):
        result = self.arm_action_client(String('SetStartArmReference'))
	return result.result.data

    #as
    def SetDisableArm(self):
        result = self.arm_action_client(String('SetDisableArm'))
	return result.result.data

    #as    
    def SetEnableArm(self):
        result = self.arm_action_client(String('SetEnableArm'))
	return result.result.data
    
    #as
    def SetOpenGripper(self):
        result = self.arm_action_client(String('SetOpenGripper'))
	return result.result.data
    
    #as
    def SetCloseGripper(self):
        result = self.arm_action_client(String('SetCloseGripper'))
	return result.result.data

    #as
    def SetResetArm(self):
        result = self.arm_action_client(String('SetResetArm'))
	return result.result.data
    
    #as
    def SetStopArmMove(self):
        result = self.arm_action_client(String('SetStopArmMove'))
	return result.result.data

    #as
    def SetClearPosBuffer(self):
        result = self.arm_action_client(String('SetClearPosBuffer'))
	return result.result.data
    
    
    
    #_____________________GET Commands_____________________#
    #as
    def GetArmState(self):
        result = self.arm_action_client(String('GetArmState'))
	return result.result.data
       
    #as
    def GetActualPosition(self):
        result = self.arm_action_client(String('GetActualPosition'))
	return result.result.data
       
    #as
    def GetGripperIsClosed(self):
        result = self.arm_action_client(String('GetGripperIsClosed'))
	return result.result.data
       
    #as    
    def GetArmIsMoving(self):
        result = self.arm_action_client(String('GetArmIsMoving'))
	return result.result.data
       
    #as    
    def GetArmHasError(self):
        result = self.arm_action_client(String('GetArmHasError'))
	return result.result.data
       
    #as
    def GetArmHasStopped(self):
        result = self.arm_action_client(String('GetArmHasStopped'))
	return result.result.data
       
    #as    
    def GetArmIsEnabled(self):
        result = self.arm_action_client(String('GetArmIsEnabled'))
	return result.result.data
       
    #as
    def GetArmIsHomed(self):
        result = self.arm_action_client(String('GetArmIsHomed'))
	return result.result.data
       
    #as    
    def GetArmInPositionArea(self):
        result = self.arm_action_client(String('GetArmInPositionArea'))
	return result.result.data
       
    #as    
    def GetArmInTargetPos(self):
        result =  self.arm_action_client(String('GetArmInTargetPos'))
	return result.result.data
       
    #as    
    def GetArmAtHomePos(self):
        result = self.arm_action_client(String('GetArmAtHomePos'))
	return result.result.data

    #as    
    def GetArmAtLearningPos(self):
        result = self.arm_action_client(String('GetArmAtLearningPos'))
	return result.result.data
       
    #as
    def GetArmAtTrayPos(self):
        result = self.arm_action_client(String('GetArmAtTrayPos'))
	return result.result.data
       
    #as
    def GetArmAtTurntablePos(self):
        result = self.arm_action_client(String('GetArmAtTurntablePos'))
	return result.result.data
       
    #as    
    def GetArmAtPreGraspFromFloorPos(self):
        result = self.arm_action_client(String('GetArmAtPreGraspFromFloorPos'))
	return result.result.data
       
    #as
    def GetTurntableAtCCWPos(self):
        result = self.arm_action_client(String('GetTurntableAtCCWPos'))
	return result.result.data
       
    #as
    def GetTurntableAtCWPos(self):
        result = self.arm_action_client(String('GetTurntableAtCWPos'))
	return result.result.data

    #as
    def GetArmAtCandlePos(self):
        result = self.arm_action_client(String('GetArmAtCandlePos'))
	return result.result.data
 
    #as
    def GetArmSoftLimitMax(self):
        result = self.arm_action_client(String('GetArmSoftLimitMax'))
	return result.result.data
    
    #as
    def GetArmSoftLimitMin(self):
        result = self.arm_action_client(String('GetArmSoftLimitMin'))
	return result.result.data


if __name__ == '__main__':
   
    #   === T E S T  ===

    # Node name
    rospy.init_node('arm_action_client_test')
    print "node arm_action_client test started"
    arm_client = ArmActionClient()
    cmd = String ("GetArmState")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res
    print "david last_feedback: ", arm_client.last_feedback

    #print "SetResetArm: ", arm_client.SetResetArm()
    print "===============GetActualPosition()================"
    print "get arm act pos: ", arm_client.GetActualPosition()

    print "===============GetArmAtPreGraspFromFloorPos()================"
    print "get arm GetArmAtPreGraspFromFloorPos: ", arm_client.GetArmAtPreGraspFromFloorPos()
    

    #raw_input("press key to get PositionsForInterpolation(armactionserver) POS1=PreGraspFromFloor")
    #cmd = String ("SetPositionsForInterpolation 69.71 43.44 86.3 134.07 107.56 3.96")
    #res = arm_client.arm_action_client(cmd)
    
    #OPEN GRIPPER
    #raw_input("press key to open gripper get SetOpenGripper(armactionserver)")
    #cmd = String ("SetOpenGripper")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res 

	#69.71, 43.44, 86.3, 134.07, 107.56, 3.96 pregrap

    #raw_input("press key to get PositionsForInterpolation(armactionserver) POS2")
    #cmd = String ("SetPositionsForInterpolation 72 43 82 134 101 4")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res 

    #raw_input("press key to get PositionsForInterpolation(armactionserver) POS3")
    #cmd = String ("SetPositionsForInterpolation 78 43 84 134 96 4")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res 

    #raw_input("press key to get PositionsForInterpolation(armactionserver) POS4")
    #cmd = String ("SetPositionsForInterpolation 78 55 78 150 90 4")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res 

    #raw_input("press key to get PositionsForInterpolationReady (armactionserver)")
    cmd = String ("SetPositionsForInterpolationReady")
    res = arm_client.arm_action_client(cmd)

    raw_input("press key to move arm after position interpolation")
    cmd = String("SetStartInterpolation")
    res = arm_client.arm_action_client(cmd)

    #CLOSE GRIPPER
    #raw_input("press key to close gripper get SetCloseGripper(armactionserver)")
    #cmd = String ("SetCloseGripper")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res 




    #raw_input("press key to get PositionsForInterpolation(armactionserver) POS3")
    #cmd = String ("SetPositionsForInterpolation 78 43 84 134 96 4")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res 

    '''
    #raw_input("press key to get PositionsForInterpolation(armactionserver) POS2")
    cmd = String ("SetPositionsForInterpolation 72 43 82 134 101 4")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 

    #raw_input("press key to get PositionsForInterpolation(armactionserver) POS1=PreGraspFromFloor")
    cmd = String ("SetPositionsForInterpolation 70 43 86 134 108 4")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 

    #raw_input("press key to get PositionsForInterpolationReady (armactionserver)")
    cmd = String ("SetPositionsForInterpolationReady")
    res = arm_client.arm_action_client(cmd)

    raw_input("press key to move arm after position interpolation")
    cmd = String("SetStartInterpolation")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 

    #OPEN GRIPPER
    raw_input("press key to open gripper get SetOpenGripper(armactionserver)")
    cmd = String ("SetOpenGripper")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 
    '''





    
    #print "===============GetArmAtCandlePos()================"
    #print "get arm act pos: ", arm_client.GetArmAtCandlePos()
    #print "===============GetArmAtLearningPos()================"
    #print "get arm act pos: ", arm_client.GetArmAtLearningPos()
    #print "set arm SetMoveToLearningPos: ", arm_client.SetMoveToLearningPos()
    #print "move to candle: ", arm_client.SetMoveToCandlePos()
    #res = arm_client.SetTurnTurntableCW()
    #print "get arm act pos: ", arm_client.GetActualPosition()
    #print "get arm at GetTurntableAtCWPos: ", arm_client.GetTurntableAtCWPos()
    #print "get arm at learning pos: ", arm_client.GetArmAtLearningPos()
    #print "storeturntable: ", arm_client.SetStoreTurntable()
    #print "get arm at act pos: ", arm_client.GetActualPosition()
    #res = arm_client.SetMoveToTrayPos()

    #print "result: ", res



	
    exit()




    raw_input("press key to get GetActualPosition (armactionserver)")
    cmd = String ("GetActualPosition")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res

    #raw_input("press key to get GetArmIsHomed (armactionserver)")
    #cmd = String ("GetArmIsHomed")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res

    if not res.result.data:
        raw_input("press key to start homing (armactionserver)")
        cmd = String ("SetStartArmReference")
        res = arm_client.arm_action_client(cmd)
        print "Result: ", res

    raw_input("press key to set SetMoveToHomePos (armactionserver)")
    cmd = String ("SetMoveToHomePos")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res

    #raw_input("press key to get GetArmAtLearningPos (armactionserver)")
    #cmd = String ("GetArmAtLearningPos")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res
    
    raw_input("press key to get GetTurntableAtCWPos (armactionserver)")
    cmd = String ("GetTurntableAtCWPos")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res

    #exit
    #raw_input("press key to get GetArmInTargetPos (armactionserver)")
    #cmd = String ("GetArmInTargetPos")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res

#    raw_input("press key to get GetArmIsEnabled (armactionserver)")
#    cmd = String ("GetArmIsEnabled")
#    res = arm_client.arm_action_client(cmd)
#    print "Result: ", res

#    raw_input("press key to get GetArmAtTrayPos (armactionserver)")
#    cmd = String ("GetArmAtTrayPos")
#    res = arm_client.arm_action_client(cmd)
#    print "Result: ", res

#    raw_input("press key to get GetArmSoftLimitMax (armactionserver)")
#    cmd = String ("GetArmSoftLimitMax")
#    res = arm_client.arm_action_client(cmd)
#    print "Result: ", res

#    raw_input("press key to get GetArmAtCandlePos (armactionserver)")
#    cmd = String ("GetArmAtCandlePos")
#    res = arm_client.arm_action_client(cmd)
#    print "Result: ", res

#    raw_input("press key to get GetArmHasStopped (armactionserver)")
#    cmd = String ("GetArmHasStopped")
#    res = arm_client.arm_action_client(cmd)
#    print "Result: ", res

#    raw_input("press key to get GetArmSoftLimitMin (armactionserver)")
#    cmd = String ("GetArmSoftLimitMin")
#    res = arm_client.arm_action_client(cmd)
#    print "Result: ", res

#    raw_input("press key to get GetGripperIsClosed (armactionserver)")
#    cmd = String ("GetGripperIsClosed")
#    res = arm_client.arm_action_client(cmd)
#    print "Result: ", res

    #raw_input("press key to finish ... get GetArmState")
    #arm_pub.GetArmState()

    #raw_input("press key to get GetActualPosition")
    #arm_pub.GetActualPosition()

    #raw_input("press key to get GetArmIsHomed")
    #arm_pub.GetArmIsHomed()

    #raw_input("press key to get GetArmAtLearningPos")
    #arm_pub.GetArmAtLearningPos()

    #raw_input("press key to get GetArmInTargetPos")
    #arm_pub.GetArmInTargetPos()

    #raw_input("press key to get GetArmIsEnabled")
    #arm_pub.GetArmIsEnabled()

    #          S E T
    raw_input("press key to get ResetArm (armactionserver)")
    cmd = String ("SetResetArm")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res

    #raw_input("press key to get SetCloseGripper (armactionserver)")
    #cmd = String ("SetCloseGripper")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res


    #raw_input("press key to get SetOpenGripper (armactionserver)")
    #cmd = String ("SetOpenGripper")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res

    raw_input("press key to get SetMoveToLearningPos (armactionserver)")
    cmd = String ("SetMoveToLearningPos")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res

    if not res.result.data:	
        raw_input("press key to get SetStoreTurntable (armactionserver)")
    else:
        cmd = String ("SetStoreTurntable")
        res = arm_client.arm_action_client(cmd)
        print "Result: ", res
    '''
    #raw_input("press key to get SetMoveToPreGraspFromFloorPos (armactionserver)")
    #cmd = String ("SetMoveToPreGraspFromFloorPos")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res

    raw_input("press key to get SetMoveToPreGraspFromTablePos (armactionserver)")
    cmd = String ("SetMoveToPreGraspFromTablePos")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res

    raw_input("press key to get SetMoveToTrayPos (armactionserver)")
    cmd = String ("SetMoveToTrayPos")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res

    raw_input("press key to get SetMoveToCandlePos (armactionserver)")
    cmd = String ("SetMoveToCandlePos")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res
    '''
#    (90, 86, 70, 0, 110, 0)
    raw_input("press key to get SetAbsolutePos (armactionserver)")
    cmd = String ("SetAbsolutePos 85 80 85 0 100 0")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 
    
    raw_input("press key to get startMove (armactionserver)")
    cmd = String ("SetStartMove 10")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 

    raw_input("press key to get PositionsForInterpolation(armactionserver)")
    cmd = String ("SetPositionsForInterpolation 85 82 85 0 100 0")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 

    raw_input("press key to get PositionsForInterpolation(armactionserver)")
    cmd = String ("SetPositionsForInterpolation 87 84 83 0 105 0")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 

    raw_input("press key to get PositionsForInterpolation(armactionserver)")
    cmd = String ("SetPositionsForInterpolation 90 86 70 0 110 0")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 

    raw_input("press key to get PositionsForInterpolationREady (armactionserver)")
    cmd = String ("SetPositionsForInterpolationReady")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 

    #raw_input("press key to get ClearPosBuffer (armactionserver)")
    #cmd = String ("SetClearPosBuffer")
    #res = arm_client.arm_action_client(cmd)
    #print "Result: ", res 

    raw_input("press key to get StartInterpolation (armactionserver)")
    cmd = String ("SetStartInterpolation")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 

    raw_input("press key to get SetMoveToHomePos (armactionserver)")
    cmd = String ("SetMoveToHomePos")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 
    
    raw_input("press key to get DisableArm (armactionserver)")
    cmd = String ("SetDisableArm")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res

    raw_input("press key to get EnableArm (armactionserver)")
    cmd = String ("SetEnableArm")
    res = arm_client.arm_action_client(cmd)
    print "Result: ", res 
    
    print "done!!!"

    #rospy.spin()

#ArmClient.SetPositionsForInterpolation(0, 0, 0, 0, 0, 0)

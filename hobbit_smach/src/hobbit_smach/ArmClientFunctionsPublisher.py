#!/usr/bin/python
'''
Created on 04.09.2014

author: david fischinger
'''

PKG = 'hobbit_smach'

import roslib
roslib.load_manifest(PKG)
import rospy
import ArmControllerFunctions
from std_msgs.msg import String


class ArmClientFunctionsPublisher():
    def __init__(self):
        self.arm_client_pub = rospy.Publisher("/arm/commands", String)
        
 
        
    #_____________________SET Commands_____________________#
    
    #pub
    def SetAbsolutePos(self,Value1, Value2, Value3, Value4, Value5, Value6):
        self.arm_client_pub.publish(String('SetAbsolutePos {Val1} {Val2} {Val3} {Val4} {Val5} {Val6}'.format(Val1=round(Value1,2),Val2=round(Value2,2),Val3=round(Value3,2),Val4=round(Value4,2),Val5=round(Value5,2),Val6=round(Value6,2))))
        return True
    
   #pub
    def SetStartInterpolation(self):
        self.arm_client_pub.publish(String('SetStartInterpolation'))
        return True
    
    #pub
    def SetPositionsForInterpolation(self,Value1, Value2, Value3, Value4, Value5, Value6):
        self.arm_client_pub.publish(String('SetPositionsForInterpolation {Val1} {Val2} {Val3} {Val4} {Val5} {Val6}'.format(Val1=Value1,Val2=Value2,Val3=Value3,Val4=Value4,Val5=Value5,Val6=Value6)))
        return True
    
    #pub
    def SetPositionsForInterpolationReady(self):
        self.arm_client_pub.publish(String('SetPositionsForInterpolationReady'))
        return True
    
    #pub
    def SetStartMove(self, Velocity):
        # Velocity in [deg/s]; Maximum Velocity = 20
        self.arm_client_pub.publish(String('SetStartMove {Val1}'.format(Val1=round(Velocity,2))))
        return True

    #pub   
    def SetMoveToHomePos(self):
        self.arm_client_pub.publish(String('SetMoveToHomePos'))
        return True

    #pub  
    def SetMoveToLearningPos(self):
        self.arm_client_pub.publish(String('SetMoveToLearningPos'))
        return True    
    
    #pub
    def SetMoveToTrayPos(self):
        self.arm_client_pub.publish(String('SetMoveToTrayPos'))
        return True
    
    #pub
    def SetMoveToPreGraspFromFloorPos(self):
        self.arm_client_pub.publish(String('SetMoveToPreGraspFromFloorPos'))
        return True
    
    #pub
    def SetMoveToPreGraspFromTablePos(self):
        self.arm_client_pub.publish(String('SetMoveToPreGraspFromTablePos'))
        return True
    
    #pub
    def SetStoreTurntable(self):
        self.arm_client_pub.publish(String('SetStoreTurntable'))
        return True
    
    #pub
    def SetMoveToCandlePos(self):
        self.arm_client_pub.publish(String('SetMoveToCandlePos'))
        return True
    
    #pub
    def SetTurnTurntableCW(self):
        self.arm_client_pub.publish(String('SetTurnTurntableCW'))
        return True
    
    #pub
    def SetTurnTurntableCCW(self):
        self.arm_client_pub.publish(String('SetTurnTurntableCCW'))
        return True
    
    #pub
    def SetStartArmReference(self):
        self.arm_client_pub.publish(String('SetStartArmReference'))
        return True

    #pub    
    def SetDisableArm(self):
        self.arm_client_pub.publish(String('SetDisableArm'))
        return ReceiveDataList

    #pub    
    def SetEnableArm(self):
        self.arm_client_pub.publish(String('SetEnableArm'))
        return True
    
    #pub
    def SetOpenGripper(self):
        self.arm_client_pub.publish(String('SetOpenGripper'))
        return True
    
    #pub
    def SetCloseGripper(self):
        self.arm_client_pub.publish(String('SetCloseGripper'))
        return True

    #pub    
    def SetResetArm(self):
        self.arm_client_pub.publish(String('SetResetArm'))
        return True
    
    #pub
    def SetStopArmMove(self):
        self.arm_client_pub.publish(String('SetStopArmMove'))
        return True

    #pub
    def SetClearPosBuffer(self):
        self.arm_client_pub.publish(String('SetClearPosBuffer'))
        return True
    
    
    
    #_____________________GET Commands_____________________#
    #pub
    def GetArmState(self):
        self.arm_client_pub.publish(String('GetArmState'))
        return True
    
    #pub
    def GetActualPosition(self):
        self.arm_client_pub.publish(String('GetActualPosition'))
        return True
    
    #pub
    def GetGripperIsClosed(self):
        self.arm_client_pub.publish(String('GetGripperIsClosed'))
        return True

    #pub    
    def GetArmIsMoving(self):
        self.arm_client_pub.publish(String('GetArmIsMoving'))
        return True

    #pub    
    def GetArmHasError(self):
        self.arm_client_pub.publish(String('GetArmHasError'))
        return True
    
    #pub
    def GetArmHasStopped(self):
        self.arm_client_pub.publish(String('GetArmHasStopped'))
        return True

    #pub    
    def GetArmIsEnabled(self):
        self.arm_client_pub.publish(String('GetArmIsEnabled'))
        return True
    
    #pub
    def GetArmIsHomed(self):
        self.arm_client_pub.publish(String('GetArmIsHomed'))
        return True

    #pub    
    def GetArmInPositionArea(self):
        self.arm_client_pub.publish(String('GetArmInPositionArea'))
        return True

    #pub    
    def GetArmInTargetPos(self):
        self.arm_client_pub.publish(String('GetArmInTargetPos'))
        return True

    #pub    
    def GetArmAtHomePos(self):
        self.arm_client_pub.publish(String('GetArmAtHomePos'))
        return True

    #pub    
    def GetArmAtLearningPos(self):
        self.arm_client_pub.publish(String('GetArmAtLearningPos'))
        return True

    #pub
    def GetArmAtTrayPos(self):
        self.arm_client_pub.publish(String('GetArmAtTrayPos'))
        return True

    #pub
    def GetArmAtTurntablePos(self):
        self.arm_client_pub.publish(String('GetArmAtTurntablePos'))
        return True

    #pub    
    def GetArmAtPreGraspFromFloorPos(self):
        self.arm_client_pub.publish(String('GetArmAtPreGraspFromFloorPos'))
        return True
 
    #pub
    def GetTurntableAtCCWPos(self):
        self.arm_client_pub.publish(String('GetArmAtCCWPos'))
        return True

    #pub    
    def GetTurntableAtCWPos(self):
        self.arm_client_pub.publish(String('GetArmAtCWPos'))
        return True

    #pub        
    def GetArmAtCandlePos(self):
        self.arm_client_pub.publish(String('GetArmAtCandlePos'))
        return True
 
    #pub
    def GetArmSoftLimitMax(self):
        self.arm_client_pub.publish(String('GetArmSoftLimitMax'))
        return True
    
    #pub
    def GetArmSoftLimitMin(self):
        self.arm_client_pub.publish(String('GetArmSoftLimitMin'))
        return True


if __name__ == '__main__':
   
    # Node name
    rospy.init_node('arm_client_functions_publisher_test')
    print "node arm_client_functions_publisher test started"
    arm_pub = ArmClientFunctionsPublisher()

    raw_input("press key to get GetArmState")
    arm_pub.GetArmState()

    raw_input("press key to get GetActualPosition")
    arm_pub.GetActualPosition()

    raw_input("press key to get GetArmIsHomed")
    arm_pub.GetArmIsHomed()

    raw_input("press key to get GetArmAtLearningPos")
    arm_pub.GetArmAtLearningPos()

    raw_input("press key to get GetArmInTargetPos")
    arm_pub.GetArmInTargetPos()

    raw_input("press key to get GetArmIsEnabled")
    arm_pub.GetArmIsEnabled()

    #          S E T

    raw_input("press key to get SetCloseGripper")
    arm_pub.SetCloseGripper()

    raw_input("press key to get SetOpenGripper")
    arm_pub.SetOpenGripper()

    raw_input("press key to get SetMoveToLearningPos")
    arm_pub.SetMoveToLearningPos()

    raw_input("press key to get SetStoreTurntable")
    arm_pub.SetStoreTurntable()

    raw_input("press key to get SetMoveToPreGraspFromFloorPos")
    arm_pub.SetMoveToPreGraspFromFloorPos()

    raw_input("press key to get SetMoveToTrayPos")
    arm_pub.SetMoveToTrayPos()

    raw_input("press key to get SetMoveToPreGraspFromTablePos")
    arm_pub.SetMoveToPreGraspFromTablePos()

    raw_input("press key to get SetMoveToCandlePos")
    arm_pub.SetMoveToCandlePos()

    raw_input("press key to get SetMoveToHomePos")
    arm_pub.SetMoveToHomePos()

    print "done!!!"

    rospy.spin()

#ArmClient.SetPositionsForInterpolation(0, 0, 0, 0, 0, 0)

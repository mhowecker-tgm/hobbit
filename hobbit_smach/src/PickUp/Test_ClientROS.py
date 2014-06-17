'''
Created on 07.01.2014

@author: HofmannS, ROSified by D. Fischinger (17.6.2014)
'''

PKG = 'hobbit_smach'

import roslib
roslib.load_manifest(PKG)
from ArmControllerClientFunctions import ArmClientFunctions


class ArmClientROS:

    def __init__(self): 
         
        self.ArmClient = ArmClientFunctions('192.168.2.190')
	self.armCommands_sub = rospy.Subscriber("/arm/commands", String, self.arm_execute)



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
	    print self.ArmClient.GetTurntableAtCCWPos(
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
                    
	''' SET Functions '''

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

	''' For Interpolation Mode '''   #no check of logic!!

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



if __name__ == '__main__':
   
    # Node name
    rospy.init_node('arm_client_ros')
    armclient = ArmClientROS()

    rospy.spin()

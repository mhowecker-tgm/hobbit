'''
Created on 07.01.2014

@author: HofmannS, ROSified by D. Fischinger
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

	if cmd  
        


''' GET Functions '''
#print ArmClient.GetArmState()                          
#print ArmClient.GetActualPosition()                    
#print ArmClient.GetArmAtHomePos()                      
#print ArmClient.GetArmAtLearningPos()                  
#print ArmClient.GetArmAtTrayPos()                      
#print ArmClient.GetArmAtTurntablePos()                 
#print ArmClient.GetTurntableAtCCWPos()                 
#print ArmClient.GetTurntableAtCWPos()                              
#print ArmClient.GetArmHasError()                        
#print ArmClient.GetArmHasStopped()                      
#print ArmClient.GetArmInPositionArea()                  
#print ArmClient.GetArmInTargetPos()                     
#print ArmClient.GetArmIsEnabled()                       
#print ArmClient.GetArmIsHomed()                         
#print ArmClient.GetArmIsMoving()                        
#print ArmClient.GetArmSoftLimitMax()                    
#print ArmClient.GetArmSoftLimitMin()                    
#print ArmClient.GetGripperIsClosed()                    


''' SET Functions '''
#print ArmClient.SetAbsolutePos(90, 0, 50, 0, 110, 0)
#print ArmClient.SetStartMove(10) #10 Grad/Sec
#print ArmClient.SetMoveToHomePos()
#print ArmClient.SetMoveToLearningPos()
print ArmClient.SetMoveToTrayPos()
#print ArmClient.SetMoveToPreGraspFromFloorPos()
#print ArmClient.SetStoreTurntable()
#print ArmClient.SetTurnTurntableCW()
#print ArmClient.SetTurnTurntableCCW()
#print ArmClient.SetStartArmReference()
#print ArmClient.SetDisableArm()
#print ArmClient.SetEnableArm()
#print ArmClient.SetOpenGripper()
#print ArmClient.SetCloseGripper()
#print ArmClient.SetResetArm()
#print ArmClient.SetStopArmMove()

''' For Interpolation Mode '''
#ArmClient.SetClearPosBuffer()
#ArmClient.SetPositionsForInterpolation(90, 86, 70, 0, 110, 0)
#ArmClient.SetPositionsForInterpolation(90, 70, 65, 0, 110, 0)
#ArmClient.SetPositionsForInterpolation(90, 60, 60, 0, 110, 0)
#ArmClient.SetPositionsForInterpolation(90, 50, 60, 0, 110, 0)
#ArmClient.SetPositionsForInterpolation(90, 40, 55, 0, 110, 0)
#ArmClient.SetPositionsForInterpolation(90, 30, 55, 0, 110, 0)
#ArmClient.SetPositionsForInterpolation(90, 20, 50, 0, 110, 0)
#ArmClient.SetPositionsForInterpolation(90, 0, 50, 0, 110, 0)
#ArmClient.SetPositionsForInterpolationReady()
#ArmClient.SetStartInterpolation()


if __name__ == '__main__':
   
    # Node name
    rospy.init_node('arm_client_ros')
    armclient = ArmClientROS()

    s = rospy.Service('manage_grasphypothesis/get_best_grasphypothesis', BestGraspHypothesis, grasplist.get_best_grasp_hypothesis)
 
    rospy.spin()

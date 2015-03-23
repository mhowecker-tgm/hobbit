'''
Created on 07.01.2014

@author: HofmannS
'''
from ArmControllerClientFunctions import ArmClientFunctions

ArmClient = ArmClientFunctions('192.168.2.190')

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
print ArmClient.SetMoveToPreGraspFromFloorPos()
print ArmClient.SetAbsolutePos(64.54, 35.18, 90.24, 127.72, 111.61, 99.4)
raw_input("press key")
print ArmClient.SetStartMove(10) #10 Grad/Sec
#print ArmClient.SetMoveToHomePos()
#print ArmClient.SetMoveToLearningPos()
#print ArmClient.SetMoveToTrayPos()
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


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
print ArmClient.GetArmAtLearningPos()                  
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
#print ArmClient.SetAbsolutePos(27.56, 33.33, 12.45, 56.01, 44.44, 55.55)
#print ArmClient.SetStartMove(10)
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
#print ArmClient.SetClearPosBuffer()
#ArmClient.SetPositionsForInterpolation(0, 0, 0, 0, 0, 0)
#print ArmClient.SetStartInterpolation()


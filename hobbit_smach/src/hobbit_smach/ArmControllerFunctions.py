'''
Created on 08.10.2013

@author: HofmannS
'''
import string
#import threading
from pprint import pprint

AxisStateNames = []
AxisStateNames.append('ArmHasError')
AxisStateNames.append('ArmHomed')
AxisStateNames.append('ArmHasStopped')
AxisStateNames.append('ArmInPositionArea')
AxisStateNames.append('ArmInTargetPosition')
AxisStateNames.append('ArmIsDisabled')
AxisStateNames.append('ArmIsMoving')
AxisStateNames.append('ArmSoftLimitMax')
AxisStateNames.append('ArmSoftLimitMin')
AxisStateNames.append('ArmAtHomePos')
AxisStateNames.append('ArmAtLearningPos')
AxisStateNames.append('ArmAtTurntablePos')
AxisStateNames.append('ArmAtTrayPos')
AxisStateNames.append('ArmAtPreGraspFromFloorPos')
AxisStateNames.append('ArmAtPreGraspFromTablePos')
AxisStateNames.append('ArmAtCCWPos')
AxisStateNames.append('ArmAtCWPos')
AxisStateNames.append('GripperIsClosed')
#AxisStateNames.append('InterpolationPosReceived')
AxisStateNames.append('EmergencyPressed')
   
   
def PublishToROS(data):
    #Here the function for publishing Axis and Gripper State to ROS is implemented:
    print 'Publishing to ROS is not implemented yet!'


def CutString(RecieveString):
    RecieveString=string.rstrip(RecieveString,';')
    liste = string.split(RecieveString,';')
    if liste[-1] == '\x00':
        liste.remove('\x00')
    return liste

def CHANGE_AXISSTRING(AxisStateString):
    if len(AxisStateString)==32:
        oldStr = AxisStateString
    else:
        rest = 32-len(AxisStateString)
        oldStr = '0'*rest + AxisStateString
    newStr = oldStr[0]+oldStr[4:6]+oldStr[14]+oldStr[20:25]+oldStr[27:32]
    return newStr

def CHAR_TO_BOOL(Char):
    if Char == '0':
        return False
    elif Char == '1':
        return True
    elif Char == 'FALSE':
        return False
    elif Char == 'TRUE':
        return True
    else:
        return None
    
def STRING_TO_REAL(StringList):
    return string.atof(StringList)

def STRING_TO_INT(StringList):
    return string.atoi(StringList)
 
def STRINGLIST_TO_REALLIST(StringList):
    RealList = []
    if len(StringList)>=8:   # Value List exists
        for i in range(2,8):
            RealList.append(STRING_TO_REAL(StringList[i]))  
        return RealList
   
def STRINGLIST_TO_INTLIST(StringList):
    IntList = []
    if len(StringList)>=8:   # Value List exists
        for i in range(2,8):
            IntList.append(STRING_TO_INT(StringList[i]))  
        return IntList
    
def STRINGLIST_TO_BOOLLIST(StringList):
    BoolList = []
    if len(StringList)>=8:
        for i in range(2,len(StringList)):
            BoolList.append(CHAR_TO_BOOL(StringList[i]))
        return BoolList
    

def PrintAxisState(StateDict):
    print '____________ARM STATE____________'
    pprint(StateDict)
    print '_________________________________\n'
    #    print '\n'


def Evaluate_AXIS_STATE(ReceiveDataList):
    BoolList = STRINGLIST_TO_BOOLLIST(ReceiveDataList)
    State ={}
    for i in range(0, len(AxisStateNames)):
        State[AxisStateNames[i]] = BoolList[i]
    return State
        
        
#def Evaluate_GripperState(ReceiveDataList):
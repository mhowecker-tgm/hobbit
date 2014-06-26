'''
Created on 12.07.2013

@author: hofmanns
'''

import socket
import ArmControllerFunctions
import math
import time


class ArmClientFunctions():
    def __init__(self,Host):
        #self.Host = '169.254.141.97'
        self.Host = Host
        self.Port = 5010
        
        self.csocket = self.OpenSocket()
    
    def OpenSocket(self):
        csocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        csocket.settimeout(5.0)
        csocket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            csocket.connect((self.Host, self.Port))
            
        except socket.error, e:
            print 'XPC Client, Open Socket: ', e
            #ArmControllerFunctions.PublishToROS(e)
            csocket.close()
            exit()
        return csocket
    
    def SendData(self,csocket,datastring):
        try:
            csocket.send(datastring)
        except socket.error, e:
            print 'XPC Client, Send Data: ', e
            #ArmControllerFunctions.PublishToROS(e)
            csocket.close()
            exit()
        
    def ReceiveData(self,csocket):
        try:
            res = csocket.recv(1024)
        except socket.error, e:
            print 'XPC Client, Receive Data: ', e
            #ArmControllerFunctions.PublishToROS(e)
            csocket.close()
            exit()
        return res
    
    def SendTCP(self,datastring):
        #csocket=self.OpenSocket()        #Open Socket
        self.SendData(self.csocket,datastring)      #Sending Data
        #t01 = time.clock()
        recv = self.ReceiveData(self.csocket) #Get the received Data
        #print 'Elapsed Time to receive data: ', (time.clock()-t01)
        ReceiveDataList = ArmControllerFunctions.CutString(recv)
        #csocket.close()             #Close Socket
        return ReceiveDataList
    
    def SendInterpolationPositions(self,):
        #csocket=self.OpenSocket()        #Open Socket
        # Test Array
        Array = []
        t0 = time.clock()
        for i in range(50):
            t = time.clock()-t0
            value1 = round( (90-math.sin(2*math.pi*0.05*t)*90) ,2 )
            value2 = round( (80-math.sin(2*math.pi*0.05*t)*90), 2 )
            value4 = round( (math.sin(2*math.pi*0.05*t)*90), 2 )
            value5 = round( (110-math.sin(2*math.pi*0.05*t)*90), 2 )
            #print value2
            #Array.append('SET;PositionsForInterpolation;{Val1}/{Val2}/70/0/110/0/;'.format(Val1=value1, Val2=value2) )
            Array.append('SET;PositionsForInterpolation;{Val1}/{Val2}/70/{Val4}/{Val5}/0/;'.format(Val1=value1, Val2=value2, Val4=value4, Val5=value5) )
            self.SendData(self.csocket,Array[i])      #Sending Data
            #t01 = time.clock()
            recv = self.ReceiveData(self.csocket)    #Get the received Data
            ReceiveDataList = ArmControllerFunctions.CutString(recv)
            if ReceiveDataList[1] == 'POSITION_BUFFER_FULL':
                print ReceiveDataList
                return ReceiveDataList
                break
        if i==49:
            #print 'Elapsed Time to send positions: ', (time.clock()-t0)
            self.SendData(self.csocket,'SET;PositionsForInterpolationReady;')      #Sending Data
            recv = self.ReceiveData(self.csocket)    #Get the received Data
            ReceiveDataList = ArmControllerFunctions.CutString(recv)
        #csocket.close()             #Close Socket  
        print ReceiveDataList      
        return ReceiveDataList
                
    
    def CheckReceive(self,ReceiveDataList):
        if ReceiveDataList[0]=='ArmState':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.Evaluate_AXIS_STATE(ReceiveDataList)
                ArmControllerFunctions.PrintAxisState(State)
        elif ReceiveDataList[0] == 'GripperIsClosed':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ActualPos':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.STRINGLIST_TO_REALLIST(ReceiveDataList)
        elif ReceiveDataList[0] == 'ArmIsMoving':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmHasError':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmIsEnabled':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmIsHomed':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmHasStopped':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmInPositionArea':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmInTargetPos':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmAtHomePos':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmAtLearningPos':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmAtTrayPos':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmAtTurntablePos':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmAtCCWPos':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmAtCWPos':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmAtPreGraspFromFloorPos':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmAtPreGraspFromTablePos':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmSoftLimitMax':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
        elif ReceiveDataList[0] == 'ArmSoftLimitMin':
            if ReceiveDataList[1]=='COMMAND_OK':
                State = ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])            
        else:
            State = None
        return State
    
        
    #_____________________SET Commands_____________________#
    
    def SetAbsolutePos(self,Value1, Value2, Value3, Value4, Value5, Value6):
        buf = 'SET;AbsolutePos;{Val1};{Val2};{Val3};{Val4};{Val5};{Val6};'.format(Val1=round(Value1,2),Val2=round(Value2,2),Val3=round(Value3,2),Val4=round(Value4,2),Val5=round(Value5,2),Val6=round(Value6,2))
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetStartInterpolation(self):
        buf = 'SET;StartInterpolation;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetPositionsForInterpolation(self,Value1, Value2, Value3, Value4, Value5, Value6):
        #self.SendInterpolationPositions()
        buf = 'SET;PositionsForInterpolation;{Val1}/{Val2}/{Val3}/{Val4}/{Val5}/{Val6};'.format(Val1=Value1,Val2=Value2,Val3=Value3,Val4=Value4,Val5=Value5,Val6=Value6)
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetPositionsForInterpolationReady(self):
        buf = 'SET;PositionsForInterpolationReady;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetStartMove(self, Velocity):
        # Velocity in [deg/s]; Maximum Velocity = 20
        buf = 'SET;StartMove;{Val1};'.format(Val1=round(Velocity,2))
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetMoveToHomePos(self):
        buf = 'SET;MoveToHomePos;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetMoveToLearningPos(self):
        buf = 'SET;MoveToLearningPos;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList    
    
    def SetMoveToTrayPos(self):
        buf = 'SET;MoveToTrayPos;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList 
    
    def SetMoveToPreGraspFromFloorPos(self):
        buf = 'SET;MoveToPreGraspFromFloorPos;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetMoveToPreGraspFromTablePos(self):
        buf = 'SET;MoveToPreGraspFromTablePos;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList    
    
    def SetStoreTurntable(self):
        buf = 'SET;StoreTurntable;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList     
    
    def SetTurnTurntableCW(self):
        buf = 'SET;TurnTurntableCW;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetTurnTurntableCCW(self):
        buf = 'SET;TurnTurntableCCW;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetStartArmReference(self):
        buf = 'SET;StartArmReference;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList    
    
    def SetDisableArm(self):
        buf = 'SET;DisableArm;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetEnableArm(self):
        buf = 'SET;EnableArm;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetOpenGripper(self):
        buf = 'SET;OpenGripper;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetCloseGripper(self):
        buf = 'SET;CloseGripper;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetResetArm(self):
        buf = 'SET;ResetArm;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    def SetStopArmMove(self):
        buf = 'SET;StopArmMove;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList

    def SetClearPosBuffer(self):
        buf = 'SET;ClearPosBuffer;'
        ReceiveDataList = self.SendTCP(buf)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ReceiveDataList
    
    
    
    #_____________________GET Commands_____________________#
    def GetArmState(self):
        buf = 'GET;ArmState;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    def GetActualPosition(self):
        buf = 'GET;ActualPos;'
        ReceiveDataList = self.SendTCP(buf)
        ActPos = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return ActPos
    
    def GetGripperIsClosed(self):
        buf = 'GET;GripperIsClosed;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status    
    
    def GetArmIsMoving(self):
        buf = 'GET;ArmIsMoving;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    def GetArmHasError(self):
        buf = 'GET;ArmHasError;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    def GetArmHasStopped(self):
        buf = 'GET;ArmHasStopped;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    def GetArmIsEnabled(self):
        buf = 'GET;ArmIsEnabled;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status   
    
    def GetArmIsHomed(self):
        buf = 'GET;ArmIsHomed;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    def GetArmInPositionArea(self):
        buf = 'GET;ArmInPositionArea;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    def GetArmInTargetPos(self):
        buf = 'GET;ArmInTargetPos;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    def GetArmAtHomePos(self):
        buf = 'GET;ArmAtHomePos;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    def GetArmAtLearningPos(self):
        buf = 'GET;ArmAtLearningPos;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status

    def GetArmAtTrayPos(self):
        buf = 'GET;ArmAtTrayPos;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status

    def GetArmAtTurntablePos(self):
        buf = 'GET;ArmAtTurntablePos;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    def GetArmAtPreGraspFromFloorPos(self):
        buf = 'GET;ArmAtPreGraspFromFloorPos;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
 
    def GetTurntableAtCCWPos(self):
        buf = 'GET;ArmAtCCWPos;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    def GetTurntableAtCWPos(self):
        buf = 'GET;ArmAtCWPos;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    '''       
    def GetArmAtGraspFromFloorPos(self):
        buf = 'GET;ArmAtGraspFromFloorPos;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    '''
        
    def GetArmSoftLimitMax(self):
        buf = 'GET;ArmSoftLimitMax;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status
    
    def GetArmSoftLimitMin(self):
        buf = 'GET;ArmSoftLimitMin;'
        ReceiveDataList = self.SendTCP(buf)
        Status = self.CheckReceive(ReceiveDataList)
        #ArmControllerFunctions.PublishToROS(ReceiveDataList)
        return Status


#ArmClient = ArmClientFunctions()

#ArmClient.SetPositionsForInterpolation(0, 0, 0, 0, 0, 0)

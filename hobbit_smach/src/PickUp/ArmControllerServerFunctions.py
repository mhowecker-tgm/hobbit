'''
Created on 19.09.2013

@author: hofmanns
'''
import socket
import ArmControllerFunctions

#HOST = ''
#PORT = 5020

class ServerApp():
    def __init__(self):
        self.Port = 5020
        self.Addr = ''
        
    def ServerProgram(self):
        csocket = self.OpenServerSocket()
        # Publish to ROS
        try:
            while True:
                try:
                    c, self.Addr = csocket.accept()
                    c.setblocking(1)
                    
                except socket.error, e:
                    print 'XPC Server:',e
                    # Publish to Ros?
                    exit()
                    
                while True:
                    #self.SetServerConnectedEvent(True)
                    data = c.recv(1024)
                    #Allocate Lock, or event?
                    ReceiveDataList = self.CheckReceiveData(data,c)
                    if not data:
                        break
                    
        finally:    
            csocket.close()   
            # Publish to ROS
                 
    def OpenServerSocket(self):
        csocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        csocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            csocket.bind(('',self.Port))
            csocket.listen(1)
            
        except socket.error, e:
            print e
            try:
                csocket.close()
                # Publish to ROS
                print 'Socket closed'
            except socket.error, e: 
                print e
                print 'Socket was not closed'
            exit()
        return csocket
    
    
    def SendEcho(self, data,s):
        # Sending Echo when receiving data:
        try:
            s.send(data)
            Sendstatus = True
        except s.error, e:
            print e
            # Publish to ROS
            Sendstatus = False
        return Sendstatus
    
    
    def CheckReceiveData(self,data,s):
        ReceiveDataList = ArmControllerFunctions.CutString(data)
        Status = {}
        if ReceiveDataList[0]=='STATE':
            for i in range(0, len(ArmControllerFunctions.AxisStateNames)):
                if ReceiveDataList[1]==ArmControllerFunctions.AxisStateNames[i]:
                    if len(ReceiveDataList)<2:
                        self.SendEcho('{Var};VALUE_MISSING'.format(Var=ArmControllerFunctions.AxisStateNames[i]),s)
                        print '{Var}: VALUE_MISSING'.format(Var=ArmControllerFunctions.AxisStateNames[i])
                    else:
                        self.SendEcho('{Var};COMMAND_OK;'.format(Var=ArmControllerFunctions.AxisStateNames[i]),s)
                        Status[ArmControllerFunctions.AxisStateNames[i]]=ArmControllerFunctions.CHAR_TO_BOOL(ReceiveDataList[2])
                        # Publish to ROS
                        print Status
        return Status            
    
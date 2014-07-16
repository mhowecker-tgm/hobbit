#!/usr/bin/env python
'''
Created on 19.09.2013
adapted to ROS by markus.bajones@tuwien.ac.at 16.07.20014

@author: hofmanns
'''

NAME = 'arm_monitor'
PKG = 'hobbit_smach'
import socket
import hobbit_smach.ArmControllerFunctions as ArmControllerFunctions
import roslib
roslib.load_manifest(PKG)
import rospy
from hobbit_msgs.msg import Event

# HOST = ''
# PORT = 5020


class ServerApp():
    def __init__(self):
        self.Port = 5020
        self.Addr = ''
        self.pub = rospy.Publisher('/Event', Event, queue_size=10)
        self.help_msg = Event()
        self.help_msg.event = 'E_HELP'

    def ServerProgram(self):
        csocket = self.OpenServerSocket()
        # Publish to ROS
        try:
            while True:
                self.pub.publish(self.help_msg)
                try:
                    print('open socket')
                    c, self.Addr = csocket.accept()
                    c.setblocking(1)

                except socket.error, e:
                    print 'XPC Server:', e
                    # Publish to Ros?
                    exit()

                #while not rospy.is_shutdown():
                while True:
                    # self.SetServerConnectedEvent(True)
                    data = c.recv(1024)
                    # Allocate Lock, or event?
                    ReceiveDataList = self.CheckReceiveData(data, c)
                    if not data:
                        break

        finally:
            csocket.close()
            # Publish to ROS
            print('Socket is closed')

    def OpenServerSocket(self):
        csocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        csocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            csocket.bind(('', self.Port))
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

    def SendEcho(self, data, s):
        # Sending Echo when receiving data:
        try:
            s.send(data)
            Sendstatus = True
        except s.error, e:
            print e
            # Publish to ROS
            Sendstatus = False
        return Sendstatus

    def CheckReceiveData(self, data, s):
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
                        rospy.logdebug(str(Status))
                        try:
                            if Status['EmergencyPressed']:
                                self.help_msg.header.stamp = rospy.get_rostime()
                                self.pub.publish(self.help_msg)
                        except:
                            pass
        return Status

if __name__ == '__main__':
    rospy.init_node(NAME)
    rospy.loginfo(NAME+' started')
    server = ServerApp()
    server.ServerProgram()

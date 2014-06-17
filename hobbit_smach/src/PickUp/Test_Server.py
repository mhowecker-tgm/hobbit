#!/usr/bin/python

import roslib; roslib.load_manifest('hobbit_smach')
import rospy
from ArmControllerServerFunctions import ServerApp

#Created on 07.01.2014
#@author: HofmannS



ArmServer = ServerApp()     # Create an Instance of the TCP/IP Server
ArmServer.ServerProgram()   # Start the Server

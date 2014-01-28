#!/usr/bin/python

PKG = 'sos_button'
# HOBBIT Interface to the SOS button
# AAT/PM 2013
# expects a serial port ttySOS by udev
# there are 2 versions of adapters to check

import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import signal
import os
import ast
import serial
import time
import commands
from std_msgs.msg import String, Header
from hobbit_msgs.msg import Command, Status, Event, Parameter

#### this needs PYTHONPATH set to where hobbitlib.py is ####
#sys.path.append(os.path.abspath(os.path.dirname(os.path.abspath(sys.argv[0]))+'/../../Coordinator/src'))
#import hobbitlib
#### for get/set param

def signal_handler(signal, frame):
        print PKG, ': You pressed Ctrl+C!'
        sys.exit(0)

def main(args):        
    print PKG,": Checking USB serial ports attached:"
    x=commands.getoutput("ls -1l /dev/serial/by-id | grep usb")
    fx=x.split("\n")
    for x in fx:
     xx=x.split('usb-')   
     print " ",xx[1]
    type3V3 = True #False
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('SosButton', anonymous=False)
    pubE = rospy.Publisher("/Event", Event)
 
    try:
       print PKG,": opening ttySOS..."
       sp=serial.Serial('/dev/ttySOS',9600) 
       print sp.portstr, "opened OK"
    except:
       print PKG,": hoppla, exiting!"
       return

    sp.setDTR(True)
    sp.setRTS(True)
    sp.setRTS(False)
    if type3V3:
      sp.setRTS(True)
    x= sp.getCTS() #check for reaction of v1 or v2
    if x:
      sp.setRTS(False)
      type3V3 = False
      print PKG,": detected v1 button - OK"
    else:
      print PKG,": detected v2 button - OK"

    while 1:
      x= sp.getCTS()
      #print "CTS", x, type3V3
      if x == type3V3:
            if type3V3:
              sp.setRTS(False)
            else:
              sp.setRTS(True)
            print PKG,": SOS pressed"
            e = Event()
            e.header = Header()
            e.event = 'E_SOS'
            e.sessionID = '0'
            e.confidence = 1
            pubE.publish(e)

            time.sleep(3)
           
            if type3V3:
              sp.setRTS(True)
            else:
              sp.setRTS(False)
            while sp.getCTS() == type3V3:
              time.sleep(1)
      else:
       time.sleep(.1)

if __name__ == "__main__":        
    main(sys.argv)

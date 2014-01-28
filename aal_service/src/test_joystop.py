#!/usr/bin/python

## stop on left #2 button by AAT PM 2013
## based on:
## walter wohlkinger
## first version: 27.04.2011

PKG = 'AalService'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import os
import numpy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Joy

pubS = rospy.Publisher("/GoToPoseCmd", String)
oneclick = True
havestop = False

def joyCallback(msg):
    global oneclick
    global havestop

    if msg.buttons[8] == 1:     #2-botton left
        #oneclick = False
        e = "Stop"
        havestop = True
        pubS.publish(e)
        try:
         if not os.path.isfile('/tmp/stopfile'): 
          print "create STOPFILE"
          open('/tmp/stopfile', 'a').close()
        except:
          print "except"
    else:
       if havestop:
        try:
         if os.path.isfile('/tmp/stopfile'): 
          print "delete STOPFILE"
          os.remove('/tmp/stopfile')
        except:
          print "except"
        havestop = False

def main(args):        
    rospy.init_node('Joystop', anonymous=True)
    
    sub = rospy.Subscriber("/joy", Joy, joyCallback )
    rospy.spin()
    
    
if __name__ == "__main__":        
    main(sys.argv)

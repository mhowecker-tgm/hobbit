#!/usr/bin/python

## david fischinger
## first version: 19.02.2015

PKG = 'joy2twist'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
from std_msgs.msg import String, Header
from std_srvs.srv import Empty
#from mira_msgs.srv import ResetMotorStop
#from sensor_msgs.msg import Joy
#from hobbit_msgs.msg import Command, Status, Event, Parameter
from hobbit_msgs import MMUIInterface as MMUI
from hobbit_msgs.srv import LeftJoyStickPressed

#pubC = rospy.Publisher("/Command", Command) 
#pubE = rospy.Publisher("/Event", Event)    
pubEmo= rospy.Publisher("/head/emo", String)

   
#srvFacePause = rospy.ServiceProxy('/face_detection/pause', Empty)
#srvFaceResume = rospy.ServiceProxy('/face_detection/resume', Empty)
 

mmui = MMUI.MMUIInterface()



#!/usr/bin/env python

#from beginner_tutorials.srv import *
import rospy

def execute_left_joystick_pressed(req):
    #do stuff
    #print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    print "==> execute_left_joystick_pressed() started"
    
    # speek something
    mmui.showMMUI_Info("Are you Sarah Connor?")
    
    #show emotions
    pubEmo.publish(String("REDEYE"))
    rospy.sleep(4)
    pubEmo.publish(String("NEUTRAL"))
    
    return LeftJoyStickPressed(True)

def joybuttongimmicks_services():
    rospy.init_node('joybuttongimmicks')
    s = rospy.Service('left_joy_stick_pressed', LeftJoyStickPressed, execute_left_joystick_pressed)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    joybuttongimmicks_services()
          

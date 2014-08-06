#!/usr/bin/python

## walter wohlkinger
## first version: 27.04.2011

PKG = 'joy2twist'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import numpy
import serial
from std_msgs.msg import String, Header
from sensor_msgs.msg import Joy
from hobbit_msgs.msg import Command, Status, Event, Parameter

# bajo: nobody is listening to the ActionSequence topic, therefore it does not
# make any sense to still publish any data onto it. 
# Commands should go to /Command and Events to /Event
#pubA = rospy.Publisher("/ActionSequence", Command)
pubC = rospy.Publisher("/Command", Command) 
pubE = rospy.Publisher("/Event", Event)    
oneclick = True

def joyCallback(msg):
    global oneclick
    if numpy.sum(numpy.asarray(msg.buttons)) < 1.0:
        #print 'True'
        oneclick = True
        
    if oneclick == False:
        #print 'returning'
        return
    

    if msg.buttons[12] == 1:    #triangle right
        e = Event()
        e.header = Header()
        e.event = "G_YES"
        e.sessionID = 'gesture_recognized'
        e.confidence = 1
        pubE.publish(e)
        print e.event
        oneclick = False


    if msg.buttons[11] == 1:    #1-button right
        e = Event()
        e.header = Header()
        e.event = "G_NO"
        e.sessionID = 'gesture_recognized'
        e.confidence = 1
        pubE.publish(e)
        print e.event
        oneclick = False


    if msg.buttons[10] == 1:    #1-button left
        a = Command()   #action sequenz
        a.header = Header()
        a.command = "C_CALLHOBBIT"
        a.sessionID = 'abc'
        pubC.publish(a)
        print a.command
        oneclick = False

    if msg.buttons[9] == 1:     #2-botton right
        e = Event()
        e.header = Header()
        e.event = "G_FALL"
        e.sessionID = 'gesture_recognized'
        e.confidence = 1
        pubE.publish(e)
        print e.event
        oneclick = False
        
    if msg.buttons[15] == 1:
        e = Command()
        e.header = Header()
        e.command = "F_SETTINGS"
        e.sessionID = '0'
        pubC.publish(e)
        print e.command
        oneclick = False

    if msg.buttons[13] == 1:
        e = Command()
        e.header = Header()
        e.command = "C_STOP"
        e.sessionID = '0'
        pubC.publish(e)
        print e.command
        oneclick = False

    if msg.buttons[14] == 1:
        e = Command()
        e.header = Header()
        e.command = "C_SLEEP"
        e.sessionID = '0'
        pubC.publish(e)
        print e.command
        oneclick = False

    if msg.buttons[7] == 1:  #left
        e = Command()
        e.header = Header()
        e.command = "F_ASR_OFF"
        e.sessionID = '0'
        pubC.publish(e)
        print e.command
        oneclick = False

    if msg.buttons[5] == 1:  #right
        e = Command()
        e.header = Header()
        e.command = "F_ASR_ON"
        e.sessionID = '0'
        pubC.publish(e)
        print e.command
        oneclick = False

    if msg.buttons[4] == 1:  #up
        e = Command()
        e.header = Header()
        e.command = "F_GESTURE_ON"
        e.sessionID = '0'
        pubC.publish(e)
        print e.command
        oneclick = False

    if msg.buttons[6] == 1:  #down
        e = Command()
        e.header = Header()
        e.command = "F_GESTURE_OFF"
        e.sessionID = '0'
        pubC.publish(e)
        print e.command
        oneclick = False




def main(args):        
    rospy.init_node('Joybutton', anonymous=False)
    
    sub = rospy.Subscriber("/joy", Joy, joyCallback )
    rospy.spin()
    
    
if __name__ == "__main__":        
    main(sys.argv)

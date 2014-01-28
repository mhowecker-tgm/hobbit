#!/usr/bin/python

## periodic blink/battery monitor by AAT PM 2013

PKG = 'AalService'
import roslib; roslib.load_manifest(PKG)
import rospy
import numpy
import serial
import time, os, sys, sched
from random import randint
import random
import signal
from std_msgs.msg import String, Header, Float64
from sensor_msgs.msg import Joy
from HobbitMsgs.msg import Command, Status, Event, Parameter
from threading import Timer
pubE2 = rospy.Publisher("/Hobbit/Emoticon2Show", String)
pubE = rospy.Publisher("/Event", Event)

global m
global state
state = -2
voltage = 0
global last

last = "EMO_NEUTRAL"

def hello():
        global m
        print "timer"
        if last == "EMO_NEUTRAL" or last == "EMO_BLINK":
         print "blink"
         pubE2.publish("EMO_BLINK")
         ti=5+randint(5,15)
         m = Timer(ti, hello)
         m.start()
         print "re-sched",ti

# the decisions need be made less jumpy - fixme!
def batCallback(msg):
  global state
  global voltage

  batt = msg.data
  if abs(batt-voltage) < 0.1:
   return

  voltage = batt

  if batt < 24:
                        if state == -1:
                           return
                        print msg.data
                        state = -1
                        e = Event()
                        e.event = "E_BATTERY_LEVEL_CRITICAL"
                        e.sessionID = "batterystateservice" + str(int(1000*random.random()))
                        pubE.publish(e)
                        print "E_BATTERY_LEVEL_CRITICAL"
                        return

  else:
    if batt >= 28.5:
                        if state == 1:
                                return
                        print msg.data
                        state = 1
                        e = Event()
                        e.event = "E_BATTERY_LEVEL_FULL"
                        e.sessionID = "batterystateservice" + str(int(1000*random.random()))
                        pubE.publish(e)
                        print "E_BATTERY_LEVEL_FULL"
                        return
    else:
                        if state == 0:
                                return
                        print msg.data
                        state = 0
                        e = Event()
                        e.event = "E_BATTERY_LEVEL_GOOD"
                        e.sessionID = "batterystateservice" + str(int(1000*random.random()))
                        pubE.publish(e)
                        print "E_BATTERY_LEVEL_GOOD"
                        return

def emoCallback(msg):
  global m
  global last
  print msg
  last = msg.data
  m.cancel()
  if last == "EMO_NEUTRAL" or last == "EMO_BLINK":
    ti = 5+randint(5,15)
    m = Timer(ti, hello)
    m.start()
    print "sched",ti

def signal_handler(signal, frame):
        global m
        m.cancel()
        print 'You pressed Ctrl+C!'
        sys.exit(0)
# end def signal_handler

def main(args):        
    global m
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Emoblink', anonymous=False)
    
    sub = rospy.Subscriber("/Hobbit/Emoticon", String, emoCallback )
    bat = rospy.Subscriber("/BatteryVoltage", Float64, batCallback )
    m = Timer(5+randint(5,15), hello)
    m.start()
    rospy.spin()
    
    
if __name__ == "__main__":        
    main(sys.argv)

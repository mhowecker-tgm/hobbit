#!/usr/bin/python

## walter wohlkinger
## first version: 27.04.2011

PKG = 'Head'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import time

import tf
import serial
from std_msgs.msg import String, Header

from Head.srv import HeadState
lastx = ""
lasty = ""
br = tf.TransformBroadcaster()


def tfmoveCallback(msg):
    global lastx
    global lasty
    global br
   
    
    if msg.data == "up" or msg.data == "down" or msg.data == "middle":
        lasty = msg.data
        
    if msg.data == "left" or msg.data == "right" or msg.data == "center":
        lastx = msg.data

    if lastx == "center" and lasty == "down":
        br.sendTransform((-0.010,
                           0.513,
                            1.079),
                         (0.694
                          ,-0.668, 
                          0.178,
                          0.199),
                         rospy.Time.now(),
                         "/hobbit",
                         "/topcamera_rgb_optical_frame")

#    if lastx == "right" and lasty == "down":
#        br.sendTransform((-0.0944817,
#                           0.543719,
#                            1.04987),
#                         (-0.247253
#                          ,0.939565, 
#                          -0.226823,
#                          -0.226823),
#                         rospy.Time.now(),
#                         "hobbit",
#                         "/topcamera_rgb_optical_frame")
        

    if lastx == "right" and lasty == "down":
        br.sendTransform((-0.13725,
                           0.557794,
                            1.03807),
                         (-0.2514
                          ,0.94008, 
                          -0.21544,
                          -0.0813494),
                         rospy.Time.now(),
                         "/hobbit",
                         "/topcamera_rgb_optical_frame")



def moveCallback(msg):
    print msg
    global pubHS

    
    if msg.data == "up":
        ser = serial.Serial('/dev/ttyACM0')
        ser.write(chr(0xAA))
        ser.flush()
        ser.write(chr(0x87)+chr(0x01)+chr(0x0a)+chr(0x00))
        ser.flush()
        ser.write(chr(0x84)+chr(0x01)+chr(104)+chr(52))
        ser.flush()
        return

    if msg.data == "down":
        ser = serial.Serial('/dev/ttyACM0')
        ser.write(chr(0xAA))
        ser.flush()
        ser.write(chr(0x87)+chr(0x01)+chr(0x0a)+chr(0x00))
        ser.flush()
        ser.write(chr(0x84)+chr(0x01)+chr(112)+chr(31))
        ser.flush()
        return

    if msg.data == "middle":
        ser = serial.Serial('/dev/ttyACM0')
        ser.write(chr(0xAA))
        ser.flush()
        ser.write(chr(0x87)+chr(0x01)+chr(0x0a)+chr(0x00))
        ser.flush()
        ser.write(chr(0x84)+chr(0x01)+chr(84)+chr(42))
        ser.flush()
        return

    if msg.data == "center":
        ser = serial.Serial('/dev/ttyACM0')
        ser.write(chr(0xAA))
        ser.flush()
        ser.write(chr(0x87)+chr(0x00)+chr(0x0a)+chr(0x00))
        ser.flush()
        ser.write(chr(0x84)+chr(0x00)+chr(32)+chr(47))
	ser.flush()
        return

    if msg.data == "left":
        ser = serial.Serial('/dev/ttyACM0')
        ser.write(chr(0xAA))
        ser.flush()
        ser.write(chr(0x87)+chr(0x00)+chr(0x0a)+chr(0x00))
        ser.flush()
        ser.write(chr(0x84)+chr(0x00)+chr(32)+chr(31))
        ser.flush()
        return

    if msg.data == "right":
        ser = serial.Serial('/dev/ttyACM0')
        ser.write(chr(0xAA))
        ser.flush()
        ser.write(chr(0x87)+chr(0x00)+chr(0x0a)+chr(0x00))
        ser.flush()
        ser.write(chr(0x84)+chr(0x00)+chr(16)+chr(61))
        ser.flush()
        return

    if msg.data == "turnoff":
        ser = serial.Serial('/dev/ttyACM0')
        ser.write(chr(0xAA))
        ser.flush()
        ser.write(chr(0x87)+chr(0x00)+chr(0x0a)+chr(0x00))
        ser.flush()
        ser.write(chr(0x84)+chr(0x00)+chr(0)+chr(0))
        ser.flush()
        ser.write(chr(0x87)+chr(0x01)+chr(0x0a)+chr(0x00))
        ser.flush()
        ser.write(chr(0x84)+chr(0x01)+chr(0)+chr(0))
        ser.flush()
        return

#    if msg.data == "myNormalizedInit":
#        ser = serial.Serial('/dev/ttyACM0')
#        ser.write(chr(0xAA))
#        ser.flush()
#        ser.write(chr(0x87)+chr(0x00)+chr(0x0a)+chr(0x00))
#        ser.flush()
#        #ser.write(chr(0x84)+chr(0x01)+chr(104)+chr(52))
#        ser.write(chr(0x87)+chr(0x01)+chr(0x0a)+chr(0x00))
#	ser.flush()
#        return


def headStatus(req):
    global lastx
    global lasty
    return HeadState(lastx, lasty)

def main(args):        
    #value = 1365
    #highbits,lowbits = divmod(value,32)
    #print lowbits << 2, highbits
    
    rospy.init_node('HeadMove', anonymous=False)
    sub = rospy.Subscriber("/HeadMove", String, moveCallback )
    tfsub = rospy.Subscriber("/HeadMove", String, tfmoveCallback )
    pub = rospy.Publisher("/HeadMove", String)
    s = rospy.Service('/HeadStatus', HeadState, headStatus)
    
    rospy.sleep(2)
#    pub.publish("myNormalizedInit")
#    rospy.sleep(1)
    pub.publish("down")
    pub.publish("center")
    rospy.spin()
    
    
if __name__ == "__main__":        
    main(sys.argv)

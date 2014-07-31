#!/usr/bin/env python
PKG = 'blue_owlpose'
import roslib; roslib.load_manifest(PKG)
import rospy
import herkulex
import atexit
import tf
import math
from time import sleep
import numpy as np
from sensor_msgs.msg import *
from geometry_msgs.msg import Quaternion
from blue_owlpose.srv import *

pitch = 0
yaw = 0
roll = 0
br = tf.TransformBroadcaster()
#base_tf = "hobbit"
#head_tf = "topcamera_rgb_optical_frame"
base_tf = "hobbit_neck_dynamic"
head_tf = "hobbit_neck"
#base_tf = "hobbit_head"
#head_tf = "headcam_depth_optical_frame"

def set_head_orientation(msg):
	print "Move to",msg.data
	if msg.data == "up_center":
		herkulex.setAngles(yaw=-20, pitch=0, roll=0, playtime=170)	        
		#br.sendTransform((0, 0, 0), (-0.20 ,0.0, 0.0, 1), rospy.Time.now(), base_tf,head_tf)
	
	if msg.data == "center_center":
		herkulex.setAngles(yaw=0, pitch=0, roll=0, playtime=170)
		#br.sendTransform((0, 0, 0), (-0.0 ,0.0, 0.0, 1), rospy.Time.now(), base_tf,head_tf)

	if msg.data == "down_center":
		herkulex.setAngles(yaw=35, pitch=0, roll=0, playtime=170)
		#br.sendTransform((0, 0, 0), (0.35 ,0.0, 0.0, 1), rospy.Time.now(), base_tf,head_tf)

	if msg.data == "littledown_center":
		herkulex.setAngles(yaw=20, pitch=0, roll=0, playtime=170)
		#br.sendTransform((0, 0, 0), (0.20 ,0.0, 0.0, 1), rospy.Time.now(), base_tf,head_tf)

	if msg.data == "center_right":
		herkulex.setAngles(yaw=0, pitch=-90, roll=0, playtime=200)
		#br.sendTransform((0, 0, 0), (0.0 ,-0.90, 0.0, 1), rospy.Time.now(), base_tf,head_tf)

	if msg.data == "down_right":
		herkulex.setAngles(yaw=50, pitch=-90, roll=0, playtime=200)
		#br.sendTransform((0, 0, 0), (0.50 ,-0.90, 0.0, 1), rospy.Time.now(), base_tf,head_tf)

	if msg.data == "to_turntable":
		herkulex.setAngles(yaw=40, pitch=-61, roll=0, playtime=200)
		#br.sendTransform((0, 0, 0), (0.4 ,-0.61, 0.0, 1), rospy.Time.now(), base_tf,head_tf)

	if msg.data == "to_grasp":
		herkulex.setAngles(yaw=55, pitch=-50, roll=0, playtime=200)
		#br.sendTransform((0, 0, 0), (0.45 ,-0.70, 0.0, 1), rospy.Time.now(), base_tf,head_tf)

	if msg.data == "center_left":
		herkulex.setAngles(yaw=0, pitch=90, roll=0, playtime=200)
		#br.sendTransform((0, 0, 0), (0.0 ,0.90, 0.0, 1), rospy.Time.now(), base_tf,head_tf)

	if msg.data == "down_left":
		herkulex.setAngles(yaw=50, pitch=90, roll=0, playtime=200)
		#br.sendTransform((0, 0, 0), (0.50 ,0.90, 0.0, 1), rospy.Time.now(), base_tf,head_tf)

	if msg.data == "up_left":
		herkulex.setAngles(yaw=-20, pitch=90, roll=0, playtime=200)
		#br.sendTransform((0, 0, 0), (-0.20 ,0.90, 0.0, 1), rospy.Time.now(), base_tf,head_tf)

	if msg.data == "up_right":
		herkulex.setAngles(yaw=-20, pitch=-90, roll=0, playtime=200)        
		#br.sendTransform((0, 0, 0), (-0.20 ,-0.90, 0.0, 1), rospy.Time.now(), base_tf,head_tf)

	rospy.sleep(0.5)
	
def command(msg):
	if msg.data == "restart":
		herkulex.resetServos()
	        print "Servo Reset"
	if msg.data == "torque":
		herkulex.setTorque()
	        print "Torque Set"
	if msg.data == "startup":
		herkulex.resetServos()
                print "Servo Reset"
		rospy.sleep(0.5)
		herkulex.setTorque()
                print "Torque Set"
		rospy.sleep(0.5)
		herkulex.setAngles(yaw=0, pitch=0, roll=0, playtime=150)
		print "Move to center_center"
			
def init():
	#Initialize Rosnode:
	rospy.init_node('owlpose')
	print "Initialized"
	#rospy.sleep(3)
	#Initialize Servos:
	#herkulex.resetServos()
	#print "Servo Reset"
	#herkulex.setTorque()
	#print "Torque Set"
	#herkulex.setAngles(yaw=0, pitch=0, roll=0, playtime=150)
	#print "Move to center_center"
	
	#Subscriber for Movements:
	rospy.Subscriber("/head/move", std_msgs.msg.String, set_head_orientation)
	rospy.Subscriber("/head/cmd", std_msgs.msg.String, command)
	
	#Send geometry/tf constantly with 5hz
	r = rospy.Rate(5) #5hz
	while not rospy.is_shutdown():
		angles = herkulex.getAngles()
		br.sendTransform((0, 0, 0), (0.0, angles[1]/100, angles[0]/100, 1), rospy.Time.now(), base_tf,head_tf)
		r.sleep()

def shutdown():
	herkulex.setAngles(yaw=0, pitch=0, roll=0, playtime=150)

if __name__ == '__main__':
	try:
		init()
		atexit.register(shutdown)
	except rospy.ROSInterruptException:
		pass

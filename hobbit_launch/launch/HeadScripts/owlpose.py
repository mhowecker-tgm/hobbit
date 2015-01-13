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

min_lr_angle_deg = -90
max_lr_angle_deg = 90

min_ud_angle_deg = -26
max_ud_angle_deg = 66

pitch = 0	#(wrongly defined as left/right movement)
yaw = 0	#wrongly defined as up/down movement)
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
		herkulex.setAngles(yaw=62, pitch=-65, roll=0, playtime=200)
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

	try:
		if (msg.data[0]) == "a":	#absolut values (in degree) for head servos
			print "==> set absolut values"
			input = msg.data.split()
			lr_angle = int(input[1])
			ud_angle = int(input[2])
			if ( angleLimitsOK(lr_angle, ud_angle) ):
				herkulex.setAngles(yaw=ud_angle, pitch=lr_angle, roll=0, playtime=200)  
			
		if (msg.data[0]) in ("r", "l", "u", "d"):	#incremental head move
			print "==> set relative values"
			lr_shift = 0
			ud_shift = 0
			currentAngles = herkulex.getAngles()
			print "currentAngles:", currentAngles

			if msg.data[0] == "r":
				lr_shift = -1	#default value for move head right
				if (len(msg.data) > 1) and (int(msg.data[1]) > 0) and (int(msg.data[1]) < 10):
					lr_shift = -int(msg.data[1])
			if msg.data[0] == "l":
				lr_shift = 1	#default value for moving head left
				if (len(msg.data) > 1) and (int(msg.data[1]) > 0) and (int(msg.data[1]) < 10):
					lr_shift = int(msg.data[1])	
					 
			if msg.data[0] == "u":
				ud_shift = -1	#default value for move head up
				if (len(msg.data) > 1) and (int(msg.data[1]) > 0) and (int(msg.data[1]) < 10):
					ud_shift = -int(msg.data[1])	
			if msg.data[0] == "d":
				ud_shift = 1	#default value for moving head left
				if (len(msg.data) > 1) and (int(msg.data[1]) > 0) and (int(msg.data[1]) < 10):
					ud_shift = int(msg.data[1])	
			if (len(msg.data) > 3):
				if msg.data[3] == "r":
					lr_shift = -1	#default value for move head right
					if (int(msg.data[4]) > 0) and (int(msg.data[4]) < 10):
						lr_shift = -int(msg.data[4])
				if msg.data[3] == "l":
					lr_shift = 1	#default value for moving head left
					if (int(msg.data[4]) > 0) and (int(msg.data[4]) < 10):
						lr_shift = int(msg.data[4])	
					 
				if msg.data[3] == "u":
					ud_shift = -1	#default value for move head up
					if (int(msg.data[4]) > 0) and (int(msg.data[4]) < 10):
						ud_shift = -int(msg.data[4])	
				if msg.data[3] == "d":
					ud_shift = 1	#default value for moving head left
					if (int(msg.data[4]) > 0) and (int(msg.data[4]) < 10):
						ud_shift = int(msg.data[4])	
					 				 
			print "lr_shift: ", lr_shift
                        print "ud_shift: ", ud_shift
			lr_angle = currentAngles[0] + lr_shift
			ud_angle = currentAngles[1] + ud_shift
			if ( angleLimitsOK(lr_angle, ud_angle) ):
				print "6 set yaw=: ", ud_angle
                                print "6 set pitch=: ", lr_angle
                                herkulex.setAngles(yaw=ud_angle, pitch=lr_angle, roll=0, playtime=200)
				rospy.sleep(2.0)
			
	except:
		print "===============================================> owlpose.py: ERROR during variable setting of HEAD"

	rospy.sleep(0.5)
	
#checks if left/right and up/down angle are in limits
def angleLimitsOK(lr_angle, ud_angle):
	print "lr_angle: ", lr_angle
	print "ud_angle: ", ud_angle
	if (lr_angle >= min_lr_angle_deg) and (lr_angle <= max_lr_angle_deg) and (ud_angle >= min_ud_angle_deg) and (ud_angle <= max_ud_angle_deg):
		return True
	else:
		return False
	
	
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
	rospy.Subscriber("/head/move/incremental", std_msgs.msg.String, set_head_orientation, queue_size=1)
	rospy.Subscriber("/head/cmd", std_msgs.msg.String, command)
	
	#Send geometry/tf constantly with 5hz
	r = rospy.Rate(5) #5hz
	while not rospy.is_shutdown():
		angles = herkulex.getAngles()
		br.sendTransform( (0,0,0), tf.transformations.quaternion_from_euler(0, angles[1]/180.0*math.pi, angles[0]/180.0*math.pi), rospy.Time.now(), base_tf, head_tf)
		r.sleep()

def shutdown():
	herkulex.setAngles(yaw=0, pitch=0, roll=0, playtime=150)

if __name__ == '__main__':
	try:
		init()
		atexit.register(shutdown)
	except rospy.ROSInterruptException:
		pass

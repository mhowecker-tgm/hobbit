#!/usr/bin/env python
# ROS node to access the Dynamixel servos in the neck.
# Also can take IMU pitch angles to replace the angles returned by the pitch servo,
# if these should not be accurate.
# NOTE: the Dynamixel RX-28 servos used in Hobbit have a resolution of 0.29 deg
# NOTE: This uses a lot of global variables which are being accessed from different parts
# of the code WITHOUT a mutex or something. So there could be race conditions lurking.
#
# @author Tobias Ferner (some), Andreas Baldinger (some more), Michael Zillich (final)
# @date April 2015

PKG = 'blue_owlpose'
import roslib; roslib.load_manifest(PKG)
import rospy
import atexit
import tf
import math
from time import sleep
import numpy as np
from sensor_msgs.msg import *
from geometry_msgs.msg import Quaternion
from dynamixel_msgs.msg import *
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from hobbit_msgs.srv import HeadSleep

# angle limits, in degrees
# mechanical stops are
# -90
# 90
# -28.075
# 63.598
min_lr_angle_deg = -90
max_lr_angle_deg = 90
min_ud_angle_deg = -22
max_ud_angle_deg = 61

# torque limit for dynamixel servos
# NOTE that typical load in the Hobbit neck is at most 0.5 for the tilt joint, but can also reach > 0.8
# if the joint is blocked
torque_limit = 0.9

# transforms from angles
br = tf.TransformBroadcaster()
base_tf = "hobbit_neck_dynamic"
head_tf = "hobbit_neck"

# pitch angle from IMU, in degrees
imuPitch = 0
# indicates whether the robot is currently moving, in which case IMU values
# are not valid
robotMoving = False
# yaw and pitch offset in degrees
pitch_offset = 0
yaw_offset = 0
# indicate whether we are using the IMU
haveImu = False
imu_pitch_offset = 0
# current yaw and pitch as read by the servos' sensors
current_yaw = 0
current_pitch = 0
# currently set goal positions for the servos
# NOTE: these will typically be different from the currently measured values, especially if the servo is
# under high load
current_yaw_as_set = 0
current_pitch_as_set = 0
yaw_temperature = 0
pitch_temperature = 0
# publishers to the dynamixel topics
pan_pub = None
tilt_pub = None
# indicates whether tilt motor is currently turned on or off (= in resting position)
motorsAreOn = True
# indicate whether we want to debug print current angles
debugPrintAngles = False

# time when last command was sent
whenLastCommandSent = 0

# duration (in seconds) after which the head is sent to sleep if no command was sent
idleDuration = 26000

# This sets angles for a pan/tilt neck. Angles are in degrees.
# There are 2 angles to set: pitch (=up/down) and yaw (= left/right).
def set_angles(pitch, yaw):
	global yaw_offset
	global pitch_offset
	global pan_pub
	global tilt_pub
	print "moving to yaw [deg] ", yaw
	print "moving to pitch [yaw] ", pitch
	# add the calibration offsets
	pitch = pitch + pitch_offset
	yaw = yaw + yaw_offset
	# clamp to limits
	if (yaw < min_lr_angle_deg):
		yaw = min_lr_angle_deg
	if (yaw > max_lr_angle_deg):
		yaw = max_lr_angle_deg
	if (pitch < min_ud_angle_deg):
		pitch = min_ud_angle_deg
	if (pitch > max_ud_angle_deg):
		pitch = max_ud_angle_deg
	# convert to rad
	pitch = pitch*math.pi/180
	yaw = yaw*math.pi/180
	# Hobbit down is +, Dynamixel down is -
	#pitch = -pitch
	# publish to Dynamixel topics
	# NOTE: we don't care if the movemnts are synchronised, we would need
	# a special joint controller for that (see ros dynamixel wiki)
	pan_pub.publish(yaw)
	print "moving to pan [rad] ", yaw
	tilt_pub.publish(pitch)
	print "moving to tilt [rad] (after offset correction) ", pitch

def set_head_orientation(msg):
	global current_yaw
	global current_pitch
	global current_yaw_as_set
	global current_pitch_as_set
	global motorsAreOn
	global whenLastCommandSent

	whenLastCommandSent = rospy.get_time()

	print "current angles (pitch, yaw) [deg]:", current_pitch, " ", current_yaw
	print "Move to", msg.data

	if (not motorsAreOn):
		turnTiltMotorOnOff(True)
		turnPanMotorOnOff(True)
		motorsAreOn = True

	if msg.data == "up_center":
		set_angles(pitch=-20, yaw=0)

	elif msg.data == "center_center":
		set_angles(pitch=0, yaw=0)

	elif msg.data == "down_center":
		set_angles(pitch=35, yaw=0)

	elif msg.data == "littledown_center":
		set_angles(pitch=16, yaw=0)

	elif msg.data == "center_right":
		set_angles(pitch=0, yaw=-90)

	elif msg.data == "down_right":
		set_angles(pitch=50, yaw=-90)

	elif msg.data == "to_turntable":
		set_angles(pitch=40, yaw=-61)

	elif msg.data == "to_grasp":
		set_angles(pitch=61, yaw=-65)
		# NOTE: this is a hacky way of letting the tilt servo rest without power in its mechanical
		# end stop position
		rospy.sleep(5.0)
		turnTiltMotorOnOff(False)
		turnPanMotorOnOff(False)
		motorsAreOn = False

	elif msg.data == "center_left":
		set_angles(pitch=0, yaw=90)

	elif msg.data == "down_left":
		set_angles(pitch=50, yaw=90)

	elif msg.data == "up_left":
		set_angles(pitch=-20, yaw=90)

	elif msg.data == "up_right":
		set_angles(pitch=-20, yaw=-90)        

	else:
		try:
			if (msg.data[0]) == "a":	#absolut values (in degree) for head servos
				print "==> set absolute values"
				input = msg.data.split()
				lr_angle = int(input[1])
				ud_angle = int(input[2])
				set_angles(pitch=ud_angle, yaw=lr_angle)
				
			if (msg.data[0]) in ("r", "l", "u", "d"):	#incremental head move
				print "==> set relative values"
				lr_shift = 0
				ud_shift = 0

				print "current angles (pitch, yaw):",current_pitch, " ", current_yaw

				# NOTE: the Dynamixel RX-28 servos used in Hobbit have a resolution of 0.29 deg
				# So if you set a relative angle of 1 deg, it will typically move 0.87 = approx. 1 - 0.29/2
				# NOTE: when moving e.g. left, then right, there is a hysteresis. E.g. when moving in 1 deg
				# increments: ... l l r (not moving) r (not moving) r (moving) r (moving) ...
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
				lr_angle = current_yaw_as_set + lr_shift
				ud_angle = current_pitch_as_set + ud_shift
	
				print "6 set yaw=: ", lr_angle
		                print "6 set pitch=: ", ud_angle
		                set_angles(pitch=ud_angle, yaw=lr_angle)
				
		except:
			print "===============================================> owlpose.py: ERROR during relative setting of HEAD: ", sys.exc_info()[0]

	rospy.sleep(0.1)

# This is called periodically and checks if the head is idle.
# If there has not been any command in some time the head is put to sleep.
def checkHeadIdle(event):
    global whenLastCommandSent
    global motorsAreOn

    if (motorsAreOn):
        now = rospy.get_time()
        if (now - whenLastCommandSent > idleDuration):
            driveServosToSleep()

# Wait until dynamixel manager node has come up
def waitForServosReady():
    # just wait for any of the services provided by the dynamixel manager
    rospy.wait_for_service('/tilt_controller/torque_enable')
    # and wait a bit longer, for good measure
    rospy.sleep(1.)

# enable/disable torque to tilt servo
# NOTE: this is a hacky solution to allow the servo to rest on its mechanical stop
# @param on  True or False
def turnTiltMotorOnOff(on):
    print "tilt motor en/disable: ", on
    rospy.wait_for_service('/tilt_controller/torque_enable')
    try:
        torque_enable = rospy.ServiceProxy('/tilt_controller/torque_enable', TorqueEnable)
        torque_enable(on)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# enable/disable torque to pan servo
# @param on  True or False
def turnPanMotorOnOff(on):
    print "pan motor en/disable: ", on
    rospy.wait_for_service('/pan_controller/torque_enable')
    try:
        torque_enable = rospy.ServiceProxy('/pan_controller/torque_enable', TorqueEnable)
        torque_enable(on)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# callback for sleep service
def sleepCallback(msg):
    driveServosToSleep()
    return std_msgs.msg.Empty

# Drive the servos to a resting position and turn torque off
def driveServosToSleep():
    global motorsAreOn

    if motorsAreOn:
        print "putting servos to sleep"
        set_angles(pitch=max_ud_angle_deg, yaw=0)
        print "before sleep"
        rospy.sleep(5.0)
        print "after 5 s sleep"
        turnPanMotorOnOff(False)
        turnTiltMotorOnOff(False)
        motorsAreOn = False
    else:
        print "servos are already asleep"

# Set torque limit for dynamixel servos
def setTorqueLimit(torque):
    rospy.wait_for_service('/pan_controller/set_torque_limit')
    rospy.wait_for_service('/tilt_controller/set_torque_limit')
    try:
        set_pan_torque = rospy.ServiceProxy('/pan_controller/set_torque_limit', SetTorqueLimit)
        set_tilt_torque = rospy.ServiceProxy('/tilt_controller/set_torque_limit', SetTorqueLimit)
	set_pan_torque(torque)
	set_tilt_torque(torque)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def command(msg):
	global whenLastCommandSent

	whenLastCommandSent = rospy.get_time()

	if msg.data == "restart":
	        print "Restarting servos. Setting torque limit to: ", torque_limit
		setTorqueLimit(torque_limit)
	if msg.data == "torque":
	        print "Setting torque limit to: ", torque_limit
		setTorqueLimit(torque_limit)
	if msg.data == "startup":
		print "Moving to center_center"
		set_angles(pitch=0, yaw=0)

# get an IMU message and store pitch angle
# NOTE: this assumes a specific mounting of the IMU inside the head:
# z pointing down, x pointing forward
def imuUpdate(msg):
    # never forget those global definitions - god how I hate python ..
    global imuPitch
    global robotMoving
    global imu_pitch_offset

    # HACK: this actually needs a mutex or something. No idea how that works
    # in python. I hate python. I also hate HOBBIT.
    if robotMoving == False:
        x = msg.linear_acceleration.x
        y = msg.linear_acceleration.y
        z = msg.linear_acceleration.z
        m = math.sqrt(x*x + y*y + z*z)
        x = x/m
        y = y/m
        z = z/m
        # HACK: this also needs a mutex or something.
        imuPitch = -math.asin(x)*180.0/math.pi - imu_pitch_offset
    # else: do not update angle

# store whether the robot is currently moving
def movementUpdate(msg):
	global robotMoving
	# HACK: this also needs a mutex or something.
	if msg.data == "Idle":
		robotMoving = False
	else:
		robotMoving = True

# get current pan (yaw) angle from dynamixel joint state topic
def panJointStateUpdate(msg):
	global current_yaw
	global yaw_offset
	global current_yaw_as_set
	global yaw_temperature
	# HACK: this also needs a mutex or something.
	current_yaw = msg.current_pos
	current_yaw_as_set = msg.goal_pos
	yaw_temperature = msg.motor_temps[0]
	# transform to degrees
	current_yaw = current_yaw*180/math.pi
	current_yaw_as_set = current_yaw_as_set*180/math.pi
	# subtract offset
	current_yaw = current_yaw - yaw_offset
	current_yaw_as_set = current_yaw_as_set - yaw_offset

# get current pitch (tilt) angle from dynamixel joint state topic
def tiltJointStateUpdate(msg):
	global current_pitch
	global pitch_offset
	global current_pitch_as_set
	global pitch_temperature
	# HACK: this also needs a mutex or something.
	current_pitch = msg.current_pos
	current_pitch_as_set = msg.goal_pos
	pitch_temperature = msg.motor_temps[0]
	# Hobbit down is +, Dynamixel down is -
	#current_pitch = -current_pitch
	#current_pitch_as_set = -current_pitch_as_set
	# transform to degrees
	current_pitch = current_pitch*180/math.pi
	current_pitch_as_set = current_pitch_as_set*180/math.pi
	# subtract offet
	current_pitch = current_pitch - pitch_offset
	current_pitch_as_set = current_pitch_as_set - pitch_offset

# read offsets from parameter server and set them
def read_and_set_offsets():
    global pitch_offset
    global yaw_offset
    global imu_pitch_offset
    # Read calibration parameters
    pitch_offset = rospy.get_param('/hobbit/head/pitch_offset', 0)
    yaw_offset = rospy.get_param('/hobbit/head/yaw_offset', 0)
    imu_pitch_offset = rospy.get_param('/hobbit/head/imu_pitch_offset', 0)
    print "set pitch offset of: ", float(pitch_offset)
    print "set yaw offset of: ", float(yaw_offset)
    print "set IMU pitch offset of: ", float(imu_pitch_offset)

def trigger_read_and_set_offsets(msg):
    read_and_set_offsets()

# check if servos are alive or have shut off due to torque or temperature
# thresholds
#def checkServosAlive():
#    if (pitch_temperature < 75)
#        setTorqueLimit(torque_limit)


def init():
    global imuPitch
    global pitch_offset
    global yaw_offset
    global imu_pitch_offset
    global haveImu
    global pan_pub
    global tilt_pub
    
    # Initialize Rosnode:
    rospy.init_node('owlpose')
    print "Initialized"

    # Read calibration parameters
    read_and_set_offsets()
    
    # Subscriber for Movements:
    rospy.Subscriber("/head/move", std_msgs.msg.String, set_head_orientation)
    rospy.Subscriber("/head/move/incremental", std_msgs.msg.String, set_head_orientation, queue_size=1)
    rospy.Subscriber("/head/cmd", std_msgs.msg.String, command)
    # Subscriber to Dynamixel joint states
    rospy.Subscriber("/pan_controller/state", dynamixel_msgs.msg.JointState, panJointStateUpdate)
    rospy.Subscriber("/tilt_controller/state", dynamixel_msgs.msg.JointState, tiltJointStateUpdate)
    # Subscriber to IMU messages for tilt angle
    if haveImu:
        rospy.Subscriber("/imu/data_raw", sensor_msgs.msg.Imu, imuUpdate)

    # Subscriber to robot motion state
    rospy.Subscriber("/DiscreteMotionState", std_msgs.msg.String, movementUpdate)
    # get triggered if new calibration offsets are available
    rospy.Subscriber("/head/trigger/set_offsets", std_msgs.msg.String, trigger_read_and_set_offsets, queue_size=1)

    # Publisher for commands to dynamixel servos
    pan_pub = rospy.Publisher('/pan_controller/command', std_msgs.msg.Float64)
    tilt_pub = rospy.Publisher('/tilt_controller/command', std_msgs.msg.Float64)

    # Services
    s = rospy.Service('/head/sleep', HeadSleep, sleepCallback)

    # Initialize Servos:
    waitForServosReady()
    print "Moving to center_center"
    set_angles(pitch=0, yaw=0)

    # Set timer for putting idle head to sleep
    #rospy.Timer(rospy.Duration(idleDuration), checkHeadIdle)

    # Send geometry/tf constantly with 5hz
    r = rospy.Rate(5) # 5 Hz
    #i = 0
    while not rospy.is_shutdown():
        if haveImu:
            # ignore the servo pitch angle (0) and use pitch from IMU instead
            br.sendTransform( (0,0,0), tf.transformations.quaternion_from_euler(0, imuPitch/180.0*math.pi, current_yaw/180.0*math.pi), rospy.Time.now(), base_tf, head_tf)
            if debugPrintAngles:
                print "owlpose pitch (IMU/servo) yaw [deg]: " + str(imuPitch) + " / " + str(current_pitch) + "  " + str(current_yaw)
        else:
            br.sendTransform( (0,0,0), tf.transformations.quaternion_from_euler(0, current_pitch/180.0*math.pi, current_yaw/180.0*math.pi), rospy.Time.now(), base_tf, head_tf)
            if debugPrintAngles:
                print "owlpose pitch yaw [deg]: " + str(current_pitch) + "  " + str(current_yaw)
        #i = i + 1
        # NOTE: sometimes the tilt servo goes into torque limitation. Setting the torque limit again typically
        # wakes it up again. So we regularly set the limit, every second. So the neck should recover quickly
        # from a torque limit situation.
        #if (i == 5):
        #    #setTorqueLimit(torque_limit)
        #    i = 0
        r.sleep()

# NOTE: Apparently this is not sent anymore. It seems ROS connections are already terminated at that point.
def shutdown():
	print "Moving to center_center"
	set_angles(pitch=0, yaw=0)

if __name__ == '__main__':
	try:
		init()
		atexit.register(shutdown)
	except rospy.ROSInterruptException:
		pass


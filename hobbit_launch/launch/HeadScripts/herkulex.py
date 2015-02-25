#!/usr/bin/python

import serial
import struct
import time
import sys
from math import *

# these are typical values and probably never need to be changed
device = "/dev/ttyUSB0"
port = serial.Serial(device, baudrate = 115200, timeout = .01)

# Herkulex servo IDs
# NOTE: pitch and yaw IDs are exchanged on purpose
pitchServoId = 0x01
yawServoId = 0x00
rollServoId = 0x02
pitch2ServoId = 0x03

# calibration offset for pitch and yaw
pitchOffset = 0
yawOffset = 0

# last angles that were set
lastSetPitch = 0
lastSetYaw = 0

# for correct speed calculation we need to know if we already have valid
# lastSetAngles
firstSetCommand = True

def sendCommand(id,command,data=[]):
	"send a servo command to the serial port"
	header = 0xff
	size = 7 + len(data)
	checksum = size ^ id ^ command
	for d in data:
		checksum ^= d
	checksum1 = checksum & 0xfe
	checksum2 = (~checksum) & 0xfe
	cmd = [header, header]
	cmd.append(size)
	cmd.append(id)
	cmd.append(command)
	cmd.append(checksum1)
	cmd.append(checksum2)
	cmd += data
	hexcmd = ''
	for c in cmd:
		hexcmd += struct.pack('B', c)
	port.write(hexcmd)
	time.sleep(0.2)

def ack():
	ack = []
	ret = []
	c = 0
	
	#Check if any Response is present and find start:
	ack.append(port.read(1))
	while ack[0] and c<3:
		if ord(ack[0])!=255:
			del(ack[0])
			ack.append(port.read(1))
			c=c+1
		else:
			c=5

	if ack[0]:
		if c==3:
			print "No ack received, tried 3 times"
			return -1
	else:
		print "No ack received at all"
		return -1
	
	#Find Response Start:
	while ord(ack[0])!=255:
		print "Deleted: ",ord(ack[0])
		del(ack[0])
		ack.append(port.read(1))
	
	#Receive Response length:
	ack.append(port.read(1)) #255
	ack.append(port.read(1)) #length

	#Receive complete Response
	for i in range(ord(ack[2])-3):
		try:
			ack.append(port.read(1))
		except :
			print "ERROR: Invalid number of bytes received"
			return -1
	
	#Return complete Response
	return ack

def resetServos():
	sendCommand(0xFE,0x09)

def setTorque():
	sendCommand(0xFE,0x03,[0x34, 0x01, 0x60])

def setPitchOffset(offs = 0):
	global pitchOffset
	pitchOffset = offs

def setYawOffset(offs = 0):
	global yawOffset
	yawOffset = offs

# Write function header comments, GODDAMNIT!
# Returns an ARRAY with PITCH and then YAAAAAAAW, in ....... DEGREEES.
# Now that was not so hard, was it?
def getAngles():
	angle = []
	valid0 = False
	valid1 = False
	valid = False
	
	#tilt Servo:
	sendCommand(pitchServoId,0x04,[0x3c,0x02])
	response = ack()
	if response!=-1 and len(response) >= 11:
		angle0 = int(response[10].encode('hex') + response[9].encode('hex'), 16) * 0.325 - 512 + 345
		angle.append(angle0 - pitchOffset)
		valid0 = True
	else:
		print "Error obtaining Angle from Servo " + str(pitchServoId)
		angle.append(0)
		
	#yaw Servo:
	sendCommand(yawServoId,0x04,[0x3c,0x02])
	response = ack()
	if response!=-1 and len(response) >= 11:
		angle1 = int(response[10].encode('hex') + response[9].encode('hex'), 16) * 0.325 - 512 + 345
		angle.append(angle1 - yawOffset)
		valid1 = True
	else:
		print "Error obtaining Angle from Servo " + str(yawServoId)
		angle.append(0)
	
	valid = valid0 and valid1
	return (valid,angle)

# This sets angles for a pan/tilt neck. Angles are in degrees.
# There are 4 angles to set: pitch (=up/down) and yaw (= left/right).
# playtime is the time until the movement is finished (in god-knows what unit). Depending on
# the angle distance to move, a given playtime results in different angular speeds.
# playTime = (int) (playTime / 11.2f); // ms --> value
def setAngles(pitch=0,yaw=0,playtime=70):
	global lastSetPitch
	global lastSetYaw
	global firstSetCommand

	ledc1 = ledc2 = ledc3 = ledc4 = 0x00
	roll = 0
	pitch2 = 0

	# If this is run for the first time, we have no valid previous angles.
	# The head might currently stand at any angle.
	print "prev angles: " + str(lastSetPitch) + " " + str(lastSetYaw)
	if firstSetCommand:
		valid,currentAngles = getAngles()
		lastSetPitch = currentAngles[0]
		lastSetYaw = currentAngles[1]
		firstSetCommand = False
		print "first prev angles: " + str(lastSetPitch) + " " + str(lastSetYaw)
	print "set angles: " + str(pitch) + " " + str(yaw)

	speed = 45
	# convert speed from degree/s to degree/mysterious playtime unit
	print "speed in degree/s: " + str(speed)
	speed = speed * 0.004
	print "speed in degree/playtime: " + str(speed)

	# calculate playtime according to set angle and speed
	maxDelta = max(abs(pitch - lastSetPitch), abs(yaw - lastSetYaw))
	print "maxDelta: " + str(maxDelta)
	pplaytime = maxDelta / speed
	# sanity checks
	pplaytime = max(pplaytime, 10)
	pplaytime = min(pplaytime, 1000)
	print "playtime would be: " + str(pplaytime)
	print ""

	lastSetPitch = pitch
	lastSetYaw = yaw

	# now add the calibration offsets
	pitch = pitch + pitchOffset
	yaw = yaw + yawOffset

	pitch = int(pitch / 0.325 + 512)
	yaw = int(yaw / 0.325 + 512)
	roll = int(roll / 0.325 + 512)
	pitch2 = int(pitch2 / 0.325 + 512)

	pitch = 21 if pitch < 21 else pitch
	pitch = 1002 if pitch > 1002 else pitch

	yaw = 21 if yaw < 21 else yaw
	yaw = 1002 if yaw > 1002 else yaw

	roll = 21 if roll < 21 else roll
	roll = 1002 if roll > 1002 else roll

	pitch2 = 21 if pitch2 < 21 else pitch2
	pitch2 = 1002 if pitch2 > 1002 else pitch2

	data = [int(playtime), pitch & 0x00FF, (pitch & 0xFF00) >> 8, ledc1, pitchServoId, yaw & 0x00FF, (yaw & 0xFF00) >> 8, ledc2, yawServoId, roll & 0x00FF, (roll & 0xFF00) >> 8, ledc3, rollServoId, pitch2 & 0x00FF, (pitch2 & 0xFF00) >> 8, ledc4, pitch2ServoId]
	sendCommand(0xFE,0x06,data)

def main():
	print "Herkulex Servo Driver started"
	pass

if __name__ == '__main__':
	main()


#!/usr/bin/python

import serial
import struct
import time
import sys
from math import *

device = "/dev/ttyUSB0"
port = serial.Serial(device, baudrate = 115200, timeout = .01)
data = []

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
	#print cmd
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

def getAngles():
	angle = []
	
	#Servo0:
	sendCommand(0x00,0x04,[0x3c,0x02])
	response = ack()
	#print response
	if response!=-1:
		#print " ".join(hex(ord(n)) for n in response)
		angle0 = int(response[10].encode('hex') + response[9].encode('hex'), 16) * 0.325 - 512 + 345
		angle.append(angle0)
	else:
		print "Error obtaining Angle from Servo 0"
		angle.append(0)
		
	#Servo1:
	sendCommand(0x01,0x04,[0x3c,0x02])
	response = ack()
	if response!=-1:
		#print " ".join(hex(ord(n)) for n in response)
		angle1 = int(response[10].encode('hex') + response[9].encode('hex'), 16) * 0.325 - 512 + 345
		angle.append(angle1)
	else:
		print "Error obtaining Angle from Servo 1"
		angle.append(0)
	
	return angle

def setAngles(pitch=0,c1=0x00,yaw=0,c2=0x01,roll=0,c3=0x02,pitch2=0,c4=0x03,playtime=70):
	ledc1 = ledc2 = ledc3 = ledc4 = 0x00

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

	data = [int(playtime), pitch & 0x00FF, (pitch & 0xFF00) >> 8, ledc1, c1, yaw & 0x00FF, (yaw & 0xFF00) >> 8, ledc2, c2, roll & 0x00FF, (roll & 0xFF00) >> 8, ledc3, c3, pitch2 & 0x00FF, (pitch2 & 0xFF00) >> 8, ledc1, c4]
	sendCommand(0xFE,0x06,data)

def main():
	print "Herkulex Servo Driver started!"
	pass

if __name__ == '__main__':
	main()

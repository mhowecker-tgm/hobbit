#!/usr/bin/python
PKG = 'aal_service'
# HOBBIT service for AAL Sensors
# AAT/PM 2013
# HM PIR sets ENV.Test for testing...

import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import os
import ast
import re
import time
import signal
from datetime import datetime
from std_msgs.msg import String, Header
from hobbit_msgs.msg import Command, Status, Event, Parameter
from hobbit_msgs.srv import Request, RequestRequest

#### this needs PYTHONPATH set to where hobbitlib.py is ####
#sys.path.append(os.path.abspath(os.path.dirname(os.path.abspath(sys.argv[0]))+'/../../hobbit_lib/src'))
#from hobbitlib import getparam, setparam
#### for get/set param

# Simple TCP Server/Client imports
# -------------------------------------------------------------

from time import sleep
from threading import Thread
from twisted.internet.protocol import Protocol, Factory
from twisted.protocols.basic import LineReceiver
from twisted.internet import reactor, protocol
from twisted.internet.protocol import Factory, Protocol, ClientCreator
from twisted.internet.endpoints import TCP4ClientEndpoint
import socket


port = 8108
callactivityset = {}
activityset = {}
placeset = {}
actuatorset = {}

def getparam(deentry):
        mypar=deentry.replace('.', '/')
        try:
            print "lib getparam", mypar, "=",
            para=rospy.get_param(mypar,"-")
            print para
            return para
        except:
            print "<>"
            return ""

def setparam(deentry,s):
        mypar=deentry.replace('.', '/')
        try:
            print "setparam", mypar, "=", s, ":",
            rospy.set_param(mypar,s)
            para=rospy.get_param(mypar,"-")
            print "OK"
        except:
            print "<>"

def signal_handler(signal, frame):
        print PKG, ': You pressed Ctrl+C!'
        reactor.stop()

# class sending as response to AAL topic c_actuator command
class topicsend():
 def _gotProtocol(self,p):
    t=str((time.time()))
    print "* got protocol", t
    p.sendMessage("a","b")
    #p.sendMessage(self.actuatorid,self.value)
    #print "request sent", self.actuatorid, self.value

 def processCommand(self, msg):
    self.busid=None
    self.sensorid=None
    self.actuatorid=None
    self.roomid=None
    self.value=None
    print msg
    for param in msg.params:
      if param.name.lower() == 'busid':
       self.busid=param.value
      if param.name.lower() == 'sensorid':
       self.sensorid=param.value
      if param.name.lower() == 'actuatorid':
       self.actuatorid=param.value
      if param.name.lower() == 'roomid':
       self.roomid=param.value
      if param.name.lower() == 'value':
       self.value=param.value
    print "actuator command:", self.busid,self.sensorid,self.actuatorid,self.roomid,self.value
    if self.actuatorid and self.busid and self.busid in actuatorset.keys():
     try:
      s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      s.settimeout(3)
      t=str((time.time()))
      print '* connect',t, s.gettimeout()
      s.connect(('128.130.197.176',5544))
      t=str((time.time()))
      print '* connected',t
      s.sendall("GET /actuators?busId=1&actuatorId=%s&value=%s\n" % (self.actuatorid, self.value))
      t=str((time.time()))
      print '* sent',t
      data = s.recv(1024)
      s.close()
      t=str((time.time()))
      print '* Received', t, repr(data)
     except:
      print"OOps, error in actuation connection"
    else:
      print "No service known for busID",self.busid

class Srv(LineReceiver):
    #=== here we get called
    def do_GET(self, path):
        self.matchObj = re.match( r'GET\s+/sensors\?busID=(\S+)&sensorID=(\S+)&(\S+)', path, re.M|re.I)

        if self.matchObj and self.matchObj.lastindex == 3:
          print "-match sensor-"
          '''self.send_response(200)
          self.send_header('Content-type','text/html')
          self.end_headers()
          # Send the html message
          self.wfile.write("OK")
          self.wfile.close()'''

          #=== retrieve sensor info
          self.skey=self.matchObj.group(1)
          self.key="bus"+self.matchObj.group(1)+"_"+self.matchObj.group(2)
          self.value=self.matchObj.group(3)

          if self.key in placeset.keys():
           self.place=placeset[self.key]

          #=== check sensor.type CALL
          if self.key in callactivityset.keys(): #it is a call button
             self.sensor=callactivityset[self.key]
             if self.sensor:
                print self.sensor
                self.pubecall_event(self.place,self.sensor) #issue call request
                t=str(int(time.time()))
                setparam("ENV.Sensor."+self.sensor+".Time",t)
                setparam("ENV.Sensor."+self.sensor+".Value", self.value)
                print "-got CALL sensor-"

          #=== check sensor.type ACTIVITY
          if self.key in activityset.keys(): #it is an activity related sensor
             self.sensor=activityset[self.key]
             if self.sensor:
                print self.sensor
                self.pubAALevent(self.place,self.sensor,self.value)
                t=str(int(time.time()))
                setparam("ENV.Sensor."+self.sensor+".Time",t)
                setparam("ENV.Sensor."+self.sensor+".Value", self.value)

                #=== remember location of ACTIVITY(user)
                setparam("ENV.Sensor.Time",t)
                setparam("ENV.Sensor.Place", self.place)
                print "-got ACTIVITY sensor-"
          #=== check sensor.type EHOME
          if self.skey == '33':
             self.sensor=self.key
             if self.sensor:
                print self.sensor
                self.pubAALevent("EHOME",self.sensor,self.value)
                t=str(int(time.time()))
                setparam("ENV.Sensor."+self.sensor+".Time",t)
                setparam("ENV.Sensor."+self.sensor+".Value", self.value)

                print "-got EHOME sensor-"
          elif self.skey == '1':
             self.sensor=self.key
             if self.sensor:
                print self.sensor
                self.pubAALevent("ENO",self.sensor,self.value)
                t=str(int(time.time()))
                setparam("ENV.Sensor."+self.sensor+".Time",t)
                setparam("ENV.Sensor."+self.sensor+".Value", self.value)

                print "-got ENO sensor-"
          elif self.skey == '2':
             self.sensor=self.key
             if self.sensor:
                self.sensor=self.sensor.replace(":","_")
                print self.sensor
                self.pubAALevent("HM",self.sensor,self.value)
                t=str(int(time.time()))
                setparam("ENV.Sensor."+self.sensor+".Time",t)
                setparam("ENV.Sensor."+self.sensor+".Value", self.value)
                if self.value.find("PIR=1") >= 0:
                 print "ACT PIR",  self.sensor
                 if self.sensor.find("JEQ0127401") >= 0:
                  setparam("ENV.Test", "Area1")
                 else:
                  setparam("ENV.Test", "Area2")


                print "-got HM sensor-"
          else:
             print "-not tagged sensor-"
          self.sendLine('HTTP/1.1 200 OK')
          self.sendLine('Content-Length: 0')
          self.sendLine('Connection: close')
          self.sendLine('')
          self.sendLine('')

        else:
            self.matchObj = re.match( r'GET\s+/setactuator\?busID=(\S+)&sensorID=(\S+)&(\S+)', path, re.M|re.I)

            if self.matchObj and self.matchObj.lastindex == 3:
              print "-match setactuator-"
              #=== retrieve sensor info
              self.skey=self.matchObj.group(1)
              self.key=self.matchObj.group(2)
              self.value=self.matchObj.group(3)
              print "##### new service for busID",self.skey,"at port",int(self.key,16)
              actuatorset[self.skey]={"port": int(self.key,16),"host": self.transport.getPeer().host}
              print "#####",actuatorset

            else:
              '''self.send_response(400)
              self.send_header('Content-type','text/html')
              self.end_headers()
              # Send the html message
              self.wfile.write("Request error!")'''
        return

    def lineReceived(self, line):
        print "cooked line"
        self.do_GET(line)

    def rawDataReceived(self, line):
        now=datetime.now()
        print now.strftime("%a, %d %b %Y %H:%M:%S.%f"),line
        #print "Peer:",self.transport.getPeer().host
        self.do_GET(line)

    def connectionMade(self):
        print "sensor connected"
        self.setRawMode()

    #=== place a CALL event
    def pubecall_event(self,place,sensor):
        print "pub e_callhobbit"
        e = Event()
        e.header = Header()
        e.event = "E_CALLHOBBIT"
        e.sessionID = '0'
        e.confidence = 1.0
        p=Parameter("place",place)
        e.params.append(p)
        p=Parameter("sensor",sensor)
        e.params.append(p)
        pubE.publish(e)

    #=== register AAL event in DB
    def pubAALevent(self,place,sensor,value,event="E_AAL"):
        print "pub aal_hobbit"
        e = Event()
        e.header = Header()
        e.event = event
        e.sessionID = '0'
        e.confidence = 1.0
        p=Parameter("place",place)
        e.params.append(p)
        p=Parameter("sensor",sensor)
        e.params.append(p)
        p=Parameter("value",value)
        e.params.append(p)
        pubA.publish(e)

if __name__ == '__main__':

   signal.signal(signal.SIGINT, signal_handler)
   signal.signal(signal.SIGTERM, signal_handler)
   rospy.init_node('AalService', anonymous=False)
   t=topicsend()
   pubE = rospy.Publisher("/Event", Event)
   pubA = rospy.Publisher("/AALEvent", Event)
   cSub = rospy.Subscriber("/AAL", Command, t.processCommand, queue_size=1)

   resp=getparam("ENV.SensorList")
   if resp:
       sensors=resp.split('+++')
       print "sensorlist: ",sensors

       for sensor in sensors:
        attr="ENV.Sensor."+sensor+".BusID"
        resp=getparam(attr)
        bus=resp
        attr="ENV.Sensor."+sensor+".NodeID"
        resp=getparam(attr)
        node=resp
        key=bus+"."+node
        attr="ENV.Sensor."+sensor+".Type"
        resp=getparam(attr)
        type=resp
        attr="ENV.Sensor."+sensor+".Place"
        resp=getparam(attr)
        place=resp
        print "ENV.Sensor."+sensor, bus, node, type, place
        if "CALL" in type and len(place) and len(node) and len(bus):
          callactivityset[key] = sensor
        if "ACTIVITY" in type and len(place) and len(node) and len(bus):
          activityset[key] = sensor
        if len(place) and len(node) and len(bus):
          placeset[key] = place
   print "CALLBUTTONS:"
   for sensor in callactivityset.keys():
        print "   ", sensor,callactivityset[sensor],placeset[sensor]
   print "ACTIVITYSENSORS:"
   for sensor in activityset.keys():
        print "   ", sensor,activityset[sensor],placeset[sensor]

   print "Server Started - Connect to port " + str(port) + "..."
   factory = Factory()
   factory.protocol = Srv
   reactor.listenTCP(port, factory)
   reactor.run()


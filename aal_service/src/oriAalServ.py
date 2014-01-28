#!/usr/bin/python
PKG = 'AalService'
# Interface to the DecisionEngine

import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import os
import ast
import re
import time
from std_msgs.msg import String, Header
from HobbitMsgs.msg import Command, Status, Event, Parameter
from HobbitMsgs.srv import Request, RequestRequest
from DecisionService.srv import Evaluate, SetGlobalStringConditionalRequest, SetGlobalStringConditional
from DecisionService.srv import SetGlobalBoolConditional,SetGlobalNumberConditional, SetGlobalBoolConditionalRequest,SetGlobalNumberConditionalRequest
from DecisionService.srv import SaveProfile, SaveProfileRequest, GetAllGlobalProfileAttributesWithPrefix, GetAllGlobalProfileAttributesWithPrefixRequest, GetGlobalProfileStringAttribute, GetGlobalProfileStringAttributeRequest 
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer

PORT_NUMBER = 8108
sensorset = {}
placeset = {}
pubE = rospy.Publisher("/Event", Event)

# this command is for checking the status of the connection
#sudo netstat -ltnp | grep ':8108'

#This class will handles any incoming request from
#the browser 
class myHandler(BaseHTTPRequestHandler):
	
    def address_string(self):
        host, port = self.client_address[:2]
        #return socket.getfqdn(host)
        return host
	#Handler for the GET requests
    def do_GET(self):
    	print "GET reveived"    	
        print self.path
        self.matchObj = re.match( r'/sensors\?busID=(\S+)&sensorID=(\S+)&value=(\d+)', self.path, re.M|re.I)

        if self.matchObj and self.matchObj.lastindex == 3:
			self.send_response(200)
			self.send_header('Content-type','text/html')
			self.end_headers()
			# Send the html message
			self.wfile.write("OK")
			self.wfile.close()

			self.key=self.matchObj.group(1)+"."+self.matchObj.group(2) 
			self.value=self.matchObj.group(3)
			if self.key in sensorset.keys():
			   self.sensor=sensorset[self.key]
			   if self.sensor:
			      print self.sensor
			      self.pubevent(placeset[self.key],sensorset[self.key])        
			      t=str(int(time.time()))
			      self.setProfileEntryString("ENV.Sensor."+sensorset[self.key]+".Time",t) 
			      self.setProfileEntryString("ENV.Sensor."+sensorset[self.key]+".Value", self.value) 
			else:
			   print "no suitable sensor"
			
        else:
			self.send_response(400)
			self.send_header('Content-type','text/html')
			self.end_headers()
			# Send the html message
			self.wfile.write("Request error!")
	return

    def setProfileEntryString(self,ss,s):
        #print "################# SETTING PROFILE ENTRY String:   ",  ss, s
        rospy.wait_for_service('/SetGlobalStringConditional')
        servicecall = rospy.ServiceProxy('/SetGlobalStringConditional', SetGlobalStringConditional)
        try:
            req = SetGlobalStringConditionalRequest(ss,s)
            resp = servicecall(req)
        except rospy.ServiceException, e:
          print "Service did not process request: %s"%str(e)

    def pubevent(self,place,sensor):
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


def getProfileEntry(deentry):
    rospy.wait_for_service('/GetGlobalProfileStringAttribute')
    servicecall = rospy.ServiceProxy('/GetGlobalProfileStringAttribute', GetGlobalProfileStringAttribute)
    try:
        resp = servicecall(deentry)
        return resp
    except rospy.ServiceException, e:
      print "Service did not process request: %s"%str(e)

def main(args):
    rospy.init_node('AalService', anonymous=False)

    resp=getProfileEntry("ENV.SensorList")
    if resp.attribute:
       sensors=resp.attribute.split('+++')
       print sensors
    
    for sensor in sensors:
        attr="ENV.Sensor."+sensor+".BusID"
        resp=getProfileEntry(attr)
        bus=resp.attribute
        attr="ENV.Sensor."+sensor+".NodeID"
        resp=getProfileEntry(attr)
        node=resp.attribute
        key=bus+"."+node
        attr="ENV.Sensor."+sensor+".Type"
        resp=getProfileEntry(attr)
        type=resp.attribute
        attr="ENV.Sensor."+sensor+".Place"
        resp=getProfileEntry(attr)
        place=resp.attribute
        print "ENV.Sensor."+sensor, bus, node, type, place
        if "CALL" in type and len(place) and len(node) and len(bus):
          sensorset[key] = sensor
          placeset[key] = place
    print "CALLBUTTONS:"
    for sensor in sensorset.keys():
        print "   ", sensor,sensorset[sensor],placeset[sensor]


	#Create a web server and define the handler to manage the
	#incoming request
	try:
		server = HTTPServer(('', PORT_NUMBER), myHandler)
		print 'Started httpserver on port ' , PORT_NUMBER
		#Wait forever for incoming htto requests
		#while not rospy.is_shutdown():

		print "starting http server:: serve_forever"
		server.serve_forever()
		print "after serve_forever"
		#df server.socket.close()
	except Exception as inst:
		print inst
		print '^C received, shutting down the web server'
		server.server_close()
	server.server_close()
	
	#rospy.spin()

if __name__ == "__main__":
    main(sys.argv)


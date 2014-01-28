#!/usr/bin/python

PKG = 'sqlitedb'
# Interface to theqlite database

import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import signal
import os
import ast
import serial
import time
import datetime
import threading
from std_msgs.msg import String, Header
from hobbit_msgs.msg import Command, Status, Event, Parameter

# -*- coding: utf-8 -*-

import sqlite3

mylock = threading.Lock()

class sqlitedb:
    def __init__(self):
        self.cSub = rospy.Subscriber("/ActionSequence", Command, self.processCommand, queue_size=1)
        self.stopSub = rospy.Subscriber("/PriorityStop", String, self.processStop, queue_size=1)

        self.subC = rospy.Subscriber("/Command", Command, self.commandCallback )
        self.subE = rospy.Subscriber("/Event", Event, self.eventCallback )
        self.subA = rospy.Subscriber("/AALEvent", Event, self.eventAALCallback )
        self.con = sqlite3.connect('hobbit.db')
        self.cur=self.con.cursor()
        #self.cur.execute("DROP TABLE IF EXISTS Hobbit")
        self.cur.execute("CREATE TABLE IF NOT EXISTS Hobbit(timestamp INTEGER NOT NULL, typCode TEXT, value FLOAT, event TEXT, ref INTEGER)")
        self.cur.execute("CREATE INDEX IF NOT EXISTS [evt] ON [Hobbit] ([event])")
        self.cur.execute("CREATE INDEX IF NOT EXISTS [seq] ON [Hobbit] ([timestamp])")
        self.cur.execute("CREATE INDEX IF NOT EXISTS [trans] ON [Hobbit] ([ref])")
        self.cur.execute("CREATE INDEX IF NOT EXISTS [type] ON [Hobbit] ([typCode])")
        #self.cur.execute("INSERT INTO Hobbit VALUES(0,'NULL',0.0,'null',0)")
        self.con.commit()
        self.cur.execute("SELECT * FROM Hobbit")
        self.rows = self.cur.fetchall()
        print "current content:"
#        for row in self.rows:
#          print row
        print "=== content:"
        self.con.close()
        self.actcon = None
        self.evtcon = None
        self.stpcon = None
        self.cmdcon = None

        
    def putInDB(self, txt, evt, val=0.0, params=[]):
        global mylock
        mylock.acquire()
        yourdate=int(time.time()*1000)
        for par in params:
          #print par
          if par.name == "Name":
            evt = evt + ";" + par.value 
          if par.name == "sensor":
            evt = evt + ";" + par.value 
          if par.name == "place":
            evt = evt + ";" + par.value 
          if par.name == "Event":
            evt = evt + ";" + par.value 
          if par.name == "value":
            evt = evt + ";" + par.value 
        d=int(time.time()*1000)
        evtcon = sqlite3.connect('hobbit.db')
        evtcur=evtcon.cursor()
        evtcur.execute('PRAGMA synchronous = 0') # takes a long time to sync immediately
        evtcur.execute('PRAGMA cache_size = 64000')
        #evtcur.execute('PRAGMA fullfsync = 0')
        #evtcur.execute('PRAGMA journal_mode = OFF')
        evtcur.arraysize = 30000
        d1=int(time.time()*1000)
        entries = evt.split(';')
        if entries[0] == 'S_MMUI' or entries[0] == 'R_MMUI':
          txt = entries[0]
          evt = evt[7:]
        print "INSERTED",yourdate, txt, evt, val
        evtcur.execute("INSERT INTO Hobbit VALUES(?,?,?,?,?)",(yourdate,txt,val,evt,0))
        d2=int(time.time()*1000)
        evtcon.commit()
        #evtcur.execute("SELECT * FROM Hobbit")
        #self.rows = evtcur.fetchall()
        evtcon.close()
        d3=int(time.time()*1000)
        #for row in self.rows:
          #print row
        print "=== content:",yourdate, d1-d, d2-d1,d3-d,"[ms]"
        mylock.release()

    def processStop(self,msg):
        self.putInDB("STP","P_STOP",int(msg.data))

    def processCommand(self,msg):
        self.putInDB("ACT",msg.command,1,msg.params)

    def commandCallback(self,msg):
        self.putInDB("CMD",msg.command,1,msg.params)
    
    def eventCallback(self,msg):
       if msg.event != "SAPIEND":
        self.putInDB("EVT",msg.event,msg.confidence,msg.params)

    def eventAALCallback(self,msg):
        self.putInDB("AAL",msg.event,msg.confidence,msg.params)
    
def main(args):        
    rospy.init_node('sqlitedb', anonymous=False)
    c = sqlitedb()

    rospy.spin()

if __name__ == "__main__":        
    main(sys.argv)

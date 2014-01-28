#!/usr/bin/python

PKG = 'sqlitedb'
# Interface to the sqlite database
# AAT/PM 2013

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
from HobbitMsgs.msg import Command, Status, Event, Parameter
import numpy as np
#from scipy import linalg
import pylab as pl
import matplotlib as mpl
from itertools import cycle
import matplotlib.cm as cm


# -*- coding: utf-8 -*-

import sqlite3

mylock = threading.Lock()

class sqlitedb:
    def getdata(self,typ='',evt=''):
        arg="SELECT * FROM Hobbit  WHERE typCode LIKE '"+typ+"%' AND event LIKE '"+evt+"%'"
        self.cur.execute(arg)
        self.rows = self.cur.fetchall()
        for row in self.rows:
	 z=time.asctime( time.localtime(row[0]/1000))
         zz = row[0]%1000
         print z,"[",zz,"]",row

    def fetch(self,what,color,pos,evt=''):
        #print color
        print what,evt
        arg="SELECT timestamp FROM Hobbit WHERE typCode LIKE '"+what+"%' AND event LIKE '"+evt+"%'"
        arg="SELECT * FROM Hobbit"
        self.cur.execute(arg)
        self.rows = self.cur.fetchall()
        self.zz = []
        t=int(time.time())
        t= t - (t%86400)
        print t, len(self.rows)
        t=0
        for row in self.rows:
          print row
	  z=time.asctime( time.localtime(row[0]/1000))
          print z
          z=mpl.dates.epoch2num(row[0]/1000.)
          if row[0]/1000 > t:
           self.zz.append(z)
        if len(self.zz):
          self.fx.vlines(self.zz, pos, pos+.1, color=color, label=what)
        else:
          self.zz.append(mpl.dates.epoch2num(time.time()))
          self.fx.vlines(self.zz, pos, pos+.01, color=color, label=what)

    def __init__(self):
        #self.cSub = rospy.Subscriber("/ActionSequence", Command, self.processCommand, queue_size=1)
        #self.stopSub = rospy.Subscriber("/PriorityStop", String, self.processStop, queue_size=1)

        #self.subC = rospy.Subscriber("/Command", Command, self.commandCallback )
        #self.subE = rospy.Subscriber("/Event", Event, self.eventCallback )
        #self.subA = rospy.Subscriber("/AALEvent", Event, self.eventAALCallback )
        self.con = sqlite3.connect('hobbit.db')
        self.cur=self.con.cursor()
        #self.cur.execute("DROP TABLE IF EXISTS Hobbit")
        self.cur.execute("CREATE TABLE IF NOT EXISTS Hobbit(timestamp INTEGER NOT NULL, typCode TEXT, value FLOAT, event TEXT, ref INTEGER)")
        self.cur.execute("CREATE INDEX IF NOT EXISTS [evt] ON [Hobbit] ([event])")
        self.cur.execute("CREATE INDEX IF NOT EXISTS [seq] ON [Hobbit] ([timestamp])")
        self.cur.execute("CREATE INDEX IF NOT EXISTS [trans] ON [Hobbit] ([ref])")
        self.cur.execute("CREATE INDEX IF NOT EXISTS [type] ON [Hobbit] ([typCode])")
        self.con.commit()
        
        self.getdata()

        '''self.fx=pl.subplot(111)
        #pl.plot_date(z, 10, "r")
        self.pos=0
        cmap=pl.cm.spectral
        mycolors = [cmap(i) for i in np.linspace(0,1,10)]
        clr = cycle(mycolors)
        self.fetch('S_MMUI',next(clr),self.pos)
        self.pos=self.pos-.1
        self.fetch('R_MMUI',next(clr),self.pos)
        self.pos=self.pos-.1
        self.fetch('CMD',next(clr),self.pos)
        self.pos=self.pos-.1
        self.fetch('EVT',next(clr),self.pos)
        self.pos=self.pos-.1
        self.fetch('AAL',next(clr),self.pos,'E_AAL;%;3.12402')
        self.con.close()

        self.fx.xaxis_date()
        pl.title("DB dump")
        pl.setp(pl.xticks()[1], rotation=30)
        pl.legend(loc='best')
        pl.gcf().autofmt_xdate()
        pl.grid()
        pl.show()
        '''

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
        evtcur.execute("SELECT * FROM Hobbit")
        self.rows = evtcur.fetchall()
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
    #rospy.init_node('sqlitedb', anonymous=False)
    c = sqlitedb()

    #rospy.spin()

if __name__ == "__main__":        
    main(sys.argv)

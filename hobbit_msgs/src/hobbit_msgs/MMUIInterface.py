#!/usr/bin/python

# David Fischinger
# TU Wien
# 11.10.2013
#
# Class defines general MMUI interface methods for text and speech output and input
#


PKG = 'hobbit_msgs'
import roslib; roslib.load_manifest(PKG)
import rospy
from hobbit_msgs.msg import Command, Status
from std_msgs.msg import String, Header
#from hobbit_msgs.msg import Command, Status, Event, Parameter
from hobbit_msgs.srv import Request, RequestRequest




class MMUIInterface:
    def __init__(self):
        self.cSub = rospy.Subscriber("/ActionSequence", Command, self.processCommand, queue_size=1)
        #self.PubStatus = rospy.Publisher("/Status", Status)
        self.stopSub = rospy.Subscriber("/Hobbit/Cancel", String, self.processStop, queue_size=1)
        self.dostop = False


    def processStop(self, msg):
        self.dostop = True  #has to be implemented => ISTU directly on MMUI




    def showMMUI_Info(self, text, wait = "1", prm = "", prm2 = "", prm3 = ""):
        parr = []
        p = Parameter('type','D_PLAIN')
        parr.append(p)
        p = Parameter('text',text)
        parr.append(p)
        p = Parameter('speak',text)
        parr.append(p)
        p = Parameter('Timeout', '15')
        parr.append(p)
        p = Parameter('Repetitions','3')
        parr.append(p)
        p = Parameter('wait',wait)
        parr.append(p)
        if prm != "":
          p = Parameter('parameter',prm)
          parr.append(p)
        if prm2 != "":
          p = Parameter('parameter',prm2)
          parr.append(p)
        if prm3 != "":
          p = Parameter('parameter',prm3)
          parr.append(p)
        r = self.callMMUIService('0','create',parr)


    def showMMUI_OK(self, text, prm = ""):
        parr = []
        p = Parameter('type','D_OK')
        parr.append(p)
        p = Parameter('text',text)
        parr.append(p)
        p = Parameter('speak',text)
        parr.append(p)
        p = Parameter('Repetitions','3')
        parr.append(p)
        p = Parameter('Timeout',"15")
        parr.append(p)
        if prm != "":
          p = Parameter('parameter',prm)
          parr.append(p)
        return self.callMMUIService('0','create',parr)

    def showMMUI_YESNO(self, text, prm = ""):
        parr = []
        p = Parameter('type','D_YES_NO')
        parr.append(p)
        p = Parameter('text',text)
        parr.append(p)
        p = Parameter('speak',text)
        parr.append(p)
        p = Parameter('Repetitions',"3")
        parr.append(p)
        p = Parameter('Timeout',"15")
        parr.append(p)
        if prm != "":
          p = Parameter('parameter',prm)
          parr.append(p)
        return self.callMMUIService('0','create',parr)

    def sendMMUI_Function(self, f):
        parr = []
        p = Parameter('type',f)
        parr.append(p)
        return self.callMMUIService('0','create',parr)


    def callMMUIService(self,sessionid, txt, params):
        rospy.wait_for_service('/MMUI')
        servicecall = rospy.ServiceProxy('/MMUI', Request)
        try:
            h = Header()
            h.stamp = rospy.Time.now()
            req = RequestRequest(h,sessionid,txt,params)
            resp = servicecall(req)
            return resp
        except rospy.ServiceException, e:
          print "Service did not process request: %s"%str(e)
          return None


            #self.showMMUI_Info('T_CF_FIRST_GRASP_FAILED_WILL_TRY_AGAIN', "1")
            #returns to default menu
            #self.sendMMUI_Function('F_MAIN')
            #r = self.showMMUI_OK('T_CF_TAKE_OUT_OBJECT_AND_CONFIRM')



def main(args):
    MMUIIF = MMUIInterface()
    MMUIIF.showMMUI_Info('test test test')

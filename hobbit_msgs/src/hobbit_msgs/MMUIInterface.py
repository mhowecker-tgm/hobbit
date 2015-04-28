#!/usr/bin/python

# David Fischinger
# TU Wien
# 11.10.2013
#
# Class defines general MMUI interface methods for text and speech output and input
#


PKG = 'hobbit_msgs'
#import roslib
import sys
#roslib.load_manifest(PKG)
import rospy
from hobbit_msgs.msg import Command, Status
from std_msgs.msg import String, Header
#from hobbit_msgs.msg import Command, Status, Event, Parameter
from hobbit_msgs.srv import Request, RequestRequest
from hobbit_msgs.msg import Parameter


class MMUIInterface:
    def __init__(self):
        self.cSub = rospy.Subscriber("/ActionSequence", Command, self.processCommand, queue_size=1)
        #self.PubStatus = rospy.Publisher("/Status", Status)
        #self.stopSub = rospy.Subscriber("/Hobbit/Cancel", String, self.processStop, queue_size=1)
        self.dostop = False

    def processStop(self, msg):
        self.dostop = True  # has to be implemented => ISTU directly on MMUI

    def processCommand(self, msg):
	#print "david"
        if msg.command == "C_SPEAK":
            print "C_SPEAK"
            self.showMMUI_Info(msg.params[0].value)
	    print "msg.params[0].value: "
	    print msg.params[0].value
        print "##",msg.command,"##"


    def showMMUI_Info(self, text, wait="1", prm="", prm2="", prm3="", audio=""):
        #print(prm)
        parr = []
        p = Parameter('type', 'D_PLAIN')
        parr.append(p)
        p = Parameter('text', text)
        parr.append(p)
        p = Parameter('speak', text)
        parr.append(p)
        p = Parameter('Timeout', '15')
        parr.append(p)
        p = Parameter('Repetitions', '1')
        parr.append(p)
        p = Parameter('wait', wait)
        parr.append(p)
        if prm != "":
            p = Parameter('parameter', prm)
            parr.append(p)
        if prm2 != "":
            p = Parameter('parameter', prm2)
            parr.append(p)
        if prm3 != "":
            p = Parameter('parameter', prm3)
            parr.append(p)
        if audio != "":
            p = Parameter('play', audio)
            parr.append(p)
        #print(parr) 
        return self.callMMUIService('0', 'create', parr)

    def showMMUI_OK(self, text, prm=""):
        parr = []
        p = Parameter('type', 'D_OK')
        parr.append(p)
        p = Parameter('text', text)
        parr.append(p)
        p = Parameter('speak', text)
        parr.append(p)
        p = Parameter('Repetitions', '3')
        parr.append(p)
        p = Parameter('Timeout', "30")
        parr.append(p)
        if prm != "":
            p = Parameter('parameter', prm)
            parr.append(p)
        return self.callMMUIService('0', 'create', parr)

    def showMMUI_YESNO(self, text, prm=""):
        parr = []
        p = Parameter('type', 'D_YES_NO')
        parr.append(p)
        p = Parameter('text', text)
        parr.append(p)
        p = Parameter('speak', text)
        parr.append(p)
        p = Parameter('Repetitions', "3")
        parr.append(p)
        p = Parameter('Timeout', "15")
        parr.append(p)
        if prm != "":
            p = Parameter('parameter', prm)
            parr.append(p)
        return self.callMMUIService('0', 'create', parr)

    def showMMUI_Calendar(
        self,
        text='show calendar entries',
        timespan='03:00',
        # cat=['drinking', 'drug', 'meeting', 'checklist']
        cat = 'meeting'
    ):
        parr = []
        p = Parameter('type', 'D_REMINDER')
        parr.append(p)
        p = Parameter('text', text)
        parr.append(p)
        #cat = ['drinking', 'drug', 'meeting', 'checklist']
        p = Parameter('category', cat)
        parr.append(p)
        p = Parameter('timespan', timespan)
        parr.append(p)
        return self.callMMUIService('0', 'create', parr)

    def DefineSOSFile(self, filename='default'):
        parr = []
        p = Parameter('type', 'F_SOSFILE')
        parr.append(p)
        p = Parameter('Audio', filename)
        parr.append(p)
        return self.callMMUIService('0', 'create', parr)

    def StartSOSCall(self):
        parr=[]
        p = Parameter('type', 'F_CALLSOS')
        parr.append(p)
        return self.callMMUIService('0', 'create', parr)

    def enable_asr(self):
        parr=[]
        p = Parameter('type', 'F_ASRON')
        parr.append(p)
        return self.callMMUIService('0', 'create', parr)

    def disable_asr(self):
        parr=[]
        p = Parameter('type', 'F_ASROFF')
        parr.append(p)
        return self.callMMUIService('0', 'create', parr)

    def enable_gesture(self):
        parr=[]
        p = Parameter('type', 'F_GestureON')
        parr.append(p)
        return self.callMMUIService('0', 'create', parr)

    def disable_gesture(self):
        parr=[]
        p = Parameter('type', 'F_GestureOFF')
        parr.append(p)
        return self.callMMUIService('0', 'create', parr)

    def remove_last_prompt(self):
        parr=[]
        p = Parameter('type', 'D_REMOVE')
        parr.append(p)
        return self.callMMUIService('-1', 'create', parr)

    def request_mmui_update(self):
        """
        send an update request to the mmui.
        only known use case is the update for the object list. maybe something else as well
        """
        parr=[]
        p = Parameter('type', 'F_UPDATE')
        parr.append(p)
        return self.callMMUIService('0', 'create', parr)

    def GoToMenu(self, menu='F_MAIN'):
        parr = []
        p = Parameter('type', 'F_GOTOMENU')
        parr.append(p)
        p = Parameter('Value', menu)
        parr.append(p)
        return self.callMMUIService('0', txt='Go to menu' + menu[2:], params=parr)

    def sendMMUI_Function(self, f):
        parr = []
        p = Parameter('type', f)
        parr.append(p)
        return self.callMMUIService('0', 'create', parr)

    def callMMUIService(self, sessionid, txt, params):
        try:
            rospy.wait_for_service('/MMUI', 5)
            servicecall = rospy.ServiceProxy('/MMUI', Request)
        except rospy.ROSException, e:
            print('timeout exceeded while waiting for service %s' % e)
            rospy.loginfo('MMUI is not responding.')
            return None
            #raise rospy.ServiceException
        #print('Trying to call MMUIService')
        try:
            h = Header()
            h.stamp = rospy.Time.now()
            req = RequestRequest(h, sessionid, txt, params)
            resp = servicecall(req)
            return resp
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)
            return None

    def askForName(self, text='What is the name of this object?', timeout='30'):
        parr = []
        p = Parameter('type', 'D_NAME')
        parr.append(p)
        p = Parameter('text', text)
        parr.append(p)
        p = Parameter('Timeout', timeout)
        parr.append(p)
        return self.callMMUIService('0', 'create', parr)



            #self.showMMUI_Info('T_CF_FIRST_GRASP_FAILED_WILL_TRY_AGAIN', "1")
            #returns to default menu
            #self.sendMMUI_Function('F_MAIN')
            #r = self.showMMUI_OK('T_CF_TAKE_OUT_OBJECT_AND_CONFIRM')



def main(args):
    #MMUIIF = MMUIInterface()
    #MMUIIF.showMMUI_Info('test test test')
    #print "MMUIInterface"
    rospy.init_node('ActionSequencerLearnObject', anonymous=False)
    mmui = MMUIInterface()
    rospy.spin()

if __name__ == "__main__":
    main(sys.argv)

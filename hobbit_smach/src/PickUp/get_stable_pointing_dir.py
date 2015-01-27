#!/usr/bin/python

## David Fischinger
## 27.01.2015
# 
# !!! IMPORTANT !!!  only works if triggered not more than once in 5 seconds !!!!!!!!!!
# (rostopic pub /SS/doSingleShot std_msgs/String "asdf" -r 0.2) for cyclic triggering
#
# subscribes to /SS/doSingleShot", if String comes in:
#     subscribes to /headcam/depth_registered/points and /camera/rgb/image_color
# publishes 
# publishes [rgb image for camera1  and] point cloud for headcam
# after that it unregisters subscriber of /headcam/depth_registered/points 



#!/usr/bin/python


PKG = 'hobbit_smach'
import roslib; roslib.load_manifest(PKG)
import rospy
import subprocess
import time, sys
from std_msgs.msg import String
from rgbd_acquisition.msg import PointEvents
#from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import actionlib


class GetStablePointingDirectionActionServer(object):

    # create messages that are used to publish feedback/result
    _feedback = hobbit_msgs.msg.StablePointingDirectionFeedback()
    _result   = hobbit_msgs.msg.StablePointingDirectionResult()

    def __init__(self, name):

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, hobbit_msgs.msg.StablePointingDirectionAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        print "GetStablePointingDirectionActionServer was started"

        #Subscriber
        ss_sub = rospy.Subscriber("/SS/doSingleShot", String, self.start_shot, queue_size=1)
        self.pc_sub = None
    
        t = None

    def execute_cb(self, goal):
        self._result.result = PointEvents() #initialization of return value
        #get command from goal
        isSetFunction = False
        strdata = str(goal.command.data)
        print "\nGRASP COMMAND received by the ArmActionServer: >> ", strdata
        input = strdata.split()        
        cmd = input[0]
        ''' GET Functions '''
        #return False/True if appropriate, otherwise True if command was executed
        if cmd == 'GetArmState':
            self._feedback.feedback.data = str(self.ArmClient.GetArmState())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            print "ArmActionServer, feedback type: ", (type) (self.ArmClient.GetArmState())
    
            self._result.result = Bool(True)
        elif cmd == 'GetActualPosition':
            self._feedback.feedback.data = str(self.ArmClient.GetActualPosition())
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = Bool(True)

            self._result.result.data = str2bool(self._feedback.feedback.data)
 
    
 
    
        #if arm was stopped
        if self._stop_arm:
            print "david: arm was stopped manually, new command send to plc"
            feedback = self.ArmClient.SetStopArmMove()
            self._feedback.feedback.data = str(feedback)
            print "ArmActionServer, feedback: ", self._feedback.feedback.data
            self._result.result = Bool(feedback[1]=='COMMAND_OK')
            self._stop_arm = False
    
        #publish feedback
        self._as.publish_feedback(self._feedback)
        self._as.set_succeeded(self._result)
    
            # helper variables
            #r = rospy.Rate(1)
            #success = True
    



    #triggers the process for publishing 
    def start_shot(self, msg):
        print "start shot"
        self.t = None
        #start subscriber
        self.pc_sub = rospy.Subscriber("/headcam/depth_registered/points", PointCloud2, self.pc_callback, queue_size=1)
        
    
    #starts method to publish point cloud for camera1; unregisters subscriber camera1
    def pc_callback(self,msg):
        self.pc_ = msg
        self.pc_sub.unregister()
        print "speichert punktwolke von headcam"
        self.do_publish_cam1()
        
    

    #publishes pc for cam1
    def do_publish_cam1(self):
        print "publish single shot for headcam"
        self.t = rospy.Time.now()
        if self.pc_ == None:
            return
        self.pc_.header.stamp = self.t
        self.pc_pub.publish(self.pc_)
 
   
   

def main(args):       
    print "Trigger node started (single shot point cloud from headcam)" 
    rospy.init_node('Trigger', anonymous=False)

    trig = Trigger()
    rospy.spin()
    
if __name__ == "__main__":        
    main(sys.argv)



   

    

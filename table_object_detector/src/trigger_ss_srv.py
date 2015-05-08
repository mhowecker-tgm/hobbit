#!/usr/bin/python

## David Fischinger
## first version: 8.05.2015
# 
# Service that returns single shot for camera (simplest implementation)
#
# if service is started node subscribes to /headcam/depth_registered/points 
# publishes point cloud for headcam
# after that it unregisters subscriber of /headcam/depth_registered/points 



#!/usr/bin/python


PKG = 'hobbit_smach'
import roslib; roslib.load_manifest(PKG)
import rospy
#import subprocess
import time, sys
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from hobbit_msgs.srv import SingleShotPC


class TriggerSSService():
    def __init__(self, parent=None):

        self.pc_sub = None
         
        self.pc_ = None
        t = None
    
    
    
    def trigger_ss_srv(self):
        s = rospy.Service('/table_object_detector/get_single_shot', SingleShotPC, start_shot)        
        rospy.spin()
    
    #triggers the process for publishing 
    def start_shot(self, req):
        print "start shot in trigger_ss_srv"
        self.t = None
        #start subscriber
        self.pc_sub = rospy.Subscriber("/headcam/depth_registered/points", PointCloud2, self.pc_callback, queue_size=1)
        
    
    #starts method to publish point cloud for camera1; unregisters subscriber camera1
    def pc_callback(self,msg):
        self.pc_ = msg
        self.pc_sub.unregister()
        print "return single shot point cloud for trigger_ss_srv service (from headcam)"
        self.do_publish_cam1()
            

    #publishes pc for cam1
    def do_publish_cam1(self):
        print "return single shot for headcam"
        self.t = rospy.Time.now()
        if self.pc_ == None:
            print "trigger_ss_srv: no point cloud received - should never happen"
            return
        self.pc_.header.stamp = self.t
        
        return SingleShotPCResponse(self.pc_)
        
        
        #self.pc_pub.publish(self.pc_)
 
   
   

def main(args):       
    print "Trigger_ss_srv node started (provides PointCloud2 single shot from headcam via service)" 
    rospy.init_node('trigger_ss_srv', anonymous=False)

    trig_ss_srv = TriggerSSService()
    trig_ss_srv.trigger_ss_srv()
    
    #rospy.spin()
    
if __name__ == "__main__":        
    main(sys.argv)



   

    

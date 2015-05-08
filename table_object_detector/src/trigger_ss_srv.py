#!/usr/bin/python

## David Fischinger
## first version: 8.05.2015
# 
# Service that returns single shot for camera (simplest implementation)
#
# if service is started node subscribes to /headcam/depth_registered/points 
# publishes point cloud for headcam
# after that it unregisters subscriber of /headcam/depth_registered/points 


PKG = 'hobbit_smach'
import roslib; roslib.load_manifest(PKG)
import rospy
import time, sys
#from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from hobbit_msgs.srv import SingleShotPC


pc_sub = None
pc_ = None
t = None
    
    
    
def trigger_ss_srv():
    print "Trigger_ss_srv node started (provides PointCloud2 single shot from headcam via service)" 
    rospy.init_node('trigger_ss_srv', anonymous=False)
    s = rospy.Service('/table_object_detector/get_single_shot', SingleShotPC, start_shot)        
    rospy.spin()



#triggers the process for getting pc 
def start_shot(req):
    global pc_sub
    global pc_
    global t
    print "start shot in trigger_ss_srv"
    t = None
    #start subscriber
    pc_ = rospy.wait_for_message("/headcam/depth_registered/points", PointCloud2, timeout=5)
    
    print "return single shot point cloud for trigger_ss_srv service (from headcam)"

    t = rospy.Time.now()
    if pc_ == None:
        print "trigger_ss_srv: no point cloud received - should never happen"
        #return
    pc_.header.stamp = t
    
    return SingleShotPCResponse(pc_)

    
if __name__ == "__main__":        
    trigger_ss_srv()



   

    

#!/usr/bin/python

## David Fischinger
## first version: 12.08.2014
# 
# Test if chech_free_space service works
#
# subscribes to /SS/doSingleShotTestCFS", if String comes in:
#     subscribes to /headcam/depth_registered/points and uses point cloud for service call
# publishes result (= number of points in defined area)
# after that it unregisters subscriber of /headcam/depth_registered/points 




PKG = 'hobbit_smach'
import roslib; roslib.load_manifest(PKG)
import rospy
import subprocess
import time, sys
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from table_object_detector.srv import *


class Trigger():
    def __init__(self, parent=None):

        #Subscriber
        ss_sub = rospy.Subscriber("/SS/doSingleShotTestCFS", String, self.start_shot, queue_size=1)
        self.pc_sub = None
        #Publisher
        #self.pc_pub = rospy.Publisher("/SS/headcam/depth_registered/points", PointCloud2 )
         
        self.pc_ = None
        t = None
    
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
        print "test check_free_space service"
        self.t = rospy.Time.now()
        if self.pc_ == None:
	    print "==> do_publish_cam1: no point cloud found"
            return
        self.pc_.header.stamp = self.t
        #self.pc_pub.publish(self.pc_)
	rospy.wait_for_service('check_free_space')
    	try:
            check_free_space = rospy.ServiceProxy('check_free_space', CheckFreeSpace)
	    input = CheckFreeSpace()
	    input.cloud = self.pc_
	    input.frame_id_original = String(self.pc_.header.frame_id)
	    input.frame_id_desired = String(self.pc_.header.frame_id)
	    input.x1 = 0
	    input.x2 = 1
	    input.y1 = -0.50
	    input.y2 = 0.5
	    input.z1 = 0
	    input.z2 = 1
	    #resp1 = check_free_space(input)
	    resp1 = check_free_space(input.cloud,input.frame_id_original,input.frame_id_desired,input.x1,input.x2,input.y1,input.y2,input.z1,input.z2)
	    print "number of points in area with boarders \nx1: ", input.x1, "\tx2: ",input.x2,"\ny1: ",input.y1,"\ty2: ",input.y2,"\nz1: ",input.z1,"\tz2: ",input.z2,"\nnr_points: ",resp1.nr_points_in_area
            return resp1.nr_points_in_area
    	except rospy.ServiceException, e:
            print "Service call failed: %s"%e
 	
   
   

def main(args):       
    print "test client for check_free_space node started" 
    rospy.init_node('check_free_space_test_node', anonymous=False)

    trig = Trigger()
    rospy.spin()
    
if __name__ == "__main__":        
    main(sys.argv)



   

    

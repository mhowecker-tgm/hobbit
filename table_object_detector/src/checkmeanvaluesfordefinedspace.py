#!/usr/bin/python

## David Fischinger
## first version: 24.2.2015
# 
# Class that takes single camera shot from head camera, defines space limit for cube and sends point cloud to service where point cloud is transformed to 
# tf /basse_link (defined in this file as parameter for the service call) and the points in the defined space are returned
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


class CheckMeanValues():
    def __init__(self, parent=None):

        #Subscriber
        ss_sub = rospy.Subscriber("/trigger/checkmeanvaluefordefinedspace", String, self.start_shot, queue_size=1)
        #read space limits for cutting point cloud (in new tf system)
        self.spacelimits_sub = rospy.Subscriber("/checkmeanvaluefordefinedspace/setspacelimitparametersheadcam", String, self.setspacelimits_callback, queue_size=1)
        
        self.pc_sub = None
        #Publisher
        #self.pc_pub = rospy.Publisher("/SS/headcam/depth_registered/points", PointCloud2 )
         
        self.pc_ = None
        t = None
        #set space limits (default value)
        self.limit_x1 = -0.5
        self.limit_x2 = 0.4
        self.limit_y1 = -1
        self.limit_y2 = -0.35
        self.limit_z1 = 0.25
        self.limit_z2 = 1.2
        #self.cnt = 0
        
        self.nr_of_points_in_given_space = -1 # -1 <=> not correctly calculated, not valid
    
    #triggers the process for publishing 
    def start_shot(self, msg=None):
        print "start shot"
        self.nr_of_points_in_given_space = -1
        self.t = None
        #start subscriber
        self.pc_sub = rospy.Subscriber("/headcam/depth_registered/points", PointCloud2, self.pc_callback, queue_size=1)
        
    
    #starts method to publish point cloud for camera1; unregisters subscriber camera1
    def pc_callback(self,msg):
        self.pc_ = msg
        self.pc_sub.unregister()
        print "point cloud from camera saved"
        self.do_publish_cam1()
        
    #sets space limit parameters for reducing/cutting point cloud before points are counted
    def setspacelimits_callback(self,msg):
        str = msg.data.split()
        print "str: ", str
        #self.cnt += 1
        #print "cnt: ", self.cnt
        self.limit_x1 = float(str[0])
        self.limit_x2 = float(str[1])
        self.limit_y1 = float(str[2])
        self.limit_y2 = float(str[3])
        self.limit_z1 = float(str[4])
        self.limit_z2 = float(str[5])
        print "new space limit parameter values are set"
        
        
    #publishes pc for cam1
    def do_publish_cam1(self):
        print "test check_mean_values_for_defined_space service"
        self.t = rospy.Time.now()
        if self.pc_ == None:
            print "==> do_publish_cam1: no point cloud found"
            return
        self.pc_.header.stamp = self.t
        #self.pc_pub.publish(self.pc_)
        rospy.wait_for_service('check_mean_values_for_defined_space')
    	try:
            check_mean_values_for_defined_space = rospy.ServiceProxy('check_mean_values_for_defined_space', CheckMeanValuesForDefinedSpace)
	    input = CheckMeanValuesForDefinedSpace()
	    input.cloud = self.pc_
	    input.frame_id_original = String(self.pc_.header.frame_id)
	    #input.frame_id_desired = String(self.pc_.header.frame_id)
	    print "input.frame_id_original: ",input.frame_id_original
	    input.frame_id_desired = String("base_link")
	    print "input.frame_id_desired: ",input.frame_id_desired
	    input.x1 = self.limit_x1
	    input.x2 = self.limit_x2
	    input.y1 = self.limit_y1
	    input.y2 = self.limit_y2
	    input.z1 = self.limit_z1
	    input.z2 = self.limit_z2
	    resp1 = check_mean_values_for_defined_space(input.cloud,input.frame_id_original,input.frame_id_desired,input.x1,input.x2,input.y1,input.y2,input.z1,input.z2)
	    print "number of points in area with boarders \nx1: ", input.x1, "\tx2: ",input.x2,"\ny1: ",input.y1,"\ty2: ",input.y2,"\nz1: ",input.z1,"\tz2: ",input.z2,"\nnr_points: ",resp1.nr_points_in_area
            #return resp1.nr_points_in_area
    	except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        print "==================================\n result:", resp1
        self.nr_of_points_in_given_space = resp1.nr_points_in_area

   

def main(args):       
    print "test client for check_mean_value_for_defined_space node started" 
    rospy.init_node('check_mean_value_for_defined_space_node', anonymous=False)

    trig = CheckMeanValues()
    trig.start_shot()
    rospy.spin()
    
if __name__ == "__main__":        
    main(sys.argv)



   

    

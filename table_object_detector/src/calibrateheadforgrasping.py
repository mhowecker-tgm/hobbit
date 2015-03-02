#!/usr/bin/python
from rgbd_acquisition.srv._SetScale import SetScale

## David Fischinger
## first version: 2.3.2015
# 
# Node sets hobbit head to "to_grasp" position, 
# takes point cloud (robot has to see pure plane)
# messures average height around grasp position
# adapts scalling factor for head camera
# afterwards an object with defined size has to placed at grasping positionthe scaling of the camera is adapted to best fit the ground plane in perception and real world
# code will retrun the offsets for tf (between baselink and hobbit_neck) to correct inaccuracy 
# these values then have to be used in tf_node (=> dynamic reconfigure?) 



PKG = 'hobbit_smach'
import roslib; roslib.load_manifest(PKG)
import rospy
import subprocess
import time, sys
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from table_object_detector.srv import *
from rgbd_acquisition.srv import *

scaleHeadCameraDepth = False


class CalibrateHeadForGrasping():
    def __init__(self, parent=None):
        print "CalibrateHeadForGrasping.__init__() started"
        self.co_emax_z = 0.2     #expected max z-value for calibration object
        self.co_emean_x = 0.0    # expected mean x of highest points for calibration object
        self.co_emean_y = -0.50    # expected mean y of highest points for calibration object
        self.pc_sub = None
        self.wait = True
        #Publisher
        self.headmove_pub = rospy.Publisher("/head/move", String )
        self.debug_pc_pub = rospy.Publisher("/dfdebugpc", PointCloud2)
        #self.actualize_head_offsets_pub = rospy.Publisher("/head/trigger/set_offsets", String)
         
        self.pc_ = None
        t = None
        c1_dist = 0.15
        #set space limits for cuboid 1: cuboid 1 is for calibration object 
        self.limitc1_x1 =  self.co_emean_x - c1_dist
        self.limitc1_x2 =  self.co_emean_x + c1_dist
        self.limitc1_y1 =  self.co_emean_y - c1_dist
        self.limitc1_y2 =  self.co_emean_y + c1_dist
        self.limitc1_z1 =  self.co_emax_z  - c1_dist/2
        self.limitc1_z2 =  self.co_emax_z  + c1_dist/2

        #set space limits for cuboid 2: cuboid 2 is for patch on floor for scaling 
        self.limitc2_x1 =  0.0
        self.limitc2_x2 =  0.2
        self.limitc2_y1 = -0.60
        self.limitc2_y2 = -0.45
        self.limitc2_z1 = -0.2
        self.limitc2_z2 =  0.2
        
        self.nr_of_points_in_given_space = -1 # -1 <=> not correctly calculated, not valid

    
    #executes head movement commands
    def move_head(self, cmd):
        self.headmove_pub.publish(cmd)
    
    #triggers the process for getting single point cloud 
    def start_calibration(self):
        print "CalibrateHeadForGrasping.start calibration() ===> take single shot of environment"
        self.wait = True
        self.nr_of_points_in_given_space = -1
        self.t = None
        raw_input("press button to grab point cloud")
        #start subscriber
        self.pc_sub = rospy.Subscriber("/headcam/depth_registered/points", PointCloud2, self.pc_callback, queue_size=1)
   
    
    #starts method to publish point cloud for camera1; unregisters subscriber camera1
    def pc_callback(self,msg):
        print "==> pc_callback executed/started"
        self.pc_ = msg
        self.pc_sub.unregister()
        print "point cloud from camera saved"
        self.wait = False
        self.debug_pc_pub.publish(self.pc_)
  
    #publishes pc for cam1
    def get_average_z_value(self, cuboidnr=1):
        print "get average z value of cuboid number ", cuboidnr
        self.t = rospy.Time.now()
        if self.pc_ == None:
            print "CalibrateHeadForGrasping.get_average_z_value() ==> no point cloud found!"
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
            if (cuboidnr == 1):
        	    input.x1 = self.limitc1_x1
        	    input.x2 = self.limitc1_x2
        	    input.y1 = self.limitc1_y1
        	    input.y2 = self.limitc1_y2
        	    input.z1 = self.limitc1_z1
        	    input.z2 = self.limitc1_z2
            elif (cuboidnr == 2):
                input.x1 = self.limitc2_x1
                input.x2 = self.limitc2_x2
                input.y1 = self.limitc2_y1
                input.y2 = self.limitc2_y2
                input.z1 = self.limitc2_z1
                input.z2 = self.limitc2_z2
            else:
                print "CalibrateHeadForGrasping.get_average_z_value() ==> cuboidnr is neither 1 nor 2!"
    	        return -1000
            
            resp1 = check_mean_values_for_defined_space(input.cloud,input.frame_id_original,input.frame_id_desired,input.x1,input.x2,input.y1,input.y2,input.z1,input.z2)
    	    print "z-mean of points in area with boarders \nx1: ", input.x1, "\tx2: ",input.x2,"\ny1: ",input.y1,"\ty2: ",input.y2,"\nz1: ",input.z1,"\tz2: ",input.z2,"\nz_mean: ",resp1.z_mean
    	except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        print "==================================\n result:", resp1
        self.nr_of_points_in_given_space = resp1.nr_points_in_area
        return resp1.z_mean



    #returns average xyz values for coboid2
    def get_average_xyz_values(self):
        print "get average z value of cuboid number 1"
        self.t = rospy.Time.now()
        if self.pc_ == None:
            print "CalibrateHeadForGrasping.get_average_xyz_values() ==> no point cloud found!"
            return
        self.pc_.header.stamp = self.t
        #self.pc_pub.publish(self.pc_)
        rospy.wait_for_service('check_mean_values_for_defined_space')
        try:
            check_mean_values_for_defined_space = rospy.ServiceProxy('check_mean_values_for_defined_space', CheckMeanValuesForDefinedSpace)
            input = CheckMeanValuesForDefinedSpace()
            input.cloud = self.pc_
            input.frame_id_original = String(self.pc_.header.frame_id)
            print "input.frame_id_original: ",input.frame_id_original
            input.frame_id_desired = String("base_link")
            print "input.frame_id_desired: ",input.frame_id_desired
            input.x1 = self.limitc1_x1
            input.x2 = self.limitc1_x2
            input.y1 = self.limitc1_y1
            input.y2 = self.limitc1_y2
            input.z1 = self.limitc1_z1
            input.z2 = self.limitc1_z2
            
            resp1 = check_mean_values_for_defined_space(input.cloud,input.frame_id_original,input.frame_id_desired,input.x1,input.x2,input.y1,input.y2,input.z1,input.z2)
            print "mean values of points in area with boarders \nx1: ", input.x1, "\tx2: ",input.x2,"\ny1: ",input.y1,"\ty2: ",input.y2,"\nz1: ",input.z1,"\tz2: ",input.z2,"\nz_mean: ",resp1.z_mean
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        print "==================================\n result:", resp1
        self.nr_of_points_in_given_space = resp1.nr_points_in_area
        return resp1
        
        

def main(args):    
    global scaleHeadCameraDepth   
    print "calibrate head for grasping (find tf correction values) node started" 
    rospy.init_node('calibratehead_for_grasping_node', anonymous=False)
    z_cuboid1 = z_cuboid2 = 0
    calibhead = CalibrateHeadForGrasping()
    rospy.sleep(2)
    #move head to grasp position
    calibhead.move_head( String("to_grasp") )
    print "main: ===> Head is sent to to_grasp position, wait for some seconds and remove all objects in grasping area!!!!!"
    rospy.sleep(5)  #wait for head movement
    
    
    if scaleHeadCameraDepth:    
        calibhead.start_calibration()
        while (calibhead.wait):
            print "sleep: wait for pc"
            rospy.sleep(0.1)
                
        z_cuboid2 = calibhead.get_average_z_value(2)
        print "main: average height of floor (NO OBJECT!!!!! on floor) in grasp area ==> z_cuboid2", z_cuboid2
    
        
        z_error = z_cuboid2   #assume the floor plane is horizontal!! (z_cuboid2 was average height in grasp area)
        print "the plane is to high by (m): ", z_cuboid2
        camheight_tg = 1.156 #camera height (tf) when camera is looking to to_grasp position
        scalingfactor = camheight_tg/(camheight_tg-z_error)
        print "calculated SCALING FACTOR: ", scalingfactor
        if abs(scalingfactor-1.0) > 0.01:
            #scale whole point cloud (using service)
            rospy.wait_for_service('/rgbd_acquisition/setScale')
            try:
                setscale = rospy.ServiceProxy('/rgbd_acquisition/setScale', SetScale)
                input = SetScale()
                input.factor = scalingfactor
                result = setscale(input.factor)
                print "headcamera scaling factor was set"
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        
    
    # get tf correction values:
    print "Put an object with max height: ", calibhead.co_emax_z, "\nat position: \n  x: ", calibhead.co_emean_x, "\n  y: ", calibhead.co_emean_y
    raw_input("read line above and press enter")
    calibhead.start_calibration()   #get point cloud
    while (calibhead.wait):
        print "sleep: wait for pc"
        rospy.sleep(0.1)
    
    meanxyz_cuboid2 = calibhead.get_average_xyz_values()
    print "tf correction factors (x,y,z): ", calibhead.co_emean_x-meanxyz_cuboid2.x_mean, " ", calibhead.co_emean_y-meanxyz_cuboid2.y_mean, " ", calibhead.co_emax_z-meanxyz_cuboid2.z_mean
    
    
if __name__ == "__main__":        
    main(sys.argv)



   

    

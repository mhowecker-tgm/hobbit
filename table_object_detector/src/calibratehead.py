#!/usr/bin/python

## David Fischinger
## first version: 24.2.2015
# 
# Node sets hobbit head to down_center position, 
# takes point cloud (robot has to see pure plane in front)
# messures average height on 2 positions infront of hobbit (1m and 2m away of hobbit)
# adapts pitch offset (rosparameter)
# sends command to owlpose to use new ros parameter for pitch offset
# moves head away (needed because of week servos, small changes would not be executed) and back to (adapted) down_center position
# this procedure is repeated until the height difference is below threshold or after n (5?) iterations
# afterwards the scaling of the camera is adapted to best fit the ground plane in perception and real world



PKG = 'hobbit_smach'
import roslib; roslib.load_manifest(PKG)
import rospy
import subprocess
import time, sys
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from table_object_detector.srv import *


class CalibrateHead():
    def __init__(self, parent=None):
        print "CalibrateHead.__init__() started"
        self.heightdiff_th = 0.03   #threshold that is accepted for heigt difference on 2 different spots (in m)
        self.pc_sub = None
        self.wait = True
        #Publisher
        self.headmove_pub = rospy.Publisher("/head/move", String )
        self.debug_pc_pub = rospy.Publisher("/dfdebugpc", PointCloud2)
        self.actualize_head_offsets_pub = rospy.Publisher("/head/trigger/set_offsets", String)
         
        self.pc_ = None
        t = None
        #set space limits for cuboid 1 (near) robot 
        self.limitc1_x1 =  0.9
        self.limitc1_x2 =  1.1
        self.limitc1_y1 = -0.1
        self.limitc1_y2 =  0.1
        self.limitc1_z1 = -0.3
        self.limitc1_z2 =  0.3

        #set space limits for cuboid 2 (away from) robot 
        self.limitc2_x1 =  1.9
        self.limitc2_x2 =  2.1
        self.limitc2_y1 = -0.1
        self.limitc2_y2 =  0.1
        self.limitc2_z1 = -0.4
        self.limitc2_z2 =  0.4
        
        self.nr_of_points_in_given_space = -1 # -1 <=> not correctly calculated, not valid

    
    #executes head movement commands
    def move_head(self, cmd):
        self.headmove_pub.publish(cmd)
    
    #triggers the process for getting single point cloud 
    def start_calibration(self):
        print "CalibrateHead.start calibration() ===> take single shot of environment"
        self.wait = True
        self.nr_of_points_in_given_space = -1
        self.t = None
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
            print "CalibrateHead.get_average_z_value() ==> no point cloud found!"
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
                print "CalibrateHead.get_average_z_value() ==> cuboidnr is neither 1 nor 2!"
    	        return -1000
            
            resp1 = check_mean_values_for_defined_space(input.cloud,input.frame_id_original,input.frame_id_desired,input.x1,input.x2,input.y1,input.y2,input.z1,input.z2)
    	    print "z-mean of points in area with boarders \nx1: ", input.x1, "\tx2: ",input.x2,"\ny1: ",input.y1,"\ty2: ",input.y2,"\nz1: ",input.z1,"\tz2: ",input.z2,"\nz_mean: ",resp1.z_mean
    	except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        print "==================================\n result:", resp1
        self.nr_of_points_in_given_space = resp1.nr_points_in_area
        return resp1.z_mean


    #get z-mean values for 2 positions and re-adjust head (change offset values)
    def readjust_head(self, z_mean1, z_mean2):
        offsetshift = 0
        z_diff = z_mean1-z_mean2
        if abs(z_diff) < self.heightdiff_th:   # plane is acceptably horizontal
            return True
        elif z_diff < -0.2:
            offsetshift = -5
        elif z_diff < -0.1:
            offsetshift = -3
        elif z_diff < -self.heightdiff_th:
            offsetshift = -1            
        elif z_diff > 0.2:
            offsetshift = 5
        elif z_diff > 0.1:
            offsetshift = 3
        elif z_diff > self.heightdiff_th:
            offsetshift = 1
         
        cur_pitch_offset = rospy.get_param('/hobbit/head/pitch_offset', 0)
        print "==> readjust_head: current pitch offset read from rosparameter: ", cur_pitch_offset   
        print "==> readjust_head: set pitch offset to ", cur_pitch_offset+offsetshift
        rospy.set_param('/hobbit/head/pitch_offset', cur_pitch_offset+offsetshift)
        rospy.sleep(1)
        self.actualize_head_offsets_pub.publish(String("asdf"))
        rospy.sleep(3)
        self.move_head( String("center_center") )
        rospy.sleep(3)
        self.move_head( String("down_center") )
        rospy.sleep(3)
        
        return False
        
        

def main(args):       
    print "calibrate head (tf, depth scaling factor) node started" 
    rospy.init_node('calibratehead_node', anonymous=False)

    calibhead = CalibrateHead()
    rospy.sleep(3)
    calibhead.move_head( String("down_center") )
    print "main: ===> Head is sent to down_center position, wait for some seconds"
    rospy.sleep(5)  #wait for head movement
    
    for i in range(5):
        print "calibration, iteration number: ", i
        calibhead.start_calibration()
        while (calibhead.wait):
            print "sleep: wait for pc"
            rospy.sleep(0.1)
            
        z_cuboid1 = calibhead.get_average_z_value(1)
        print "main: ==> z_cuboid1", z_cuboid1
        z_cuboid2 = calibhead.get_average_z_value(2)
        print "main: ==> z_cuboid2", z_cuboid2
        if calibhead.readjust_head(z_cuboid1, z_cuboid2):
            print "adjusting head was successful"
            break
        elif i == 4:
            print "adjusting head was NOT successful"
    
    #rospy.spin()
    
if __name__ == "__main__":        
    main(sys.argv)



   

    

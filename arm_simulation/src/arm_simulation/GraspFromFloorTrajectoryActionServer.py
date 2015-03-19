#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

""" Finds trajectories for grasps for pick up objects from floor for HobbitPT2

AUTHOR:     David Fischinger
DATE:       15.10.2014
MODIFIED (completely): from OpenRAVE tutorial 

Description
-----------

This node is an ActionServer for calculation trajectories given an input defining a graps.
Old: receives a grasp destination (x,y,z) near the floor where the HobbitPT2 arm should grasp.
The arm trajectory is calculated and saved as a string and a service of the ArmActionServer is 
called for execution of the trajectory. 

"""


PKG = 'hobbit_smach'
#PKG = 'arm_simulation'
EXECUTE_ARM_MOVEMENT = True

import roslib
roslib.load_manifest(PKG)

import actionlib
from hobbit_msgs.msg import *
from hobbit_smach.ArmActionClient import ArmActionClient    #new!!! (21.10.2014)
from std_msgs.msg import String, Bool

import time
import math
from itertools import izip
import rospy
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments


class GraspTrajectoryActionServerFromFloor():

    # create messages that are used to publish feedback/result for action lib
    _feedback = hobbit_msgs.msg.GraspTrajectoryServerFeedback()
    _result   = hobbit_msgs.msg.GraspTrajectoryServerResult()

    #gp_pnt_fixed = [0.32, -0.34, 0.15]   # grasp pre-point coordinates
    grasp_area_param = 5                 #this parameter defines how big the area is where grasps should be possible: value of 5 means that the gripper has 10cm (50cm/5=10cm) space in each direction
    grasp_xy_variation_param = 250        #defines how much offset grasp-x-y-position can have to get valid grasp (trajectory): value of 25 <=> 2 cm offset in each direction (50cm/25=2cm)
    max_traj_diff_rad = 40*pi/180        # maximal joint difference between two trajectory points in rad  => now: 40 degrees tolerated per joint between pos:graspfromfloor and calculated trajectory
    max_obj_height = 18		 #max height of object (grasp point z-value) and also the height the lowest gripper part is away from floor in pre-grasp-position
    gripper_go_further_down_blind_grasp = 6    # amount the gripper goes further down the the gp_z value (now: without collision check)
    gripper_floor_safetey_buffer_cm = 3

    #set up the environment
    #@openravepy.with_destroy
    def __init__(self, name):
        
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, hobbit_msgs.msg.GraspTrajectoryServerAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.iv_model_sub = rospy.Subscriber("/pc_to_iv/generated_ivfilename", String, self.insert_iv_object)
        print "GraspTrajectoryServer was started"
        self.gp_pnt_xy = None
        
        # create arm_action_client
        #self.arm_action_client = actionlib.SimpleActionClient("arm_action_server", hobbit_msgs.msg.ArmServerAction)   #"arm_action_server" has to be nodename of ArmActionServer(.py)
        
        #OpenRAVE scene setup
        sceneName = '/opt/ros/hobbit_hydro/src/arm_simulation/data/Hobbit.env.xml'           
        self.env = Environment()
        #self.env.SetDebugLevel(DebugLevel.Debug)
        #self.env.SetViewer('qtcoin')    #turn off/on visualization in openrave
        self.env.Load(sceneName)
        arm_client = ArmActionClient()
        self.ArmClient = ArmActionClient() #new!!! (21.10.2014) create instance of arm action client to send command to arm
       
        self.robot = self.env.GetRobots()[0]
        manip = self.robot.GetActiveManipulator()
        self.robot.SetActiveManipulator(manip.GetName())
        with self.env:
            self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=IkParameterization.Type.Transform6D)
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()
            self.basemanip = interfaces.BaseManipulation(self.robot)
            self.taskmanip = interfaces.TaskManipulation(self.robot)
            self.robot.SetJointValues([-0.97, -0.97],self.ikmodel.manip.GetGripperIndices())
    
    
    def insert_iv_object(self, data):

        iv_filename = str(data.data)
        print 'GraspFromFloorTrajectoryActionServer.py: insert_iv_object ', iv_filename
        unknown = self.env.ReadKinBodyXMLFile(iv_filename)
        bodyName = unknown.GetName()
        objectInEnv = self.env.GetKinBody(bodyName)
        if objectInEnv is not None:   
            print 'Removing Object'
            self.env.Remove(objectInEnv)
            self.env.UpdatePublishedBodies()
            time.sleep(0.1) # give time for environment to update

        self.env.UpdatePublishedBodies()
        self.env.AddKinBody(unknown)   
        self.env.UpdatePublishedBodies()
  
    # calculates grasping trajectory of given grasp position
    def getTrajForGraspFromFloor(self, gp_z_cm, roll_degree):
	#define steps for trajectory
        stepsize=0.01
        maxsteps = min(self.max_obj_height-self.gripper_floor_safetey_buffer_cm, self.max_obj_height-round(gp_z_cm)+self.gripper_go_further_down_blind_grasp - self.gripper_floor_safetey_buffer_cm)  #!!!!!!!!!!!!!! delete this (safety buffer)
        minsteps = 2 #maxsteps #self.max_obj_height-round(gp_z_cm)+self.gripper_go_further_down_blind_grasp - self.gripper_floor_safetey_buffer_cm  #!!!!!!!!!!!!!! delete this (safety buffer)
        max_fails = 200
        iter_before_relaxing_dir = 30	#after this number of iterations the param grasp_tilt_variation_param is incremented
        print "maxsteps: ", maxsteps
        print "max_obj_height: ", self.max_obj_height
        print "round(gp_z_cm)",round(gp_z_cm)
        print "self.gripper_go_further_down_blind_grasp", self.gripper_go_further_down_blind_grasp
        print "self.gripper_floor_safetey_buffer_cm",self.gripper_floor_safetey_buffer_cm
        failedattempt = 0
        succeeded = True #indicates if trajectory was found
        grasp_tilt_variation_param = 1 #parameter varies tilt relaxation
        # start position where manipulator is oriented normal to the ground (defines manipulator orientation for grasping in this implementation) 
        #PosStart = [30*pi/180,    0,           0,           170*pi/180-20*pi/180, 90*pi/180,     -90*pi/180,  10*pi/180, 10*pi/180]
        PosStart = [69.71*pi/180, 31.39*pi/180, 96.31*pi/180, 122.2*pi/180,         109.74*pi/180, (0.0+roll_degree)*pi/180,  10*pi/180, 10*pi/180] # pregrasp from floor pos
        self.robot.SetJointValues(PosStart)
        Tee_start = self.ikmodel.manip.GetTransform()
        Tee = Tee_start
        Tee[0:2,3] = self.gp_pnt_xy
        print "==================================================================>"
        print Tee[2,3]

        print 'checking for existence of trajectories for grasping from floor'

        while True:
            #with self.env:		#potential source of problem!!!!!!!
            print "failedattempt: ", failedattempt
            direction = array([0,0,-1])    #this direction defines approach direction
    
            if failedattempt > 1:    #if variation is needed to get possible grasp trajectory
                Tee[0:2,3] = self.gp_pnt_xy + (random.rand(2)-0.5)/self.grasp_xy_variation_param    #vary position to get possible solution

            h = self.env.drawlinelist(array([Tee[0:3,3],Tee[0:3,3]+direction*maxsteps*stepsize]),1)
            try:
                trajdata_str = self.basemanip.MoveHandStraight(direction=direction,starteematrix=Tee,stepsize=stepsize,minsteps=minsteps,maxsteps=maxsteps,outputtraj=True)
                #print "result MoveHandStraight: ",trajdata_str
                trajdata = RaveCreateTrajectory(self.env,'').deserialize(trajdata_str)
                num_waypoints = trajdata.GetNumWaypoints()
                
                tra1 = trajdata.GetWaypoint(0)[0:6]*180.0/math.pi
                tra2 = trajdata.GetWaypoint(num_waypoints/2)[0:6]*180.0/math.pi
                tra3 =  trajdata.GetWaypoint(num_waypoints-1)[0:6]*180.0/math.pi
                
                if self.checkTrajectoryOk(PosStart, trajdata.GetWaypoint(0)[0:6]) and self.checkTrajectoryOk(trajdata.GetWaypoint(0)[0:6], trajdata.GetWaypoint(num_waypoints/2)[0:6]):  # tajectory is accepted 
                    # write command (for ArmActionServer) and the first, the one in the middle and the last waypoint of the trajectory into the trajectory_string variable traj_str
                    traj_str = "SetExecuteGrasp " + numpy.array_str(tra1).strip("[]") + numpy.array_str(tra2).strip("[]") + numpy.array_str(tra3).strip("[]")
                    print "traj_str: \n", traj_str
                    #print "trajdata.GetNumWaypoints(): ",trajdata.GetNumWaypoints()
                    
                    params = (direction,Tee)
                    print '%d failed attemps before found'%failedattempt#,repr(params)
                    print "trajectory found. press enter"
                    h = self.env.drawlinelist(array([Tee[0:3,3],Tee[0:3,3]+direction*maxsteps*stepsize]),4,[0,0,1])
                    self.robot.WaitForController(0)
                    
                    if (EXECUTE_ARM_MOVEMENT):
                        #raw_input("press enter to send trajectory to arm_action_server and physically execute the grasp action")
                        print "press enter to send trajectory to arm_action_server and physically execute the grasp action"
                        #EXECUTE arm movement for grasping from floor
                        if (self.ArmClient.SetMoveToPreGraspFromFloorPos()):
                            print "Arm was moved to PreGraspFromFloorPos"
                        
                        if (self.ArmClient.GetArmAtPreGraspFromFloorPos()):
                            self.ArmClient.arm_action_client(String(traj_str))    #should execute the whole grasp trajectory from pregrapsfromflorpos
                            print "the arm is moved for grasping!! tatatata"
                            self._feedback.feedback.data = str("The arm executes the grasping action.")
                            return True #exit function
                        else:
                            self._feedback.feedback.data = str("The arm was not brought to the pre-grasp position as expected, grasping failed.")
                            print " ================> ARM NOT IN PREGRASPFROMFLOOR POSITION - IDIOT! "
                            return False #exit function
                            
                else:
                    print "=============================================================================> trajectory was not accepted because to much movement necessary"
                    failedattempt += 1
                
            except planning_error,e:
                failedattempt += 1

            if failedattempt % iter_before_relaxing_dir == 0:    #after a number of tries (100) the tilt direction is relaxed
                grasp_tilt_variation_param += 0.05
                print "===> New grasp_tilt_variation_param: ", grasp_tilt_variation_param
                self.robot.SetJointValues(PosStart)
                Tee = dot(self.ikmodel.manip.GetTransform(),matrixFromAxisAngle(random.rand(3)-0.5,grasp_tilt_variation_param*0.2*random.rand())) 
            
            if (failedattempt >= max_fails):
                self._feedback.feedback.data = str("The number of tries to get valid grasp was exceeded!")
                return False

    #tests if for two trajectory waypoints tra1 and tra2 the maximal joint difference (each joint is compared) is lower then the treshold self.max_traj_diff_rad
    def checkTrajectoryOk(self, tra1, tra2):
        maxdiff = 0
        for x in range(0,5): # first 5 joints
            if abs(tra1[x]-tra2[x]) > maxdiff:
                maxdiff = abs(tra1[x]-tra2[x])
        if maxdiff < self.max_traj_diff_rad:    # in rad
            return True
        else:
            print "checkTrajectoryOk: max diff in degree: ", maxdiff*180.0/pi
            return False
        
 
 
     # commands triggered by call of ActionServer
    def execute_cb(self, goal):

        #get command from goal
        strdata = str(goal.command.data)
        print "\n input (definition for grasp) for GraspTrajectoryActionServer (command) received: >> ", strdata
        #input format (string): "(0)eval_val (1)gp1_x (2)gp1_y (3)gp1_z (4)gp2_x (5)gp2_y (6)gp2_z (7)ap_vec_x (8)ap_vec_y (9)ap_vec_z (10)gp_center_x (11)gp_center_y (12)gp_center_z (13)roll"
        input = strdata.split()        
        
        #return False/True if appropriate, otherwise True if command was executed
        self._feedback.feedback.data = str("feedback bla bla")
        print "GraspTrajectoryActionServer, feedback: ", self._feedback.feedback.data
        
        
  
        print "type: "
        print type(input[10])
        self.gp_pnt_xy = [float(input[10]),float(input[11])] #[0.32, -0.34]
        gp_z_cm = 100*float(input[12]) #z-value of calculated grasp point z
        roll = float(input[13])	#roll of gripper
        print "New grasp position (x and y value): ", self.gp_pnt_xy
        
        #get trajectory (execution of trajectory via ArmServer is included here)
        res = self.getTrajForGraspFromFloor(gp_z_cm, roll) 
        print "GraspFromFloorGrajectoryActionServer.py -> execute_cb(): res of self.getTrajForGraspFromFloor(): ", res
        if res:
            self._result.result = Bool(True) #value for result if trajectory was found
        else:
            self._result.result = Bool(False) #value for result if trajectory was not found
    
        #publish feedback
        self._as.publish_feedback(self._feedback)
        self._as.set_succeeded(self._result)
    

 
 

if __name__ == "__main__":
    rospy.init_node('grasp_trajectory_action_server')
    print "ROS node grasp_trajectory_action_server started"
    trajSim = GraspTrajectoryActionServerFromFloor(rospy.get_name())
    
    rospy.spin()


    
    

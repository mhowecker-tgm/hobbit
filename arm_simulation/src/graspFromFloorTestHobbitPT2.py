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

This node receives a grasp destination (x,y,z) near the floor where the HobbitPT2 arm should grasp.
The arm trajectory is calculated and saved as a string and a service of the ArmActionServer is 
called for executing the execution of the trajectory. 

"""


PKG = 'hobbit_smach'
#PKG = 'arm_simulation'

import roslib
roslib.load_manifest(PKG)

import actionlib
from hobbit_msgs.msg import *
from hobbit_smach.ArmActionClient import ArmActionClient    #new!!! (21.10.2014)
#from std_msgs.msg import String, Bool

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


#class ArmActionServerROS(object):

class GraspTrajectoryActionServerFromFloor():

    # create messages that are used to publish feedback/result for action lib
    #_feedback = hobbit_msgs.msg.ArmServerFeedback()
    #_result   = hobbit_msgs.msg.ArmServerResult()

    gp_pnt_fixed = [0.32, -0.34, 0.15]    # grasp pre-point coordinates
    grasp_area_param = 5                #this parameter defines how big the area is where grasps should be possible: value of 5 means that that the gripper has 10cm (50cm/5=10cm) space in each direction
    grasp_xy_variation_param = 25         #defines how much offset grasp-x-y-position can have to get valid grasp (trajectory): value of 25 <=> 2 cm offset in each direction (50cm/25=2cm)
    grasp_distance_from_floor_cm = 5     #distance how near gripper should approach the floor
    
    #set up the environment
    #@openravepy.with_destroy
    def __init__(self):
        
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
    
  
    # calculates grasping trajectory of given grasp position
    def getTrajForGraspFromFloor(self):
		#define steps for trajectory
        stepsize=0.01
        maxsteps = 15-self.grasp_distance_from_floor_cm
        minsteps = 15-self.grasp_distance_from_floor_cm
        
        failedattempt = 0
        succeeded = True #indicates if trajectory was found
        grasp_tilt_variation_param = 1 #parameter varies tilt relaxation
        # start position where manipulator is oriented normal to the ground (defines manipulator orientation for grasping in this implementation) 
        PosStart = [30*pi/180, 0, 0, 170*pi/180-20*pi/180, 90*pi/180, -90*pi/180,  10*pi/180, 10*pi/180]
        self.robot.SetJointValues(PosStart)
        Tee_start = self.ikmodel.manip.GetTransform()
        Tee = Tee_start
        Tee[0:2,3] = self.gp_pnt_xy

        print 'checking for existence of trajectories with for grasping from floor'

        while True:
            with self.env:
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
                
                # write command (for ArmActionServer) and the first, the one in the middle and the last waypoint of the trajectory into the trajectory_string variable traj_str
                traj_str = "SetExecuteGrasp " + numpy.array_str(tra1).strip("[]") + numpy.array_str(tra2).strip("[]") + numpy.array_str(tra3).strip("[]")
                print "traj_str: \n", traj_str
                print "trajdata.GetNumWaypoints(): ",trajdata.GetNumWaypoints()
                
                params = (direction,Tee)
                print '%d failed attemps before found'%failedattempt#,repr(params)
                raw_input("trajectory found. press enter")
                h = self.env.drawlinelist(array([Tee[0:3,3],Tee[0:3,3]+direction*maxsteps*stepsize]),4,[0,0,1])
                self.robot.WaitForController(0)
                
                raw_input("press enter to send trajectory to arm_action_server")
                self.ArmClient.arm_action_client(traj_str)    #should execute the whole grasp trajectory
                
                break    #exit trajectory calculation
                
            except planning_error,e:
                failedattempt += 1

            if failedattempt % 100 == 0:    #after a number of tries (100) the tilt direction is relaxed
                grasp_tilt_variation_param += 0.05
                print "===> New grasp_tilt_variation_param: ", grasp_tilt_variation_param
                self.robot.SetJointValues(PosStart)
                Tee = dot(self.ikmodel.manip.GetTransform(),matrixFromAxisAngle(random.rand(3)-0.5,grasp_tilt_variation_param*0.2*random.rand())) 
            #succeeded = False


 

if __name__ == "__main__":
    rospy.init_node('graspFromFloor')
    trajSim = GraspTrajectoryActionServerFromFloor()
    while True:
    	trajSim.gp_pnt_xy = trajSim.gp_pnt_fixed[0:2] + (random.rand(2)-0.5)/trajSim.grasp_area_param
        print "New grasp position (x and y value): ", trajSim.gp_pnt_xy
    	trajSim.getTrajForGraspFromFloor()
    
    

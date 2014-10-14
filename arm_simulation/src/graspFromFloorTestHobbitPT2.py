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
""" Finds trajectories for grasps for pick up object from floor

Description
-----------

This node receives a grasp destination (x,y,z) near the floor where the HobbitPT2 arm should

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time
from itertools import izip
import rospy
import openravepy
if not __openravepy_build_doc__:
	from openravepy import *
	from numpy import *

#gp_pnt = [0.299942150,-0.403854100,0.15]# grasp pre-point coordinates (working coordinates)
gp_pnt_fixed = [0.32,-0.34	,0.15]# grasp pre-point coordinates
grasp_area_param = 5	#this parameter defines how big the area is where grasps should be possible: value of 5 means that that the gripper hast 10cm (50cm/5=10cm) space in each direction
grasp_xy_variation_param = 25 #defines how much offset grasp-x-y-position can have to get valid grasp (trajectory): value of 25 <=> 2 cm offset in each direction (50cm/25=2cm)
grasp_distance_from_floor_cm = 3 #distance how near gripper should approach the floor

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    if options.manipname is not None:
        robot.SetActiveManipulator(options.manipname)
    with env:
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()
        basemanip = interfaces.BaseManipulation(robot)
        taskmanip = interfaces.TaskManipulation(robot)
        robot.SetJointValues([-0.97, -0.97],ikmodel.manip.GetGripperIndices())
        #Tstart = array([[ -1,  0,  0,   2.00000000e-01], [  0,0,   1, 6.30000000e-01], [  0,   1  , 0,   5.50000000e-02], [  0,0,0,1]])
        #sol = ikmodel.manip.FindIKSolution(Tstart,IkFilterOptions.CheckEnvCollisions)
        
    #PosLearning = [70*pi/180,58*pi/180,38*pi/180,-35.5*pi/180,118*pi/180,19*pi/180,10*pi/180,10*pi/180]
    PosStart = [30*pi/180, 0, 0, 170*pi/180-20*pi/180, 90*pi/180, -90*pi/180,  10*pi/180, 10*pi/180]
    #print "PosStart: ", PosStart
    robot.SetJointValues(PosStart)
    Tee_start = ikmodel.manip.GetTransform()
    Tee = Tee_start


    print 'checking for existance of trajectories with for grasping from floor'
 
    stepsize=0.01
    failedattempt = 0
    succeeded = True
    grasp_tilt_variation_param = 1
    while True:
        with env:
	    print "failedattempt: ", failedattempt
            #Tee = dot(ikmodel.manip.GetTransform(),matrixFromAxisAngle(random.rand(3)-0.5,0.2*random.rand()))
	    #Tee = eye(4)
            #Tee = matrixFromAxisAngle(random.rand(3)-0.5,pi*random.rand())
	      #get current transfrom of gripper (further grasping done with same orientation of gripper)
 
	    direction = array([0,0,-1])	#this direction defines apprach direction
	
	    if succeeded:	#if trajectory was found for last grasp point/position
		#define new grasp point
		gp_pnt_xy = gp_pnt_fixed[0:2] + (random.rand(2)-0.5)/grasp_area_param
		print "New grasp postion (x and y value): ", gp_pnt_xy
		Tee[0:2,3] = gp_pnt_xy

	    if failedattempt > 1:	#if variation is needed to get possible grasp trajectory
		Tee[0:2,3] = gp_pnt_xy + (random.rand(2)-0.5)/grasp_xy_variation_param	#vary position to get possible solution
		#print "Tee: ", Tee[0:3,3]


	    maxsteps = 15-grasp_distance_from_floor_cm
	    minsteps = 15-grasp_distance_from_floor_cm
            h = env.drawlinelist(array([Tee[0:3,3],Tee[0:3,3]+direction*maxsteps*stepsize]),1)
        try:
            trajdata_str = basemanip.MoveHandStraight(direction=direction,starteematrix=Tee,stepsize=stepsize,minsteps=minsteps,maxsteps=maxsteps,outputtraj=True)
            #print "result MoveHandStraight: ",trajdata_str
            trajdata = RaveCreateTrajectory(env,'').deserialize(trajdata_str)
            num_waypoints = trajdata.GetNumWaypoints()
            print " MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMDAVID waypoint 0: ",trajdata.GetWaypoint(0)*180.0/3.1415926
            print " MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMDAVID waypoint middle: ",trajdata.GetWaypoint(num_waypoints/2)*180.0/3.1415926
            print " MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMDAVID waypoint last: ", trajdata.GetWaypoint(num_waypoints-1)[0:6]*180.0/3.1415926
            #trajdata2 = RaveCreateTrajectory(self.simulator.simulator,'').deserialize(trajdata_str2)
            print "trajdata.GetNumWaypoints(): ",trajdata.GetNumWaypoints()
            
	    params = (direction,Tee)
            print '%d failed attemps before found'%failedattempt,repr(params)
	    print "direction: ",    direction
	    raw_input("enter")
            failedattempt = 0
	    succeeded = True
	    grasp_tilt_variation_param = 1
	    Tee = Tee_start	# set manipulator orientation to originally desired one
            h = env.drawlinelist(array([Tee[0:3,3],Tee[0:3,3]+direction*maxsteps*stepsize]),4,[0,0,1])
            robot.WaitForController(0)
            
        except planning_error,e:
            failedattempt += 1
	    #raw_input("david")
	    if failedattempt % 100 == 0:
		grasp_tilt_variation_param += 0.05
		print "===> New grasp_tilt_variation_param: ", grasp_tilt_variation_param
		robot.SetJointValues(PosStart)
		Tee = dot(ikmodel.manip.GetTransform(),matrixFromAxisAngle(random.rand(3)-0.5,grasp_tilt_variation_param*0.2*random.rand())) 
	    succeeded = False




from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy 
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how choose IK solutions so that move hand straight can move without discontinuities.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='/opt/ros/grasping/catkin_ws/src/HobbitArm/Hobbit_BG_SLDASM/Hobbit.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname',
                      action="store",type='string',dest='manipname',default=None,
                      help='Choose the manipulator to perform movement for')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()

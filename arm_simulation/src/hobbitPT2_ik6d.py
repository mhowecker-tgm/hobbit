#!/usr/bin/env python
# David Fischinger
# 16.04.2014
# Vienna University of Technology
# 
# Program calculates joint values for grasping an object
#
# input: grasp points
#
# output: joint values for grasp pose
#
# programm initially based on OpenRAVEs tutorial: tutorial_ik5d =====================================> TRANSFORMATION NOT FINISHED!! (October 2014), file kept for looking up stuff

from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


class HobbitGraspControl:

  def init(self):
    self.env = Environment()
    self.env.SetViewer('qtcoin')
    
    self.env.Load('/opt/ros/grasping/catkin_ws/src/HobbitArm/Hobbit_BG_SLDASM/Hobbit.robot.xml')
    self.robot = self.env.GetRobots()[0]
    self.robot.SetActiveManipulator('hand')
    self.iv_model_sub = rospy.Subscriber("/pc_to_iv/generated_ivfilename", String, self.insert_iv_object)
    self.graspPoints_sub = rospy.Subscriber("/SVM/grasp_hypothesis_eval", String, self.calc_grasp_pose)
    self.jointValues_pub = rospy.Publisher("/hobbitarm_joints", String)
    self.waypoints_pub = rospy.Publisher("/hobbit_grasp_control/waypoints",String)
    self.hobbit_jointvalues_pub = rospy.Publisher("/Hobbit/ArmJoints",JointState)
    self.height_diff_pre_grasp = 0.07
    self.PosBase = [0,0,0,0,0,0]
    self.PosGeneralPreGrasp = [-0.364919200084, 1.32040378183, -1.23268442262, 1.30219471831, 1.35427393843, 1]
    self.PosDeliveryGrasp = [0,0,0,0,0,0]

    self.robot = self.env.GetRobots()[0] # get the first robot
    manip = self.robot.GetManipulators()[0]
    jointnames = ' '.join(self.robot.GetJoints()[j].GetName() for j in manip.GetArmJoints())
    #controller = RaveCreateController(self.env,'ROSOpenRAVE + trajectoryservice /controller_session '+jointnames)
    #self.robot.SetController(self.env, controller)
    print "jointnames", jointnames
    #controller = RaveCreateController(self.env,"idealcontroller")
    #self.robot.SetController(controller,range(self.robot.GetDOF()),0)
      
    # generate the ik solver
    #self.ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=IkParameterization.Type.TranslationDirection5D)
    self.ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=IkParameterization.Type.Transform6D)
    if not self.ikmodel.load():
        self.ikmodel.autogenerate()
    self.manipprob = interfaces.BaseManipulation(self.robot)  #DAVID151212
    print "E N D : init()"


  def insert_iv_object(self, data):

	iv_filename = str(data.data)
	print 'insert_iv_object ', iv_filename
	#self.env.UpdatePublishedBodies()
	#time.sleep(0.1) # give time for environment to update
	unknown = self.env.ReadKinBodyXMLFile(iv_filename)
	bodyName = unknown.GetName()
#	objectInEnv = self.env.GetKinBody(bodyName)
	if objectInEnv is not None:   
		print 'Removing Object'
		self.env.Remove(objectInEnv)
		self.env.UpdatePublishedBodies()
		time.sleep(0.1) # give time for environment to update

	self.env.UpdatePublishedBodies()
	self.env.AddKinBody(unknown)   
	self.env.UpdatePublishedBodies()


        
  def open_gripper(self):
      #in simulation
      self.robot.SetDOFValues([-0.69,-0.69],[5,6])
      self.env.UpdatePublishedBodies()


  def close_gripper(self):
      #in simulation
      self.robot.SetDOFValues([0.0,0.0],[5,6])
      self.env.UpdatePublishedBodies()

  def move_arm_to_joint_values(self, joint_values, publish_traj=True):
      print "joint_values: ",joint_values
      print "armindices: ", self.ikmodel.manip.GetArmIndices()
      openrave_traj = self.manipprob.MoveManipulator(goal=joint_values,outputtraj=True) #DAVID151212
      #self.robot.WaitForController(0) # wait DAVID151212
      #self.robot.SetDOFValues(joint_values, self.ikmodel.manip.GetArmIndices()) #DAVID151212
      self.env.UpdatePublishedBodies()
      
      
      if (publish_traj):
          traj = RaveCreateTrajectory(self.env,'')
          traj.deserialize(openrave_traj)
          #robot.GetController().SetPath(traj)
          #robot.WaitForController(0)
          spec = traj.GetConfigurationSpecification()
          for i in range(traj.GetNumWaypoints()):
              data = traj.GetWaypoint(i)
              waypoint = spec.ExtractJointValues(data,self.robot,self.ikmodel.manip.GetArmIndices(),0)
              print "WAYPOINT: ",waypoint, type(waypoint)
	      self.publish_waypoint(waypoint)
            
  def publish_waypoint(self,waypoint):
      #recalculate (offset) jointvalues and transform into increments
      j1_t = waypoint[0]
      j2_t = waypoint[1]
      j3_t = waypoint[2]
      j4_t = waypoint[3]
      j5_t = waypoint[4]
      str_waypoint = str(j1_t)+" "+str(j2_t)+" "+str(j3_t)+" "+str(j4_t)+" "+str(j5_t)+" "      
      self.waypoints_pub.publish(String(str_waypoint))

  def publish_joint_state(self,jointvalues):
      js = JointState()
      jointnames = ('katana_motor1_pan_joint','katana_motor2_lift_joint','katana_motor3_lift_joint','katana_motor4_lift_joint','katana_motor5_wrist_roll_joint')
      for i in range(5):
        js.name.append(jointnames[i])
	js.position.append(jointvalues[i])
	js.velocity.append(0.0)
	js.header.stamp=rospy.Time.now()

      self.hobbit_jointvalues_pub.publish(js)

  def rad_into_tics(self, joints_rad):
      tics = (4640,4960,4640,4960,4640)
      tics_per_rad = [x/(2*pi) for x in tics]
      offset_in_rad = (1.5707963268,-1.5707963268,-1.5707963268,-1.5707963268,0)
      curjointval_plus_offset = [x+y for x,y in zip(joints_rad,offset_in_rad)]
      tics = [x*y for x,y in zip(curjointval_plus_offset,tics_per_rad)]
      tics=(round(-tics[0]), round(tics[1]), round(tics[2]), round(tics[3]), round(-tics[4]))

      return (tics)

  def calc_grasp_pose(self, data):
      print "--> calc_grasp_pose() started"
      #RaveSetDebugLevel(5);
      cnt=-1
      print data.data.split()
      gh = data.data.split()
      k = 0 #value to correct indices of gh regarding if a evaluation value is at the beginning <=> >9 arguments
      if len(gh) > 9:
          k = 1
      gp1 = array([float(gh[0+k]),float(gh[1+k]),float(gh[2+k])])
      gp2 = array([float(gh[3+k]),float(gh[4+k]),float(gh[5+k])])
      print "gp1: ", gp1
      print "gp2: ", gp2
      print "array([gp1,gp1+0.05*(gp2-gp1)]): ",array([gp1,gp1+0.05*(gp2-gp1)])
      h=self.env.drawlinestrip(array([list(gp1),list(gp1+5.5*(gp2-gp1))]),5)
      
      #target_base = ((gp1[0]+gp2[0])/2+(random.rand()-0.5)/fkt_xyz_base_target,(gp1[1]+gp2[1])/2+(random.rand()-0.5)/fkt_xyz_base_target,0.1)
      target_base = ((gp1[0]+gp2[0])/2,(gp1[1]+gp2[1])/2,(gp1[2]+gp2[2])/2.0)
      print "target_base",target_base

      fkt_xyz = 5.0
      fkt_dir=1.0   #the higher the more accurate (but harder to find solution)
      cnt_smopa = 0
      self.solution_pre_grasp = None
      while (self.solution_pre_grasp == None):      #loop for finding smooth path between grasps (pre and end grasp)
          cnt_smopa += 1
          print " >>> Find solutions for pre and end grasp. TRY NUMBER: ", cnt_smopa
          #find joint solution for end grasp    
          while True:
              cnt = cnt+1  
              if (cnt % 1000) == 0:
                  fkt_dir = fkt_dir*0.95 
                  print "fkt_dir for end grasp: ",fkt_dir    
              target=target_base+((random.rand(3)-0.5)/fkt_xyz)    
              dir_x = (random.rand()-0.5)/fkt_dir
              dir_y = (random.rand()-0.5)/fkt_dir
              dir_z = -(sqrt(1-dir_x*dir_x-dir_y*dir_y))
              direction = (dir_x,dir_y,dir_z)
              direction /= linalg.norm(direction)
            
              print "target: ", target
	      print "direction: ", direction
              print "Ray: ", Ray(target,direction)
	      Tmat = eye(4)
	      Tmat[0:3,3] = target
              solutions = self.ikmodel.manip.FindIKSolutions(Tmat,IkFilterOptions.CheckEnvCollisions)
	      print "Tmat =", Tmat
	      #solutions = self.ikmodel.manip.FindIKSolutions(IkParameterization(Ray(target,direction),IkParameterization.Type.TranslationDirection5D),IkFilterOptions.CheckEnvCollisions)
              if solutions is not None and len(solutions) > 0: # if found, then break
                self.solution_grasp = solutions[0]
                print "Solutions end grasp:" 
                for k in range(len(solutions)):
                    print "solution number ", k,": ", solutions[k]
                break
    
    
          fkt_dir=3.0
          cnt1 = -1
          #find joint solution for n cm above end grasp
          while (fkt_dir > 1.0):
              cnt1+=1
              if (cnt1 % 1000) == 0:
                  fkt_dir = fkt_dir*0.93
                  print "fkt_dir for pre grasp: ",fkt_dir     
              target=target_base+((random.rand(3)-0.5)/fkt_xyz)+(0,0,self.height_diff_pre_grasp)
              dir_x = (random.rand()-0.5)/fkt_dir
              dir_y = (random.rand()-0.5)/fkt_dir
              dir_z = -(sqrt(1-dir_x*dir_x-dir_y*dir_y))
              direction = (dir_x,dir_y,dir_z)
              direction /= linalg.norm(direction)
     
              #solutions = self.ikmodel.manip.FindIKSolutions(IkParameterization(Ray(target,direction),IkParameterization.Type.TranslationDirection6D),IkFilterOptions.CheckEnvCollisions)
              sol_ok_at = -1    
              if solutions is not None and len(solutions) > 0: # if found, then break
                print "PPPPPPPPPPPPPPPPPPPPPPPP self.solution_pre_grasp: ",self.solution_pre_grasp
                print "Solutions higher grasp:" 
                max_diff_j3 = 1.0
                for k in range(len(solutions)):
                    if ((solutions[k][2] + max_diff_j3 > self.solution_grasp[2]) and (solutions[k][2] - max_diff_j3 < self.solution_grasp[2]) ):
                        print "j3: diff < ", max_diff_j3
                        sol_ok_at = k
                        self.solution_pre_grasp = solutions[k]
                    print "solution number ", k,": ", solutions[k]
                print "Needed tries for pre grasp: ", cnt1;  
                if (sol_ok_at > -1):
                    break   
                else:
                    print "search for new solution because of j3 difference"
            
          print ">>>>>>>>>>>> COUNTER for end grasp: ",cnt
          
      
      raw_input("solutions anschaun")
      
  
      #return #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
      h=self.env.drawlinestrip(array([list(gp1),list(gp1+5.5*(gp2-gp1))]),7)
        #h=self.env.drawlinestrip(array([target,target+0.1*direction]),10)
        

      i = 0
      with self.env:
            #calculate rotation in current position
            y_h = array([0,1,0])    #y_direction w.r.t. manipulator frame
            self.robot.SetDOFValues(solutions[i],self.ikmodel.manip.GetArmIndices())
            self.env.UpdatePublishedBodies()
            y_h_no_z = dot(self.ikmodel.manip.GetEndEffectorTransform()[0:3,0:3],y_h)
            y_h_no_z[2] = 0
            #calculate alpha = angle difference to reech roll zero position
            y_min_dir = array([0,-1,0]) #has to be of length one! otherwise linalg.norm(y_min_dir) has to be done in angle calc
            alpha = arccos(dot(y_h_no_z,y_min_dir)/(linalg.norm(y_h_no_z)))
            #case for x direction of y_h_no_z positiv
            if (y_h_no_z[0]>0):
                alpha = -alpha
           #calculate rollshift beta to align manipulator with desired rollorientation
            gp_dir = gp2-gp1 #gp_dir is the direction between the two grasp points
            if gp_dir[0] < 0:
                gp_dir = gp1-gp2 
            beta = arccos(dot(gp_dir,y_min_dir)/(linalg.norm(gp_dir)))
            new_roll_angle = self.robot.GetDOFValues()[4]-alpha-(beta+pi/2)
           
            #bring new value for "roll"-joint into limit range
            if (new_roll_angle <= -pi):
                new_roll_angle += 2*pi
            elif (new_roll_angle > pi):
                new_roll_angle -= 2*pi 

            self.roll_angle = new_roll_angle
            #self.robot.SetDOFValues([new_roll_angle],[4])
            #self.robot.GetController().SetDesired(self.robot.GetDOFValues())
            #self.env.UpdatePublishedBodies()
            self.solution_grasp_with_roll = hstack((self.solution_grasp[0:4],array([self.roll_angle])))
            self.solution_pre_grasp_with_roll = hstack((self.solution_pre_grasp[0:4],array([self.roll_angle])))

      self.execute_path()
      #h=self.env.drawlinestrip(array([list(gp1),list(gp1+5.5*(gp2-gp1))]),7)
      #time.sleep(0.2)
      h=None
        
        # Publish Joints
      str_jointvalues = str(solutions[i][0])+" "+str(solutions[i][1])+" "+str(solutions[i][2])+" "+str(solutions[i][3])+" "+str(solutions[i][4])+" 0.5 0.5 "
      self.jointValues_pub.publish(String(str_jointvalues))





  def execute_path(self):
      self.move_arm_to_joint_values(self.PosBase,False)
      #assume arm is is base position and bring it in general pre-grasp position
      raw_input("Press enter to move arm in: General Pre Grasp Position")
      self.move_arm_to_joint_values(self.PosGeneralPreGrasp,False)
      raw_input("Press enter to open gripper")
      self.open_gripper()
      self.env.UpdatePublishedBodies()
      raw_input("Press enter to move arm to: Pre Grasp Position")
      self.move_arm_to_joint_values(self.solution_pre_grasp_with_roll,True)
  
      raw_input("Press enter to move physical arm into pre grasp position: publish joint values at /Hobbit/ArmJoints")  
      joints = self.rad_into_tics(self.solution_pre_grasp_with_roll)
      #joints=(-1160 , -1240 , -1160 , -1240 , 0)
      print joints
	
      self.publish_joint_state(joints)


      raw_input("joint values gepublished??         Press enter to move arm to grasp position")
      self.move_arm_to_joint_values(self.solution_grasp_with_roll,True)
      self.env.UpdatePublishedBodies()
      raw_input("Press enter to close gripper")
      self.close_gripper()
      raw_input("Press enter to move arm to Pre Grasp Position")
      self.move_arm_to_joint_values(self.solution_pre_grasp_with_roll,True)
      raw_input("Press enter to move arm to Delivery Grasp Position")
      self.move_arm_to_joint_values(self.PosDeliveryGrasp,False)
      raw_input("Press enter to open gripper")
      self.open_gripper()
      raw_input("Press enter to move arm to Base Grasp Position")
      self.move_arm_to_joint_valuesmove_arm_to_joint_values(self.PosBase,False)
      raw_input("Ende einer Runde")
      
      
if __name__ == "__main__":
    rospy.init_node('hobbit_grasp_control')
    hgc = HobbitGraspControl()
    hgc.init()
    rospy.spin()

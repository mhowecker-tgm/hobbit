#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'pickup_import'
DEBUG = False

import roslib
roslib.load_manifest(PKG)
import rospy
import numpy
from smach import Concurrence, Sequence, State, StateMachine
from smach_ros import MonitorState
from hobbit_user_interaction import HobbitEmotions, HobbitMMUI
from sensor_msgs.msg import PointCloud, PointCloud2
from geometry_msgs.msg import Point, PointStamped
import uashh_smach.util as util
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.head_move_import as head_move
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.arm_move_import as arm_move
import math, struct
from testdetector import TD
import tf
from tf.transformations import euler_from_quaternion, quaternion_matrix
from sensor_msgs.msg import PointField
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from hobbit_smach.ArmActionClient import ArmActionClient
import actionlib
import hobbit_msgs.msg
#import arm_simulation.GraspTrajectoryActionClient as grasptraj #
from GraspTrajectoryActionClient import GraspTrajectoryActionClient2

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

class DavidLookForObject(State):
    """
    This state is called after the robot moved to a position from where it
    should be able to observe the floor and find an object there.
    The calculations should result in a pose from where the robot is able to
    grasp the detected object.

    input_keys:
        cloud: sensor_msgs/PointCloud2
        goal_position_x: Needed to be able to write in the
        classes scope (SMACH thingy)
        goal_position_y: Needed to be able to write in the
        classes scope (SMACH thingy)
        goal_position_yaw: Needed to be able to write in the
        classes scope (SMACH thingy)

    output_keys:
        goal_position_x: x coordinate of the pose the robot should move to
        (world coordinates.)
        goal_position_y y coordinate of the pose the robot should move to
        (world coordinates.)
        goal_position_yaw: rotation of the pose the robot should move to
        (world coordinates.)
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'],
            input_keys=['cloud', 'goal_position_x', 'goal_position_y', 'goal_position_yaw'],
            output_keys=['goal_position_x', 'goal_position_y', 'goal_position_yaw']
        )
	self.listener = tf.TransformListener()
	self.pubClust = rospy.Publisher("/objectclusters", PointCloud2)
	self.rec = TD()
        self.restrictfind = False
        self.robotDistFromGraspPntForGrasping = 0.43
        self.robotOffsetRotationForGrasping = 0.06+math.pi/2.0

    def findobject(self, ud):
	print "===> pickup_import.py: DavidLookForObject.findobject()"
        #random.seed(rospy.get_time())
        pointcloud = ud.cloud
	clusters = self.rec.findObjectsOnFloor(pointcloud, [0,0,0,0])
        print "number of object clusters on floor found: ", len(clusters)

        for cluster in clusters:
            self.pc = cluster
	    print " ==============> publish cluster"
	    #self.pubClust.publish(cluster)
	    #self.pubClust.publish(pointcloud)					#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! new 31.7.2014
            if self.isGraspableObject():
                    self.pubClust.publish(cluster)
                    #self.showMMUI_Info("T_CF_I_FOUND_OBJECT_ON_FLOOR","1")
                    #print "findobject(): cluster saved and published"
                    return True

        print "findobect(): no graspable object found"
        return False



    def execute(self, ud):
	print "===> pickup_import.py: DavidLookForObject.execute()"
        if self.preempt_requested():
            return 'preempted'
        # TODO: David please put the pose calculations in here
        # The acquired point cloud is stored in ud.cloud
        # (sensor_msgs/PointCloud2)
        # The head will already be looking down to the floor.



        if self.findobject(ud):

            (robot_x, robot_y, robot_yaw) = util.get_current_robot_position(frame='/map')
            posRobot = [robot_x, robot_y] #Bajo, please fill in
	    print "actual robot position: ", posRobot
	    print "graspable center of cluster: ", self.graspable_center_of_cluster_wcs  #center of graspable point cluster
            robotApproachDir = [self.graspable_center_of_cluster_wcs.point.x - posRobot[0], self.graspable_center_of_cluster_wcs.point.y - posRobot[1]]
            robotApproachDir = [robotApproachDir[0]/numpy.linalg.norm(robotApproachDir), robotApproachDir[1]/numpy.linalg.norm(robotApproachDir)]       #normalized
	    print "normalized robot approach direction: ", robotApproachDir


            ud.goal_position_x = self.graspable_center_of_cluster_wcs.point.x - self.robotDistFromGraspPntForGrasping * robotApproachDir[0]
            ud.goal_position_y = self.graspable_center_of_cluster_wcs.point.y - self.robotDistFromGraspPntForGrasping * robotApproachDir[1]
            ud.goal_position_yaw = math.atan2(robotApproachDir[1],robotApproachDir[0]) + self.robotOffsetRotationForGrasping #can be negative!  180/math.pi*math.atan2(y,x) = angle in degree of vector (x,y)
	    print "robotDistFromGraspPntForGrasping: ", self.robotDistFromGraspPntForGrasping
	    print "robotOffsetRotationForGrasping: ", self.robotOffsetRotationForGrasping
	    print "robot goal position:"
	    print "ud.goal_position_x:   ", ud.goal_position_x
	    print "ud.goal_position_y:   ", ud.goal_position_y
	    print "ud.goal_position_yaw:   ", ud.goal_position_yaw


            return 'succeeded'
        else:
            return 'failed'




    #checks if object is suitable for grasping for camera center/down (later extension e.g. check distance to wall)
    def isGraspableObject(self):
        print "\n ===> F(): isGraspableObject start"

        print "trying to get tf transform"
        while True:
            try:
                (trans,rot) = self.listener.lookupTransform('/headcam_rgb_optical_frame', '/map', rospy.Time(0))
                print trans,rot
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "- isGraspableObject(): tf transform /headcam_rgb_optical_frame to /map not found"
                rospy.sleep(1)
                continue

        m = self.getCenterOfCluster()
        print "center of cluster", m
        p = PointStamped()
        p.header.frame_id = '/headcam_rgb_optical_frame'
        p.point.x = m[0]
        p.point.y = m[1]
        p.point.z = m[2]

        pnt_wcs = self.listener.transformPoint('/map', p)           #center of cluster in world coordinate system
        pnt_rcs = self.listener.transformPoint('/base_link', p)     #center of cluster in robot coordinate system

        #print "============== center of cluster in camera coordinate system: ", p
        print "============== center of cluster in rcs:                    : ", pnt_rcs
        print "============== center of cluster in wcs:                    : ", pnt_wcs

        isgraspable = self.ispossibleobject(pnt_rcs)
        if isgraspable:
            self.pc_rcs = self.transformPointCloud('/base_link',self.pc)
        self.graspable_center_of_cluster_wcs = pnt_wcs
        return isgraspable



    def transformPointCloud(self, target_frame, point_cloud):
        """
        :param target_frame: the tf target frame, a string
        :param ps: the sensor_msgs.msg.PointCloud2 message
        :return: new sensor_msgs.msg.PointCloud2 message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs PoseStamped message to frame target_frame, returns a new PoseStamped message.
        """
	print "===> pickup_import.py: DavidLookForObject.transformPointCloud()"
        r = PointCloud()
        r.header.stamp = rospy.Time.now() #point_cloud.header.stamp
        r.header.frame_id = target_frame
        #r.channels = point_cloud.channels

        #get points from pc2
        fmt = self._get_struct_fmt(point_cloud)
        narr = list()
        offset = 0
        for i in xrange(point_cloud.width * point_cloud.height):
            p = struct.unpack_from(fmt, point_cloud.data, offset)
            offset += point_cloud.point_step
            narr.append(p[0:3])

        while True:
            try:
                t = rospy.Time(0)
                point_cloud.header.stamp = t
                (trans,rot) = self.listener.lookupTransform('/headcam_rgb_optical_frame', target_frame, rospy.Time(0))
                print trans,rot
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "isGraspableObject(): tf transform /headcam_rgb_optical_frame to ", target_frame," not found"
                rospy.sleep(1)
                continue
        mat44 = self.listener.asMatrix(target_frame, point_cloud.header)
        #print mat44

        def xf(p):
            #xyz = tuple(numpy.dot(mat44, numpy.array([p.x, p.y, p.z, 1.0])))[:3]
            xyz = tuple(numpy.dot(mat44, numpy.array([p[0], p[1], p[2], 1.0])))[:3]
            return Point(*xyz)
        #r.points = [xf(p) for p in point_cloud.points]
        r.points = [xf(p) for p in narr]
        return r



    def ispossibleobject(self, pnt):    #pnt is in rcs
        print "===> pickup_import.py: DavidLookForObject.ispossibleobject()"
        #criteria for valid objects
        x_min = -0.5
        x_max = 1.0
        y_min = -1.0
        y_max = 0.5
        z_min = 0.0
        z_max = 0.25	#only that high because of buggy camera

        if (self.restrictfind): #only search for objects where grasped object was lying
            x_min = 0.15
            x_max = 0.45
            y_min = -0.45
            y_max = -0.17
            z_min = 0.01
            z_max = 0.25	#only that high because of buggy camera

        #print "pnt.point.x: ", pnt.point.x
        #print "pnt.point.x *2: ", 2*pnt.point.x
        if (pnt.point.x > x_min and pnt.point.x < x_max and pnt.point.y > y_min and pnt.point.y < y_max and pnt.point.z > z_min and pnt.point.z < z_max):
            print "ispossibleobject(): object ACCEPTED"
            return True
        else:
            print "ispossibleobject(): object DENIED (x,y,z coordinates of center of cluster not in acceptable range)"
            return False



    def getCenterOfCluster(self):
        print "===> pickup_import.py: DavidLookForObject.getCenterOfCluster()"
        #extract points from PointCloud2 in python....
        fmt = self._get_struct_fmt(self.pc)
        narr = list()
        offset = 0
        for i in xrange(self.pc.width * self.pc.height):
            p = struct.unpack_from(fmt, self.pc.data, offset)
            offset += self.pc.point_step
            narr.append(p[0:3])

        a = numpy.asarray(narr)
        pcmean = numpy.mean(a, axis=0)
        #print "mean", pcmean
        amin = numpy.min(a, axis=0)
        #print "amin", amin
        amax = numpy.max(a,axis=0)
        #print "amax", amax

        #print (amax[0]+amin[0])/2.0
        #print (amax[1]+amin[1])/2.0
        #print (amax[2]+amin[2])/2.0

        return [(amax[0]+amin[0])/2.0, (amax[1]+amin[1])/2.0, (amax[2]+amin[2])/2.0]



    def getRollForPointCloud(self):
	print "===> pickup_import.py: DavidLookForObject.getRollForPointCloud()"
        #finds and return good roll angle (assuming its good to grasp where object is slim)
        # returned angle has to be added to cf_pregrasp and cf_finalgrasp
        print "start of getRollForPointCloud"
        bestrollangle = 0
        mindiff_x = 1000
        alpha = 15*numpy.pi/180 #rad rotation angle (per step) (mathematical pos. direction)
        bestangle = 0
        #Rotation matrix R
        rotmat = numpy.array([[ numpy.cos(alpha), -numpy.sin(alpha)], [numpy.sin(alpha), numpy.cos(alpha)]])

        narr = list()
        for p in self.pc_rcs.points:
            narr.append([p.x, p.y])

        a = numpy.asarray(narr)

        for i in range(12):
            #calculate rotated pointcloud (rotate iterativly)
            a = numpy.transpose(numpy.dot(rotmat,numpy.transpose(a)))
            #get min point distance in x direction (mindiff_x)
            amin = numpy.min(a, axis=0)
            print "amin", amin
            amax = numpy.max(a,axis=0)
            print "amax", amax
            print "=========================== x abstand ==========================="
            print "x-dif: ",(amax[0]-amin[0])
            if (amax[0]-amin[0] < mindiff_x):
                mindiff_x = amax[0]-amin[0]
                bestangle = (i+1)*alpha
                print "new best angle:", bestangle

        return bestangle


    def _get_struct_fmt(self, cloud, field_names=None):
        #print cloud
        fmt = '>' if cloud.is_bigendian else '<'
        offset = 0
        for field in (f for f in sorted(cloud.fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in _DATATYPES:
                print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
            else:
                datatype_fmt, datatype_length = _DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length
        return fmt




class DavidLookingPose(State):
    """
    This state should handle the following task.
    Given the data from the pointing gesture (x,y,z,vectorX, vectorY, vectorZ)
    a Pose is calculated to which the robot will then navigate. This pose has
    to be stored inside the userdata output keys.

    input_keys:
        pointing_msg: rgbd_acquisition/PointEvents
        goal_position_x: Needed to be able to write in the
        classes scope (SMACH thingy)
        goal_position_y: Needed to be able to write in the
        classes scope (SMACH thingy)
        goal_position_yaw: Needed to be able to write in the
        classes scope (SMACH thingy)

    output_keys:
        goal_position_x: x coordinate of the pose the robot should move to
        (world coordinates.)
        goal_position_y y coordinate of the pose the robot should move to
        (world coordinates.)
        goal_position_yaw: rotation of the pose the robot should move to
        (world coordinates.)
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'],
            input_keys=['pointing_msg', 'goal_position_x', 'goal_position_y', 'goal_position_yaw'],
            output_keys=['goal_position_x', 'goal_position_y', 'goal_position_yaw']
        )
	self.listener = tf.TransformListener()
	self.pointing_gesture_pub = rospy.Publisher("pointing_gesture_pub", Marker)
	self.obj_marker = rospy.Publisher('obj_marker_array', MarkerArray)
	self.test_pub = rospy.Publisher('test_pub', String)

    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        print(ud.pointing_msg)
        # TODO: David please put the pose calculations in here


    #def savePointingDirection(self, msg):
        mm2m = 1000
        robotDistFromGraspPnt = 0.6 #in meters
        robotOffsetRotationForLooking = math.pi/3

        self.pointingDirCCS = [float(ud.pointing_msg.x)/mm2m,float(ud.pointing_msg.y)/mm2m,float(ud.pointing_msg.z)/mm2m,float(ud.pointing_msg.vectorX)/mm2m,float(ud.pointing_msg.vectorY)/mm2m,float(ud.pointing_msg.vectorZ)/mm2m]
        #calculate hand wrist point in robot coordinate system
        p = PointStamped()
        p.header.frame_id = '/headcam_rgb_optical_frame'
        p.point.x = self.pointingDirCCS[0]
        p.point.y = self.pointingDirCCS[1]
        p.point.z = self.pointingDirCCS[2]
	print "===================================================> p.point.x:", p.point.x
        pspWCS = self.listener.transformPoint('/map', p) #pspWCS: PointingStartPoint of the pointing gesture, i.e. the position of the shoulder in WCS (world coordinate system)
        print "shoulder coordinates in world coordinate system: ", pspWCS
        #calculate pointing vector in world coordinate system
        target_frame = "/map"
        pvecWCS = (0,0,0)
        while True:
            try:
                t = rospy.Time(0)

                #point_cloud.header.stamp = t
                (trans,rot) = self.listener.lookupTransform('/headcam_rgb_optical_frame', target_frame, rospy.Time(0))
		#print "rot1: ", rot
		rot = quaternion_matrix(rot)[0:3,0:3]
		#print "trans: ", trans
		#rot[0:3,3]=trans
		#print "rot quat plus trans", rot
                pvec = (self.pointingDirCCS[3],self.pointingDirCCS[4],self.pointingDirCCS[5])
		pvec = (pvec[0]/numpy.linalg.norm(pvec),pvec[1]/numpy.linalg.norm(pvec),pvec[2]/numpy.linalg.norm(pvec))  #normalize
		pvecWCS = numpy.dot(pvec,rot)
		print "======================================================="
		print "pvec (CCS, normalized): ", pvec # (CCS)
		#print "roation matrix: ", rot
		print "Pointing Vector in WCS: ", pvecWCS


                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "savePointingDirection(), calc pvec in RCS: tf transform /headcam_rgb_optical_frame to ", target_frame," not found"
                rospy.sleep(0.1)
                continue


	(robot_x, robot_y, robot_yaw) = util.get_current_robot_position(frame='/map')
        posRobot = [robot_x, robot_y] #Bajo, please fill in
        gpOnFloor = self.calcIntersectionPointingDirWithFloor(pspWCS, pvecWCS)
        if gpOnFloor == None:
		return 'preempted'



	#publish marker for visualizing the pointing point on floor
	print "========================> now the point on floor is published as marker "
	obj_markerArray = MarkerArray()

	pose = Pose()
        pose.position.x = gpOnFloor[0]
        pose.position.y = gpOnFloor[1]
        pose.position.z = gpOnFloor[2]
        pose.orientation.w = 1
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0

	ellipse = Marker()
	ellipse.id = 1
    	ellipse.header.frame_id = "/map"
    	ellipse.header.stamp = rospy.Time()
	ellipse.type = ellipse.CYLINDER
	ellipse.action = ellipse.ADD
    	ellipse.pose = pose
    	ellipse.scale.x = 0.1
    	ellipse.scale.y = 0.2
    	ellipse.scale.z = 0.1
    	ellipse.color.a = 1.0
    	ellipse.color.r = 1.0
    	ellipse.color.g = 0.5
    	ellipse.color.b = 1.0

        #marker1 = Marker()
        #marker1.id = 1
        #marker1.header.frame_id = "/map"
        #marker1.type = marker1.SPHERE
        #marker1.action = marker1.ADD
        #marker1.scale.x = 0.05
        #marker1.scale.y = 0.05
        #marker1.scale.z = 0.05
        #marker1.color.a = 1.0
        #marker1.color.r = 1.0
        #marker1.color.g = 1.0
        #marker1.color.b = 0.0

        #marker1.pose.orientation = pose.orientation
        #marker1.pose.position = pose.position

        #obj_markerArray.markers.append(marker1)
	obj_markerArray.markers.append(ellipse)

    	# Publish the Marker
    	#self.pointing_gesture_pub.publish(ellipse)

	#obj_markerArray.markers.append(ellipse)
	self.obj_marker.publish(obj_markerArray)


	print "grasp point on floor: X:  ", gpOnFloor[0], "   Y:  ", gpOnFloor[1], "   Z:  ", gpOnFloor[2]
        robotApproachDir = [gpOnFloor[0] - posRobot[0], gpOnFloor[1] - posRobot[1]]
        robotApproachDir = [robotApproachDir[0]/numpy.linalg.norm(robotApproachDir), robotApproachDir[1]/numpy.linalg.norm(robotApproachDir)]       #normalized


        #self.position2viewobject(gpOnFloor) #move hobbit near object

        ud.goal_position_x = gpOnFloor[0] - robotDistFromGraspPnt * robotApproachDir[0]
        ud.goal_position_y = gpOnFloor[1] - robotDistFromGraspPnt * robotApproachDir[1]
        ud.goal_position_yaw = math.atan2(robotApproachDir[1],robotApproachDir[0]) + robotOffsetRotationForLooking #can be negative!  180/math.pi*math.atan2(y,x) = angle in degree of vector (x,y)
        return 'succeeded'



    def calcIntersectionPointingDirWithFloor(self, pspWCS, pvecWCS):
        #calculates intersection of pointing vector (pvecWCS)
        #from shoulder point (pspWCS) with ground floor (pspWCS + k * pvecWCS = (a,b,0) => calculate k first, than values a,b

        k = -pspWCS.point.z/pvecWCS[2]
        if k < 0:
            print "ERROR, pointing direction must be downwards!!"
            print "k = ", k
            return None
        gpOnFloor = [pspWCS.point.x+k*pvecWCS[0],pspWCS.point.y+k*pvecWCS[1],0]
        return gpOnFloor


#class DavidCalcGraspPose(State):
#    """
#    This state should handle the following task.
#    Given the Pose data given in the object_pose a Pose is calculated to
#    which the robot will then navigate. This pose has to be stored inside
#    the userdata output keys.
#    """
#    def __init__(self):
#        State.__init__(
#            self,
#            outcomes=['succeeded', 'aborted', 'preempted'],
#            input_keys=['object_pose', 'goal_position_x', 'goal_position_y', 'goal_position_yaw'],
#            output_keys=['goal_position_x', 'goal_position_y', 'goal_position_yaw']
#        )
#    def execute(self, ud):
#        if self.preempt_requested():
#            return 'preempted'
#        print(ud.object_pose)
#        # TODO: David please put the pose calculations in here
#
#        ud.goal_position_x = 0
#        ud.goal_position_y = 0
#        ud.goal_position_yaw = 0
#        return 'succeeded'





class CalcGrasppointsActionClient():

    last_feedback = None

    def __init__(self):
        self.calc_grasppoints_client = actionlib.SimpleActionClient("calc_grasppoints_svm_action_server", hobbit_msgs.msg.CalcGraspPointsServerAction)


    # use this method with a String cmd to send goal to the ArmActionServer and return its result value
    def calc_grasppoints_action_client(self, cmd):

        print "type of cmd: ", type(cmd)
        # Waits until the action server has started up and started
        # listening for goals.
        print "CalcGrasppointsActionClient: wait_for_server()"
        self.calc_grasppoints_client.wait_for_server()
        print "server found!"


        print "CalcGrasppointsActionClient: create goal"
        # Creates a goal to send to the action server.
        goal = hobbit_msgs.msg.CalcGraspPointsServerGoal(input_pc=cmd)

        #print "CalcGrasppointsActionClient: send goal"
        # Sends the goal to the action server.
        self.calc_grasppoints_client.send_goal(goal, feedback_cb=self.feedback_cb)

        print "wait for result"
        # Waits for the server to finish performing the action.
        self.calc_grasppoints_client.wait_for_result()


        # Prints out the result of executing the action
        returnval = self.calc_grasppoints_client.get_result()  #
        print "returnval from calc_grasppoints: ", returnval
        return returnval

    def feedback_cb(self, feedback):
        print "feedback_cb executed!"
        #print "feedback type: ", (type) (feedback)
        self.last_feedback = feedback
        print "==========> feedback: ", self.last_feedback





class DavidPickingUp(State):
    """
    This state moves the arm from the PreGrasping position to the object,
    closes the gripper and returns 'succeeded' afterwards.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'],
            input_keys=['cloud']
        )
        self.arm_client = ArmActionClient()
        self.listener = tf.TransformListener()
        self.pubClust = rospy.Publisher("/objectclusters", PointCloud2)
        self.rec = TD()
        self.restrictfind = True
        self.calc_graspoints_client = CalcGrasppointsActionClient()


    def execute(self, ud):
        print "===> pickup_import.py: DavidPickingUp.execute()"
        if self.preempt_requested():
            return 'preempted'
        # TODO: David please add the grasping in here

        if self.findobject(ud):

            print "DavidPickingUp.execute: findobject was successful"
            #self.pc_rcs is available () => calc_grasp_points.cpp (AS) and get back grasp pose definition
            #todo:
            # 1) call calc_grasp_points AS
            # 2) call openrave trajectory AS
            # 3) call Arm AS (arm client already included in code)
            #grasp pose definition => simulation => execution

            # 1) call calc_grasppoints_action_server/client (calc_aS(self.pc_rcs)) and receive a grasp_representation in the format (string):
            # "(0)eval_val (1)gp1_x (2)gp1_y (3)gp1_z (4)gp2_x (5)gp2_y (6)gp2_z (7)ap_vec_x (8)ap_vec_y (9)ap_vec_z (10)gp_center_x (11)gp_center_y (12)gp_center_z (13)roll"
            gp_representation = self.calc_graspoints_client.calc_grasppoints_action_client(ud.cloud)#self.pc_rcs) #ud.cloud) 12.12.2014

            # 2) call GraspFromFloorTrajectoryActionServer/Client to receive a trajectory (that will be later directly executed) by calling the ArmActionServer/client
            #daviddavid
            grasp_traj_ac = GraspTrajectoryActionClient()
            print grasp_traj_ac
            
            """ arm makes fix grasping-from-floor-movement (old)
            #OPEN GRIPPER
            res = self.arm_client.arm_action_client(String ("SetOpenGripper"))
            #clear buffer
            res = self.arm_client.SetClearPosBuffer()
            #PositionsForInterpolation(armactionserver) Intermediate Floor GraspPOS4 (= Grasp at floor)
            cmd = String ("SetPositionsForInterpolation 78 55 78 150 90 4")
            res = self.arm_client.arm_action_client(cmd)
            #PositionsForInterpolationReady
            cmd = String ("SetPositionsForInterpolationReady")
            res = self.arm_client.arm_action_client(cmd)
        	#    raw_input("press key to move arm after position interpolation")
            cmd = String("SetStartInterpolation")
            res = self.arm_client.arm_action_client(cmd)
        	#CLOSE GRIPPER
            cmd = String ("SetCloseGripper")
            res = self.arm_client.arm_action_client(cmd)
        	#clear buffer
            res = self.arm_client.SetClearPosBuffer()
        	#PositionsForInterpolation(armactionserver) POS1=PreGraspFromFloor
            cmd = String ("SetPositionsForInterpolation 69.7 43.4 86.3 134.3 107.6 4")
            res = self.arm_client.arm_action_client(cmd)
        	#PositionsForInterpolationReady (armactionserver)")
            cmd = String ("SetPositionsForInterpolationReady")
            res = self.arm_client.arm_action_client(cmd)
        	#raw_input("press key to move arm after position interpolation")
            cmd = String("SetStartInterpolation")
            res = self.arm_client.arm_action_client(cmd)
            """

        return 'succeeded'

#===========
    def findobject(self, ud):
        print "===> pickup_import.py: DavidPickingUp.findobject()"
        pc_ccs = ud.cloud   #point cloud in camera coordinate system
        #raw_input(" ==============> publish cluster that findObjectsOnFloor is receiving from pickup_import.py")
        self.pubClust.publish(ud.cloud)
        clusters = self.rec.findObjectsOnFloor(pc_ccs, [0,0,0,0]) #before: pointcloud instead of pc_ccs
        print "number of object clusters on floor found: ", len(clusters)

        for cluster in clusters:
            self.pc = cluster
            print " ==============> publish cluster"
            self.pubClust.publish(cluster)
            if self.isGraspableObject():
                return True

        print "findobect(): no graspable object found"
        return False


    #checks if object is suitable for grasping
    def isGraspableObject(self):
        print "\n ===> F(): DavidPickingUp.isGraspableObject() start (pickup_import.py)"

        print "trying to get tf transform"
        while True:
            try:
                (trans,rot) = self.listener.lookupTransform('/headcam_rgb_optical_frame', '/base_link', rospy.Time(0))
                print trans,rot
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "- isGraspableObject(): tf transform /headcam_rgb_optical_frame to /base_link not found"
                rospy.sleep(1)
                continue

        m = self.getCenterOfCluster()
        print "center of cluster", m
        p = PointStamped()
        p.header.frame_id = '/headcam_rgb_optical_frame'
        p.point.x = m[0]
        p.point.y = m[1]
        p.point.z = m[2]

        pnt_rcs = self.listener.transformPoint('/base_link', p)     #center of cluster in robot coordinate system

        #print "============== center of cluster in camera coordinate system: ", p
        print "============== center of cluster in rcs:                    : ", pnt_rcs

        isgraspable = self.ispossibleobject(pnt_rcs)
        if isgraspable:
            self.pc_rcs = self.transformPointCloud('/base_link',self.pc)
            self.pc_ccs = self.pc
        return isgraspable


    def getCenterOfCluster(self):
        print "===> pickup_import.py: DavidPickingUp.getCenterOfCluster()"
        #extract points from PointCloud2 in python....
        fmt = self._get_struct_fmt(self.pc)
        narr = list()
        offset = 0
        for i in xrange(self.pc.width * self.pc.height):
            p = struct.unpack_from(fmt, self.pc.data, offset)
            offset += self.pc.point_step
            narr.append(p[0:3])
            

        df = narr.__len__(); #length of point cloud
        print "===> pickup_import.py: DavidPickingUp.getCenterOfCluster() LENGTH OF POINTCLOUD:", df
        a = numpy.asarray(narr)
        pcmean = numpy.mean(a, axis=0)
        amin = numpy.min(a, axis=0)
        amax = numpy.max(a,axis=0)

        return [(amax[0]+amin[0])/2.0, (amax[1]+amin[1])/2.0, (amax[2]+amin[2])/2.0]



    def transformPointCloud(self, target_frame, point_cloud):
        """
        :param target_frame: the tf target frame, a string
        :param ps: the sensor_msgs.msg.PointCloud2 message
        :return: new sensor_msgs.msg.PointCloud2 message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs PoseStamped message to frame target_frame, returns a new PoseStamped message.
        """
        print "===> pickup_import.py: DavidLookForObject.transformPointCloud()"
        r = PointCloud()
        r.header.stamp = rospy.Time.now() #point_cloud.header.stamp
        r.header.frame_id = target_frame
        #r.channels = point_cloud.channels

        #get points from pc2
        fmt = self._get_struct_fmt(point_cloud)
        narr = list()
        offset = 0
        for i in xrange(point_cloud.width * point_cloud.height):
            p = struct.unpack_from(fmt, point_cloud.data, offset)
            offset += point_cloud.point_step
            narr.append(p[0:3])

        while True:
            try:
                t = rospy.Time(0)
                point_cloud.header.stamp = t
                (trans,rot) = self.listener.lookupTransform('/frame', target_frame, rospy.Time(0))   #12.12.2014: /headcam_rgb_optical_frame => /frame => FORTH changed it!
                print trans,rot
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "isGraspableObject(): tf transform /frame to ", target_frame," not found"
                rospy.sleep(1)
                continue
        mat44 = self.listener.asMatrix(target_frame, point_cloud.header)
        #print mat44

        def xf(p):
            #xyz = tuple(numpy.dot(mat44, numpy.array([p.x, p.y, p.z, 1.0])))[:3]
            xyz = tuple(numpy.dot(mat44, numpy.array([p[0], p[1], p[2], 1.0])))[:3]
            return Point(*xyz)
        #r.points = [xf(p) for p in point_cloud.points]
        r.points = [xf(p) for p in narr]
        return r



    def _get_struct_fmt(self, cloud, field_names=None):
        #print cloud
        fmt = '>' if cloud.is_bigendian else '<'
        offset = 0
        for field in (f for f in sorted(cloud.fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in _DATATYPES:
                print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
            else:
                datatype_fmt, datatype_length = _DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length
        return fmt

    def ispossibleobject(self, pnt):    #pnt is in rcs
        print "===> pickup_import.py: DavidPickingUp.ispossibleobject()"
        #criteria for valid objects  => has to me more accurate!
        x_min = -0.5
        x_max = 1.0
        y_min = -1.0
        y_max = 0.5
        z_min = 0.0
        z_max = 0.25    #only that high because of buggy camera

        if (self.restrictfind): #only search for objects where grasped object was lying
            x_min = -0.15
            x_max = 0.20
            y_min = -0.55
            y_max = -0.23
            z_min = 0.01
            z_max = 0.20    #only that high because of buggy camera

        #print "pnt.point.x: ", pnt.point.x
        if (pnt.point.x > x_min and pnt.point.x < x_max and pnt.point.y > y_min and pnt.point.y < y_max and pnt.point.z > z_min and pnt.point.z < z_max):
            print "ispossibleobject(): object ACCEPTED"
            return True
        else:
            print "ispossibleobject(): object DENIED (x,y,z coordinates of center of cluster not in acceptable range)"
            return False




def point_cloud_cb(ud, msg):
    print('point cloud received')
    ud.cloud = msg
    return False



class DavidCheckGrasp(State):
    """
    This state has to check if the grasping was successful or not.
    Return 'succeeded' if it was, 'aborted' if it was not.
    Returns 'preempted' on request.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        # TODO: David please add the check of the grasping in here

        return 'succeeded'


def child_term_cb(outcome_map):
    return False


def sayPointingGestureNotDetected1():
    cc = Concurrence(
        outcomes=['yes', 'no', 'preempted', 'failed'],
        default_outcome='no',
        child_termination_cb=child_term_cb,
        outcome_map={'yes': {'EMO_SAD': 'succeeded',
                             'SAY_POINTING_NOT_DETECTED': 'yes'},
                     'no': {'SAY_POINTING_NOT_DETECTED': 'no'}}
    )

    with cc:
        Concurrence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Concurrence.add(
            'SAY_POINTING_NOT_DETECTED',
            #speech_output.askYesNo(question='T_PU_PointingGestureNotDetected1')
            # speech_output.askYesNo(question='PointingGestureNotDetected1')
            HobbitMMUI.AskYesNo(question='T_PU_PointingGestureNotDetected1')
        )
    return cc


def sayPointingGestureNotDetected2():
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_SAD': 'succeeded',
                                   'SAY_POINTING_NOT_DETECTED': 'succeeded'}}
    )

    with cc:
        Concurrence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Concurrence.add(
            'SAY_POINTING_NOT_DETECTED',
            #speech_output.sayText(info='What the hell')
            speech_output.sayText(info='T_PU_PointingGestureNotDetected2')
        )
    return cc


def sayStartLooking():
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_HAPPY': 'succeeded',
                                   'SAY_START_LOOKING': 'succeeded'}}
    )

    with cc:
        Concurrence.add(
            'EMO_HAPPY',
            HobbitEmotions.ShowEmotions(emotion='HAPPY', emo_time=4)
        )
        Concurrence.add(
            'SAY_START_LOOKING',
            speech_output.sayText(info='T_PU_StartLooking')
        )
    return cc


def sayObjectNotDetected1():
    cc = Concurrence(
        outcomes=['yes', 'no', 'failed', 'preempted'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'yes': {'EMO_SAD': 'succeeded',
                             'SAY_OBJECT_NOT_DETECTED': 'yes'},
                     'no': {'SAY_OBJECT_NOT_DETECTED': 'no'}}
    )

    with cc:
        Concurrence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Concurrence.add(
            'SAY_OBJECT_NOT_DETECTED',
            # speech_output.askYesNo(question='T_PU_ObjectNotDetected1')
            HobbitMMUI.AskYesNo(question='T_PU_ObjectNotDetected1')
        )
    return cc


def sayObjectNotDetected2():
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_SAD': 'succeeded',
                                   'SAY_OBJECT_NOT_DETECTED': 'succeeded'}}
    )

    with cc:
        Concurrence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Concurrence.add(
            'SAY_OBJECT_NOT_DETECTED',
            speech_output.sayText(info='T_PU_ObjectNotDetected2')
        )
    return cc


def sayObjectFoundRepositioning():
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_VERY_HAPPY': 'succeeded',
                                   'SAY_OBJECT_FOUND': 'succeeded'}}
    )

    with cc:
        Concurrence.add(
            'EMO_VERY_HAPPY',
            HobbitEmotions.ShowEmotions(emotion='VERY_HAPPY', emo_time=4)
        )
        Concurrence.add(
            'SAY_OBJECT_FOUND',
            speech_output.sayText(info='T_PU_ObjectFoundRepositioning')
        )
    return cc


def sayUnableToGraspObject():
    cc = Concurrence(
        outcomes=['yes', 'no', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'yes': {'EMO_SAD': 'succeeded',
                             'SAY_UNABLE_TO_GRASP_OBJECT': 'yes'},
                     'no': {'SAY_UNABLE_TO_GRASP_OBJECT': 'no'}}
    )

    with cc:
        Concurrence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Concurrence.add(
            'SAY_UNABLE_TO_GRASP_OBJECT',
            HobbitMMUI.AskYesNo(question='T_PU_UnableToGraspObject')
        )
    return cc


def sayTryToRemoveObject():
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_SAD': 'succeeded',
                                   'SAY_TRY_TO_REMOVE_OBJECT': 'succeeded'}}
    )

    with cc:
        Concurrence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Concurrence.add(
            'SAY_TRY_TO_REMOVE_OBJECT',
            speech_output.sayText(info='T_PU_TryToRemoveObject')
        )
    return cc


def sayDidNotPickupObject1():
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_SAD': 'succeeded',
                                   'SAY_DID_NOT_PICKUP_OBJECT': 'succeeded'}}
    )

    with cc:
        Concurrence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Concurrence.add(
            'SAY_DID_NOT_PICKUP_OBJECT',
            speech_output.sayText(info='T_PU_DidNotPickupObject1')
        )
    return cc


def sayDidNotPickupObject2():
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_SAD': 'succeeded',
                                   'SAY_DID_NOT_PICKUP_OBJECT': 'succeeded'}}
    )

    with cc:
        Concurrence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Concurrence.add(
            'SAY_DID_NOT_PICKUP_OBJECT',
            speech_output.sayText(info='T_PU_DidNotPickupObject2')
        )
    return cc


def sayPickedUpObject():
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_HAPPY': 'succeeded',
                                   'SAY_PICKED_UP_OBJECT': 'succeeded'}}
    )

    with cc:
        Concurrence.add(
            'EMO_HAPPY',
            HobbitEmotions.ShowEmotions(emotion='HAPPY', emo_time=4)
        )
        Concurrence.add(
            'SAY_PICKED_UP_OBJECT',
            speech_output.sayText(info='T_PU_PickedUpObject')
        )
    return cc


def sayRemoveObjectTakeObject():
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_HAPPY': 'succeeded',
                                   'SAY_REMOVE_OBJECT_TAKE_OBJECT': 'succeeded'}}
    )

    with cc:
        Concurrence.add(
            'EMO_HAPPY',
            HobbitEmotions.ShowEmotions(emotion='HAPPY', emo_time=4)
        )
        Concurrence.add(
            'SAY_REMOVE_OBJECT_TAKE_OBJECT',
            speech_output.sayText(info='T_PU_RemovedObjectTakeObject')
        )
    return cc


def getStartLooking():
    """
    Return SMACH Sequence that will let the robot give some speech output
    and moves the robot to a pose that should be good enough so that an
    object could be detected.
    """
    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        input_keys=['pointing_msg'],
        connector_outcome='succeeded'
    )
    if DEBUG:
        seq.userdata.goal_position_x = 0
        seq.userdata.goal_position_y = 0
        seq.userdata.goal_position_yaw = 0

    with seq:
        Sequence.add(
        'EMO_SAY_START_LOOKING',
        sayStartLooking()
        )
        Sequence.add(
            'CALCULATE_LOOKING_POSE',
            DavidLookingPose()
        )
        Sequence.add(
            'MOVE_TO_POSE',
            hobbit_move.goToPoseSilent(),
            transitions={'aborted': 'failed'},
            remapping={'x':'goal_position_x',
                       'y':'goal_position_y',
                       'yaw':'goal_position_yaw'}
        )
    return seq


def getEndPickupSeq():
    """
    Return a SMACH Sequence that gives feedback to the user, stores a grasped
    object on the tray and searches for the user.
    """
    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'EMO_SAY_PICKED_UP',
            sayPickedUpObject()
        )
        if not DEBUG:
            Sequence.add(
                'MOVE_ARM_TO_TRAY_AND_HOME',
                arm_move.goToTrayPosition()
            )
            #Sequence.add(
            #    'LOCATE_USER',
		#sayRemoveObjectTakeObject() # df 18.8.2014: delete this line again, only inserted to get code running!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                #Dummy()
            #)
        Sequence.add(
            'EMO_SAY_REMOVE_OBJECT',
            sayRemoveObjectTakeObject()
        )
        return seq


def getPickupSeq():
    """
    Return a SMACH Sequence
    """
    #seq = Sequence(
    #    outcomes=['succeeded', 'preempted', 'failed'],
    #    connector_outcome='succeeded'
    #)
    sm = StateMachine(
        outcomes=['succeeded', 'preempted', 'failed']
    )

    #with seq:
    with sm:
        if not DEBUG:
            StateMachine.add(
            'GET_POINT_CLOUD',
            MonitorState(
                '/headcam/depth_registered/points',
                PointCloud2,
                cond_cb=point_cloud_cb,
                max_checks=20,
                output_keys=['cloud']
            ),
            transitions={'valid': 'GET_POINT_CLOUD',
                         'invalid': 'GRASP_OBJECT',#'MOVE_ARM_TO_PRE_GRASP_POSITION',
                         'preempted': 'preempted'}
            )
            #StateMachine.add(
            #    'MOVE_ARM_TO_PRE_GRASP_POSITION',
            #    arm_move.goToPreGraspPosition(),
            #transitions={'succeeded': 'GRASP_OBJECT',
            #             'preempted': 'preempted',
            #             'failed': 'failed'}
            #)
        StateMachine.add(
            'GRASP_OBJECT',
            DavidPickingUp(),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'preempted',
                         'failed': 'failed'}
        )
        return sm
        #return seq

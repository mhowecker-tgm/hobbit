#!/usr/bin/python
# -*- coding: utf-8 -*-
from distance_to_obstacle.srv._distance_to_obstacle import distance_to_obstacle

PKG = 'hobbit_smach'
NAME = 'pickup_import'
DEBUG = False
SCENARIO='PickUp'

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
import hobbit_smach.speech_output_import as speech_output
import math, struct
from testdetector import TD
import tf
from tf.transformations import euler_from_quaternion, quaternion_matrix
from sensor_msgs.msg import PointField
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose
from hobbit_smach.ArmActionClient import ArmActionClient
import actionlib
import hobbit_msgs.msg
from hobbit_msgs.srv import *
import arm_simulation.GraspTrajectoryActionClient # as grasptraj #
#from arm_simulation import GraspTrajectoryActionClient
from table_object_detector.srv import *
from hobbit_msgs import MMUIInterface as MMUI
import hobbit_smach.logging_import as logging
from hobbit_msgs.msg import Event, Parameter

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
        obj_center_rcs

    output_keys:
        goal_position_x: x coordinate of the pose the robot should move to
        (world coordinates.)
        goal_position_y y coordinate of the pose the robot should move to
        (world coordinates.)
        goal_position_yaw: rotation of the pose the robot should move to
        (world coordinates.)
        obj_center_rcs: x,y coordinates of the center of graspable object found (point)
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'],
            input_keys=['cloud', 'goal_position_x', 'goal_position_y', 'goal_position_yaw','obj_center_rcs'],
            output_keys=['goal_position_x', 'goal_position_y', 'goal_position_yaw','obj_center_rcs']
        )
        self.listener = tf.TransformListener()
        self.pubClust = rospy.Publisher("/objectclusters", PointCloud2)
        self.rec = TD()
        self.robotDistFromGraspPntForGrasping = 0.51
        self.robotOffsetRotationForGrasping = 0.06+math.pi/2.0
        self.graspable_center_of_cluster_wcs = None
        self.graspable_center_of_cluster_rcs = None
        self.min_obj_to_mapboarder_distance = 0.30 #object has to be at least self.min_obj_to_mapboarder_distance cm away from next boarder in navigation map
        self.logger = logging.DoLogScenarioAndData()
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)
        

    def findobject(self, ud):
        print "===> pickup_import.py: DavidLookForObject.findobject() started"
        #random.seed(rospy.get_time())
        pointcloud = ud.cloud
        #df 19.3.2015 changed to small objects:
        #clusters = self.rec.findObjectsOnFloor(pointcloud, [0,0,0,0])
        clusters = self.rec.findObjectsOnFloorSmall(pointcloud, [0,0,0,0])
        print "===> pickup_import.py: findobect():number of object clusters on floor found: ", len(clusters)
        
        self.logger.execute("3106NOSPP Nr of Objects Segmented (prepos): "+str(len(clusters)))  #log nr of object segmented at pregrasp position

        nr_objects_segmented = len(clusters)
        i = 0
        for cluster in clusters:
            i = i+1
            self.pc = cluster
            print "===> pickup_import.py: findobect(): publish potential cluster on topic /pickup/objectclusters"
            #self.pubClust.publish(cluster)
            if self.isGraspableObject():
                    self.pubClust.publish(cluster)
                    self.logger.execute("3799OPPPP Object passed position check. Segmented Object Nr: " + str(i) + "/" + str(nr_objects_segmented))  #log nr of object that passed position check in prepos
                    #self.showMMUI_Info("T_CF_I_FOUND_OBJECT_ON_FLOOR","1")
                    #print "findobject(): cluster saved and published"
                    return True
            else:
                self.logger.execute("3698OFPPP Object failed position check. Segmented Object Nr: " + str(i) + "/" + str(nr_objects_segmented))  #log nr of object that passed position check

        print "===> pickup_import.py: findobect(): no graspable object found"
        self.logger.execute("3207RORPP Reason why Object(s) not accepted (prepos): " + "All " + str(nr_objects_segmented) + " segmented objects did not pass position check")
        return False

    def isObjectAwayFromMapBoarders(self):
        rospy.wait_for_service('distance_to_obstacle')
        try:
            distance_to_obstacle_service = rospy.ServiceProxy('distance_to_obstacle', distance_to_obstacle)
            input = distance_to_obstacle()  #data type for this service
            response = distance_to_obstacle_service(self.graspable_center_of_cluster_wcs.point)
            print "===> pickup_import.py: isObjectAwayFromMapBoarders(): object distance to map boarder: ", response.d
            dist = response.d
            if (dist > self.min_obj_to_mapboarder_distance):
                print "===> pickup_import.py: isObjectAwayFromMapBoarder(): object is far enough from map boarders."
                return True
            else:
                print "===> pickup_import.py: isObjectAwayFromMapBoarder(): object is to near to map obstacle! OBJECT DENIED"
                return False
            
        except rospy.ServiceException, e:
            print "===> pickup_import.py: isObjectAwayFromMapBoarder(): Service call distance_to_obstacle failed: %s"%e
            return False
                
                                
                        
    def execute(self, ud):
        print "===> pickup_import.py: DavidLookForObject.execute() started"
        if self.preempt_requested():
            return 'preempted'
        # TODO: David please put the pose calculations in here
        # The acquired point cloud is stored in ud.cloud
        # (sensor_msgs/PointCloud2)
        # The head will already be looking down to the floor.


        if self.findobject(ud):
            #object was found, is graspable (has correct dimension; is not near to fixed obstacle such as a wall)
            self.logger.execute("3409GOFPP Graspable Object found (prepos)")  #graspable object found (in preposition)
            
            (robot_x, robot_y, robot_yaw) = util.get_current_robot_position(frame='/map')
            posRobot = [robot_x, robot_y] #Bajo, please fill in
            print "===> pickup_import.py: DavidLookForObject.execute(): actual robot position: ", posRobot
            print "===> pickup_import.py: DavidLookForObject.execute(): graspable center of cluster: ", self.graspable_center_of_cluster_wcs  #center of graspable point cluster
            robotApproachDir = [self.graspable_center_of_cluster_wcs.point.x - posRobot[0], self.graspable_center_of_cluster_wcs.point.y - posRobot[1]]
            robotApproachDir = [robotApproachDir[0]/numpy.linalg.norm(robotApproachDir), robotApproachDir[1]/numpy.linalg.norm(robotApproachDir)]       #normalized
            print "===> pickup_import.py: DavidLookForObject.execute(): normalized robot approach direction: ", robotApproachDir


            ud.goal_position_x = self.graspable_center_of_cluster_wcs.point.x - self.robotDistFromGraspPntForGrasping * robotApproachDir[0]
            ud.goal_position_y = self.graspable_center_of_cluster_wcs.point.y - self.robotDistFromGraspPntForGrasping * robotApproachDir[1]
            ud.goal_position_yaw = math.atan2(robotApproachDir[1],robotApproachDir[0]) + self.robotOffsetRotationForGrasping #can be negative!  180/math.pi*math.atan2(y,x) = angle in degree of vector (x,y)
            #define center of found object in rcs
            ud.obj_center_rcs = self.graspable_center_of_cluster_rcs.point
            
            
            print "===> pickup_import.py: DavidLookForObject.execute(): robotDistFromGraspPntForGrasping: ", self.robotDistFromGraspPntForGrasping
            print "===> pickup_import.py: DavidLookForObject.execute(): robotOffsetRotationForGrasping: ", self.robotOffsetRotationForGrasping
            print "===> pickup_import.py: DavidLookForObject.execute(): robot goal position:"
            print "ud.goal_position_x:   ", ud.goal_position_x
            print "ud.goal_position_y:   ", ud.goal_position_y
            print "ud.goal_position_yaw:   ", ud.goal_position_yaw


            return 'succeeded'
        else:
            self.logger.execute("3510NOFPP No Graspable Object found (prepos)")  #log: no graspable object found (preposition)
            return 'failed' #object not found




    #checks if object is suitable for grasping for camera center/down (later extension e.g. check distance to wall)
    def isGraspableObject(self):
        print "\n ===> pickup_import.py: isGraspableObject() started"

        print "trying to get tf transform"
        while True:
            try:
                (trans,rot) = self.listener.lookupTransform('/headcam_rgb_optical_frame', '/map', rospy.Time(0))
                print trans,rot
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "===> pickup_import.py: isGraspableObject(): tf transform /headcam_rgb_optical_frame to /map not found"
                rospy.sleep(1)
                continue

        m = self.getCenterOfCluster()
        print "===> center of cluster", m
        p = PointStamped()
        p.header.frame_id = '/headcam_rgb_optical_frame'
        p.point.x = m[0]
        p.point.y = m[1]
        p.point.z = m[2]

        pnt_wcs = self.listener.transformPoint('/map', p)           #center of cluster in world coordinate system
        pnt_rcs = self.listener.transformPoint('/base_link', p)     #center of cluster in robot coordinate system

        #print "============== center of cluster in camera coordinate system: ", p
        print "===> center of cluster in rcs:                    : ", pnt_rcs
        print "===> center of cluster in wcs:                    : ", pnt_wcs
        
        #log object data: center pnt in wcs, center_pnt in rcs, segmented point cloud size
        self.logger.execute("3308ODSPP Object Data per Segmented Object (prepos): " + "Object center in WCS: " + str(pnt_wcs) + " Object center in RCS: "+ str(pnt_rcs) + " Number points of segmented objects: " + str(self.pc.height*self.pc.width))

        isgraspable = self.ispossibleobject(pnt_rcs)
        if isgraspable:
            self.pc_rcs = self.transformPointCloud('/base_link',self.pc)
            self.graspable_center_of_cluster_wcs = pnt_wcs
            self.graspable_center_of_cluster_rcs = pnt_rcs
            if self.isObjectAwayFromMapBoarders():
                return True
            else:
                self.logger.execute("3897OFDPP Object failed position check (prepos). Distance to wall to small.")
                return False #object to near to map boarder

        
        
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
                #print trans,rot
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "===> isGraspableObject() => transformPointCloud(): tf transform /headcam_rgb_optical_frame to ", target_frame," not found"
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
        print "===> pickup_import.py: DavidLookForObject.ispossibleobject() started"
        #criteria for valid objects
        x_min = -0.5
        x_max = 1.1
        y_min = -1.1
        y_max = 0.5
        z_min = 0.0
        z_max = 0.25	#only that high because of buggy camera

        if (pnt.point.x > x_min and pnt.point.x < x_max and pnt.point.y > y_min and pnt.point.y < y_max and pnt.point.z > z_min and pnt.point.z < z_max):
            print "===> ispossibleobject(): object ACCEPTED"
            return True
        else:
            print "===> ispossibleobject(): object DENIED (x,y,z coordinates of center of cluster not in acceptable range)"
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
        print "===> pickup_import.py: DavidLookForObject.getRollForPointCloud() started"
        #finds and return good roll angle (assuming its good to grasp where object is slim)
        # returned angle has to be added to cf_pregrasp and cf_finalgrasp
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


# new: 27.2.2015 start
class GoToFinalGraspPose(State):
    """
    This state should handle the following task.
    Calculate angle and distance the robot has to turn and move to get perfect grasping position
    (x,y,z,vectorX, vectorY, vectorZ)
    a Pose is calculated to which the robot will then navigate. This pose has
    to be stored inside the userdata output keys.
    If a detected object is already very near to the object, it could be that there is no mathematical solution
    In the case the system returns a failure which is then handled in the pickup logic.

    input_keys:
        obj_center_rcs: point that defines the x,y-values of the object center (of graspable object) in rcs

    output_keys:
        none
(
    input_keys:
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
 old)
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['obj_center_rcs'],
            output_keys=[]
        )
        self.move_robot_relative_pub = rospy.Publisher('/DiscreteMotionCmd', String)
        self.move_head_pub = rospy.Publisher('/head/move', String, queue_size=50)
        self.logger = logging.DoLogScenarioAndData()
        self.fms_x1 = 0.2  #free move space x_min (rcs)
        #self.fms_x2 =   variable due to distance the robot should move
        self.fms_y1 = -0.4  #free move space x_min (rcs)
        self.fms_y2 = 0.25  #free move space y_max (rcs)
        self.fms_z1 = 0.25  #free move space z_min (rcs)
        self.fms_z2 = 1.3  #free move space z_max (rcs)
        


    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        print "===> GoToFinalGraspPose.execute: execute started and received obj_center_x_rcs: ",ud.obj_center_rcs.x
        print "===> GoToFinalGraspPose.execute: execute started and received obj_center_y_rcs: ",ud.obj_center_rcs.y

        # robot detected object to grasp. Goal: robot turns "turn_r" (rad) and then 
        # moves mv_m (meter) to reach perfect grasping position
        # terminology: 
        #   d...distance
        #   a...angle in rad
        # We use for all points the rcs (x-axis forward, minus y-axis to the right side):
        #   pnt_O  .............. origin of robot => (0,0)
        #   pnt_GR .............. Grasp Region: relative to origin of robot (currently used: (0.05,-0.50)
        #   ud.obj_center_rcs ... input parameter: object center of graspable object (finally (after robot movement) this point should match the pnt_GR)
        #   pnt_RG .............. Robot Goal: this is the position (point) where the robot base has to be placed for the perfect grasp 
        #                         (we calculate it in a way such that no final rotation is needed 
        #   d_O_GR .............. fix distance between robot base point and best grasp region
        #   d_O_OC .............. fix distance between robot base (initially) and object center
        #   d_O_RG .............. distance (to calculate) between (original position of) robot base and desired end position for robot base
        #   a_MX_GR ............. fix angle between the minus x-axis of the robot and the GR (best Grasp Region relative to robot origin)
        #   a_OC_RG ............. the angle (measured at the robot origin) between the line O_OC and O_RG
        #   a_O_RG .............. the angle (measured at point ud.obj_center_rcs) between line to O (origin robot base) and point RG (robot goal) 
        #   a_OC_MY ............. the fix angle (measured at origin O) between the object center (graspable object) and the negative y-axis 
        #   
        #
        # we search now for the position where the robot base (pnt_O) has the distance of d_O_GR from ud.obj_center_rcs
        # and the angle for the movement to the final grasp position is s.t. the robot arrives already with the correct rotation angle
        # Therefore sine rule is applied to the triangle with points pnt_O, ud.obj_center_rcs and pnt_RG (has to be determined)
        
        try:        
            # pnt_O:       
            pnt_O = Point()
            pnt_O.x = 0
            pnt_O.y = 0
            
            # pnt_GR: define best grasp regien (right now 5cm infront and 50 cm right of robot base)       
            pnt_GR = Point()
            pnt_GR.x =  0.05
            if (pnt_GR.x < 0.0):
                print "pnt_GR.x has to be greater than 0. Act. value is: ", pnt_GR.x # rely on assumption for calculating  angle a_MX_GR
            pnt_GR.y = -0.50
            
            # pnt_RG: define best grasp regien (right now 5cm infront and 50 cm right of robot base)       
            #pnt_RG = Point()
            
            #distance between robot base and best grasp region        
            d_O_GR = math.sqrt( pnt_GR.x*pnt_GR.x + pnt_GR.y*pnt_GR.y)
            print "d_O_GR (m): ", d_O_GR
            
            # a_MX_GR: angle between robot minus x-axis of robot and best grasp region (calclated as 90 degrees (pi/2) plus degree of direction between -y-axis and d_O_GR
            a_MX_GR = math.pi/2 + math.atan( pnt_GR.x/ abs(pnt_GR.y) )
            print "a_MX_GR (rad):", a_MX_GR
            print "a_MX_GR (grad):", a_MX_GR*180/math.pi
            
            # d_O_OC fix distance between robot base (initially) and object center
            d_O_OC = math.sqrt( ud.obj_center_rcs.x*ud.obj_center_rcs.x + ud.obj_center_rcs.y*ud.obj_center_rcs.y )
            print "d_O_OC (distance between robot base (initially) and object center in m): ", d_O_OC
            
            #a_OC_RG: the angle (measured from the robot origin) between the line O_OC and O_RG (sine rule on triangle O - RG - OC and transformations)
            a_OC_RG = math.asin( d_O_GR * math.sin(a_MX_GR) / d_O_OC )
            print "a_OC_RG: the angle (measured from the robot origin) between the line O_OC and O_RG (in rad)", a_OC_RG
            print "a_OC_RG: the angle (measured from the robot origin) between the line O_OC and O_RG (in degree)", a_OC_RG*180/math.pi
            
            #a_O_RG: the angle (measured at point ud.obj_center_rcs) between line to O (origin robot base) and point RG (robot goal)
            a_O_RG = math.pi - a_MX_GR - a_OC_RG
            print "a_O_RG: the angle (measured at point ud.obj_center_rcs) between line to O (origin robot base) and point RG (robot goal) in rad: ", a_O_RG
            print "a_O_RG: the angle (measured at point ud.obj_center_rcs) between line to O (origin robot base) and point RG (robot goal) in grad: ", a_O_RG*180/math.pi
            
            #d_O_RG: distance (to calculate) between (original position of) robot base and desired end position for robot base
            d_O_RG = math.sin( a_O_RG ) * d_O_OC / math.sin( a_MX_GR )
            print "d_O_RG: distance (to calculate) between (original position of) robot base and desired end position for robot base in m: ", d_O_RG
            
            # a_OC_MY: the fix angle (measured at origin O) between the object center (graspable object) and the negativ y-axis (normal tan calculation in triangle O-(0,abs(y))-OC
            a_OC_MY = math.atan( ud.obj_center_rcs.x / abs(ud.obj_center_rcs.y) )
            print "a_OC_MY: the fix angle (measured at origin O) between the object center (graspable object) and the negativ y-axis in rad: ", a_OC_MY
            print "a_OC_MY: the fix angle (measured at origin O) between the object center (graspable object) and the negativ y-axis in grad: ", a_OC_MY*180/math.pi
            
            #calculate how much robot has to turn
            turn_rad = math.pi/2 - a_OC_MY - a_OC_RG
            #since robot has to turn right, it has to be multiplied by -1
            turn_rad = -turn_rad
            turn_degree = turn_rad*180/math.pi
            print "robot has to turn by (turn_rad): ", turn_rad        
            print "robot has to turn by (turn_degree): ", turn_degree
            print "robot has to move by d_O_RG (m)", d_O_RG
            
            if not self.moveRobotRelative(turn_degree, d_O_RG):
                print "61"
                self.logger.execute("4212FPF   Fine Positioning failed")    #log that fine positioning failed
                print "62"
                return 'aborted'
            
            print "63"
            self.logger.execute("4111FPS   Fine Positioning successful")    #log that fine positioning succeeded
            print "64"
            return 'succeeded'
    
        except:
            print "===> GoToFinalGraspPose: Error "
            print "65"
            self.logger.execute("4212FPF   Fine Positioning failed")    #log that fine positioning failed
            print "66"
            return 'aborted'

    def moveRobotRelative(self, turn_degree, distance_m):
        #turns the robot, then moves the robot to perfect grasp position
        min_turn_deg = 3    # minimal angles that rotation is executed (otherwise it is ignored)
        max_turn_deg_for_service = 7            # if not the discretemotion (with min 10 deg) is used
        min_turn_deg_for_discretemotion = 10    #fixed value, below nothing would be done by plattform (at least in the past)
        
        #turn head to navigation positon (without smach-stuff and without turning of emergency for head-motion (should not look up before anyway)
        self.move_head_pub.publish(String('down_center'))

        if (abs(turn_degree) < min_turn_deg):
            print "abs(turn_degree) < ", min_turn_deg, " => no turn executed"
        elif (abs(turn_degree) < max_turn_deg_for_service):

            #use service to turn robot for small values
            rospy.wait_for_service('apply_rotation')
            try:
                apply_rotation = rospy.ServiceProxy('apply_rotation', SendValue)
                input = SendValue()
                input.value = Float32(turn_degree)
                resp = apply_rotation(input.value)
                print "===> moveRobotRelative(): execute service apply rotation. Turn by degree: ", turn_degree
                print "==> moveRobotRelative() ==> turn successful: ", resp.state
                if resp.state == False:
                    print "======================================================================> TURNING ROBOT WITH SERVICE FAILED !!!!!!!!!!!!!!!!!!!!! => proceed"
                    #return -1    ignore problem and hope...
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                print "=====exception============================================================> TURNING ROBOT WITH SERVICE FAILED !!!!!!!!!!!!!!!!!!!!! => proceed"
                #return -1
            print "this is printed after the service for turning Hobbit is executed (without delay) "
        elif (abs(turn_degree) < min_turn_deg_for_discretemotion):
            turn_degree = min_turn_deg_for_discretemotion   #in case turndegree is between 7 and 10 degree => turn 10 degree
            if turn_degree < 0:
                turn_degree = -10
            if turn_degree > 0:
                turn_degree = 10
        
        print "0.1"
        #if (abs of) turn_degree greater then 10 degree, use topic for Hobbit rotation
        if (abs(turn_degree) >= min_turn_deg_for_discretemotion): 
            print "0.2"
            turn = String("Turn "+str(turn_degree))
            print "turn (String): ", turn
            self.move_robot_relative_pub.publish(turn)      # T U R N
            print "0.3"
        
        print "1"
        rospy.sleep(5)  #wait until head moved to down_center position AND robot has turned (if it turns)
        print "2"
        #check if there is enough space to move forward
        #!!!!!!!!!!!!!!!!!!!!!!missing: get point cloud!!
        #get point cloud
        ss_point_cloud = self.get_point_cloud_ss()  #DDDDDDDDDDDDDDD does not work
        print "3"
        if ss_point_cloud == None:
            print "4"
            return False
        
        print "5"
        nr_points_before_robot = self.check_free_space_for_moving_to_grasp_pos(ss_point_cloud, self.fms_x1, self.fms_x1+distance_m,self.fms_y1,self.fms_y2, self.fms_z1, self.fms_z2)
        print "6"
        if (nr_points_before_robot > 0):
            print "=> moveRobotRelative() ==> move forward could not be executed due to obstacles on the way"
            print "=> nr_points_before_robot: ", nr_points_before_robot
            return False
        print "7"
        #move Hobbit forward (discrete Motion)
        move = String("Move "+str(distance_m))
        print "8"
        print "move (String): ", move
        self.move_robot_relative_pub.publish(move)  # M O V E   F O R W A R D
        #move head back to grasp position 
        
        return True
# new 27.2.2015 = end ==

    #function to get single shot point cloud from head camera 
    def get_point_cloud_ss(self):
        print "11"
        rospy.wait_for_service('/table_object_detector/get_single_shot')
        print "12"
        try:
            print "13"
            get_single_shot = rospy.ServiceProxy('/table_object_detector/get_single_shot', SingleShotPC)
            print "14"
            req = SingleShotPC()
            print "15"
            req.ss = "getshot"
            print "16"
            res = get_single_shot(req.ss)
            print "17"
            return res.point_cloud
        except rospy.ServiceException, e:
            print "Service call in get_point_cloud_ss() failed: %s"%e
            return None


    def check_free_space_for_moving_to_grasp_pos(self, obstacle_cloud, x1=None, x2=None, y1=None, y2=None, z1=None, z2=None):
        print "test if there is enough space for moving to final grasp pos"
        resp1=None
        if obstacle_cloud == None:
            print "===> check_free_space_for_moving_to_grasp_pos(): no point cloud received!"
            return -1

        rospy.wait_for_service('check_free_space')
        try:
            check_free_space = rospy.ServiceProxy('check_free_space', CheckFreeSpace)
            input = CheckFreeSpace()
            input.cloud = obstacle_cloud
            input.frame_id_original = String(obstacle_cloud.header.frame_id)
            print "===> check_free_space_for_moving_to_grasp_pos(): input.frame_id_original: ", input.frame_id_original
            input.frame_id_desired = String("base_link")
            print "===> check_free_space_for_moving_to_grasp_pos(): input.frame_id_desired: ",input.frame_id_desired
            if (z2 is None):
                return -1
            else:
                input.x1=x1
                input.x2=x2
                input.y1=y1
                input.y2=y2
                input.z1=z1
                input.z2=z2
            #resp1 = check_free_space(input)
            resp1 = check_free_space(input.cloud,input.frame_id_original,input.frame_id_desired,input.x1,input.x2,input.y1,input.y2,input.z1,input.z2)
            print "===> check_free_space_for_moving_to_grasp_pos(): number of points in area with boarders \nx1: ", input.x1, "\tx2: ",input.x2,"\ny1: ",input.y1,"\ty2: ",input.y2,"\nz1: ",input.z1,"\tz2: ",input.z2,"\nnr_points: ",resp1.nr_points_in_area
            #return resp1.nr_points_in_area
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return -1
            
            
        return resp1.nr_points_in_area
   


#move robot back and turn to see an object that it couldn't see before
# new: 17.3.2015 start
class MoveRobotBackForBetterObjectView(State): # !!!done without seeing backwards!
    """
    This state should handle the following task.
    Robot could not grasp object (no object, no trajectory, ...) => hence the robot should move back turn and try again 
    (assume the object was to near to robot or not in grasping area)
    
    input_keys:
        none

    output_keys:
        none
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=[],
            output_keys=[]
        )
        self.move_robot_relative_pub = rospy.Publisher('/DiscreteMotionCmd', String)

    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        print "===> MoveRobotBackForBetterObjectView.execute: execute (move back and turn)"
        

        # robot moves back distance d and turns by turn_r (rad)
        #   dis_m...........distance (m)
        #   turn_degree .....degree
        dis_m = -0.5
        turn_degree = 30
        
        try:        
            if not self.moveRobotRelative(turn_degree, dis_m):
                return 'aborted'
    
        
            return 'succeeded'
    
        except:
            print "===> MoveRobotBackForBetterObjectView: Error "
            return 'aborted'

    def moveRobotRelative(self, turn_degree, distance_m):
        #moves the robot back and then moves the robot to perfect grasp position
        
        
        #move Hobbit backward (discrete Motion)
        move = String("Move "+str(distance_m))
        print "move (String): ", move
        self.move_robot_relative_pub.publish(move)
        rospy.sleep(7)
  
        
        #turn robot
        
        #if (abs of) turn_degree greater the 10 degree, use topic for Hobbit rotation
        turn = String("Turn "+str(turn_degree))
        print "turn (String): ", turn
        self.move_robot_relative_pub.publish(turn)
        rospy.sleep(5)
        
        return True
# new 17.3.2015 = end ==






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
        print "===> DavidLookingPose.execute: pointing message received: ",ud.pointing_msg


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
        print "===> DavidLookingPose.execute: p.point.x:", p.point.x
        pspWCS = self.listener.transformPoint('/map', p) #pspWCS: PointingStartPoint of the pointing gesture, i.e. the position of the shoulder in WCS (world coordinate system)
        print "===> DavidLookingPose.execute: shoulder coordinates in world coordinate system: ", pspWCS
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
		#print "======================================================="
		print "===> DavidLookingPose.execute: Pointing Vector in CCS (normalized): ", pvec # (CCS)
		#print "roation matrix: ", rot
		print "===> DavidLookingPose.execute: Pointing Vector in WCS: ", pvecWCS


                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "===> DavidLookingPose.execute: savePointingDirection(), calc pvec in RCS: tf transform /headcam_rgb_optical_frame to ", target_frame," not found"
                rospy.sleep(0.1)
                continue


	(robot_x, robot_y, robot_yaw) = util.get_current_robot_position(frame='/map')
        posRobot = [robot_x, robot_y] #Bajo, please fill in
        gpOnFloor = self.calcIntersectionPointingDirWithFloor(pspWCS, pvecWCS)
        if gpOnFloor == None:
            #FIXME: David look into this one
            rospy.loginfo('preempt is wrong. do failed')
	    return 'failed'



	#publish marker for visualizing the pointing point on floor
	print "===> DavidLookingPose.execute: now the point on floor is published as marker "
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


	print "===> grasp point on floor: X:  ", gpOnFloor[0], "   Y:  ", gpOnFloor[1], "   Z:  ", gpOnFloor[2]
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
            print "===> calcIntersectionPointingDirWithFloor(): ERROR, pointing direction must be downwards!!"
            print "===> calcIntersectionPointingDirWithFloor(): k = ", k
            return None
        gpOnFloor = [pspWCS.point.x+k*pvecWCS[0],pspWCS.point.y+k*pvecWCS[1],0]
        return gpOnFloor





class CalcGrasppointsActionClient():

    last_feedback = None

    def __init__(self):
        self.calc_grasppoints_client = actionlib.SimpleActionClient("calc_grasppoints_svm_action_server", hobbit_msgs.msg.CalcGraspPointsServerAction)


    # use this method with a String cmd to send goal to the ArmActionServer and return its result value
    def calc_grasppoints_action_client(self, cmd):

        print "===> CalcGrasppointsActionClient() started"
        #print "type of cmd: ", type(cmd)
        # Waits until the action server has started up and started
        # listening for goals.
        print "===> CalcGrasppointsActionClient(): wait_for_server()"
        self.calc_grasppoints_client.wait_for_server()
        print "===> CalcGrasppointsActionClient(): server found!"


        print "===> CalcGrasppointsActionClient(): create goal"
        # Creates a goal to send to the action server.
        goal = hobbit_msgs.msg.CalcGraspPointsServerGoal(input_pc=cmd)

        #print "CalcGrasppointsActionClient: send goal"
        # Sends the goal to the action server.
        self.calc_grasppoints_client.send_goal(goal, feedback_cb=self.feedback_cb)

        print "CalcGrasppointsActionClient(): wait for result"
        # Waits for the server to finish performing the action.
        self.calc_grasppoints_client.wait_for_result()


        # Prints out the result of executing the action
        returnval = self.calc_grasppoints_client.get_result()  #
        print "===> CalcGrasppointsActionClient(): returnval from calc_grasppoints: ", returnval
        return returnval

    def feedback_cb(self, feedback):
        print "===> CalcGrasppointsActionClient(): feedback_cb executed!"
        #print "feedback type: ", (type) (feedback)
        self.last_feedback = feedback
        print "===> CalcGrasppointsActionClient(): feedback: ", self.last_feedback





class DavidPickingUp(State):
    """
    This state moves the arm from the PreGrasping position to the object,
    closes the gripper and returns 'succeeded' afterwards.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted',
                      'failed_no_trajectory_for_grasping_found',
                      'failed_no_suitable_object_found',
                      'failed_no_sufficient_grasp_detected',
                      'failed_no_space_to_move_arm'],
            input_keys=['cloud']
        )
        self.arm_client = ArmActionClient()
        self.listener = tf.TransformListener()
        self.pubClust = rospy.Publisher("/pickup/objectclusters", PointCloud2)
        self.pubGraspableObjectCCS = rospy.Publisher("/pickup/graspableobjectCCS", PointCloud2)
        self.rec = TD()
        self.restrictfind = True
        self.calc_graspoints_client = CalcGrasppointsActionClient()
        self.logger = logging.DoLogScenarioAndData()
        #define limits for checking free space for grasping (ccs)
        self.limit_x1 = -0.5
        self.limit_x2 = 0.4
        self.limit_y1 = -0.7
        self.limit_y2 = -0.35
        self.limit_z1 = 0.25
        self.limit_z2 = 1.2


    def execute(self, ud):
        print "===> pickup_import.py: DavidPickingUp.execute()"
        mmui = MMUI.MMUIInterface()
        if self.preempt_requested():
            return 'preempted'
        # TODO: David please add the grasping in here

        if self.findobject(ud):

            print "===> DavidPickingUp.execute: findobject was successful"
            #self.pc_rcs is available () => calc_grasp_points.cpp (AS) and get back grasp pose definition
            #overview:
            # 1) check if there is enough space for moving the arm
            # 2) call calc_grasp_points AS
            # 3) call openrave trajectory AS
            # 4) call Arm AS (arm client already included in code)
            #grasp pose definition => simulation => execution
            # 5) check if still objects are on the floor (<=> check if grasping was successfull, asuming there was only one object)


            #1) check if there is enough space for moving the arm
            pnt_in_space = self.check_free_space_for_arm_pickup_movement(ud.cloud)
            print "=======================================================================================================>"
            print "===> DavidPickingUp.execute: check_free_space_for_arm_pickup_movement: pnt_in_space: ", pnt_in_space
            if (pnt_in_space > 0):
                self.logger.execute("6219NFSG No Free Space to grasp")
                self.logger.execute("5214RORGP Reason why Object(s) not accepted (grasppos): " + "No Free Space to grasp")
                print "===> DavidPickingUp.execute: arm not able to move savely for picking up => PICKING UP WAS STOPPED"
                return 'failed_no_space_to_move_arm'
            else:
                self.logger.execute("6118FSG   Free Space to grasp")

            # 2) call calc_grasppoints_action_server/client (calc_aS(self.pc_rcs)) and receive a grasp_representation in the format (string):
            # "(0)eval_val (1)gp1_x (2)gp1_y (3)gp1_z (4)gp2_x (5)gp2_y (6)gp2_z (7)ap_vec_x (8)ap_vec_y (9)ap_vec_z (10)gp_center_x (11)gp_center_y (12)gp_center_z (13)roll"
            gp_representation = self.calc_graspoints_client.calc_grasppoints_action_client(self.pc_ccs)#ud.cloud)#self.pc_rcs) #ud.cloud) 12.12.2014
            print "===> DavidPickingUp.execute: gp_representation: ", gp_representation
            gp_pres_str = str(gp_representation.result.data)
            gp_eval = int(gp_pres_str[0:2])
            if (gp_eval < 2):
                print "===> DavidPickingUp.execute: GRASP EVALUATION WAS TO BAD FOR RELIABLE GRASPING - GRASPING STOPPED"
                self.logger.execute("5214RORGP Reason why Object(s) not accepted (grasppos): " + "Grasp point evaluation was too bad")
                return 'failed_no_sufficient_grasp_detected'
            # 3) (and 4)) call GraspFromFloorTrajectoryActionServer/Client to receive a trajectory (that will be directly executed) by calling the ArmActionServer/client
            grasp_traj_ac = arm_simulation.GraspTrajectoryActionClient.GraspTrajectoryActionClient()
            print "===> DavidPickingUp.execute: grasp trajectory action client generated: ",grasp_traj_ac
            #calculate grasp grajectory (way points)
            cmd = gp_representation.result #String ("81 0.04 -0.45 0.127266 0.04 -0.51 0.127266 0 0 1 0.04 -0.48 0.127266 0") #input (=> = output from calc_grasppoints_svm_action_server)
            #df new 6.2.2015: 
            resp = mmui.showMMUI_Info(text='T_PU_PickingUpObject')
            res = grasp_traj_ac.grasp_trajectory_action_client(cmd)
            print "pickup_import.py: DavidPickingUp -> execute: res of grasp_traj_ac.grasp_trajectory_action_client(cmd): ", res
            print "pickup_import.py: DavidPickingUp -> execute: res.result.data of grasp_traj_ac.grasp_trajectory_action_client(cmd): ", res.result.data
            
            if res.result.data:
                #trajectory was found and object was grasped
                self.logger.execute("5416GOFGP Graspable Object found (grasppos)")
                #pass    
            else:
                self.logger.execute("5214RORGP Reason why Object(s) not accepted (grasppos): " + "No Arm Trajectory for Grasping found")
                return 'failed_no_trajectory_for_grasping_found'
                        
            #move object to tray and move arm back to home position
            # => no done via logic in pickup.py  res = self.arm_client.arm_action_client(String ("SetMoveToTrayPos"))
        else:
            return 'failed_no_suitable_object_found'   

        return 'succeeded'



    def check_free_space_for_arm_pickup_movement(self, obstacle_cloud, x1=None, x2=None, y1=None, y2=None, z1=None, z2=None):
        print "test if there is enough space for moving the arm for picking up an object"
        #self.t = rospy.Time.now()
        if obstacle_cloud == None:
            print "===> check_free_space_for_arm_pickup_movement(): check_free_space_for_arm_pickup_movement: no point cloud received!"
            return -1
        #self.pc_.header.stamp = self.t
        rospy.wait_for_service('check_free_space')
        try:
            check_free_space = rospy.ServiceProxy('check_free_space', CheckFreeSpace)
            input = CheckFreeSpace()
            input.cloud = obstacle_cloud
            input.frame_id_original = String(obstacle_cloud.header.frame_id)
            print "===> check_free_space_for_arm_pickup_movement(): input.frame_id_original: ", input.frame_id_original
            input.frame_id_desired = String("base_link")
            print "===> check_free_space_for_arm_pickup_movement(): input.frame_id_desired: ",input.frame_id_desired
            if (z2 is None):
                input.x1 = self.limit_x1
                input.x2 = self.limit_x2
                input.y1 = self.limit_y1
                input.y2 = self.limit_y2
                input.z1 = self.limit_z1
                input.z2 = self.limit_z2
            else:
                input.x1=x1
                input.x2=x2
                input.y1=y1
                input.y2=y2
                input.z1=z1
                input.z2=z2
            #resp1 = check_free_space(input)
            resp1 = check_free_space(input.cloud,input.frame_id_original,input.frame_id_desired,input.x1,input.x2,input.y1,input.y2,input.z1,input.z2)
            print "===> check_free_space_for_arm_pickup_movement(): number of points in area with boarders \nx1: ", input.x1, "\tx2: ",input.x2,"\ny1: ",input.y1,"\ty2: ",input.y2,"\nz1: ",input.z1,"\tz2: ",input.z2,"\nnr_points: ",resp1.nr_points_in_area
            #return resp1.nr_points_in_area
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return -1
            
        return resp1.nr_points_in_area

    
    def findobject(self, ud):
        print "===> pickup_import.py: DavidPickingUp.findobject() started"
        pc_ccs = ud.cloud   #point cloud in camera coordinate system
        #raw_input(" ==============> publish cluster that findObjectsOnFloor is receiving from pickup_import.py")
        self.pubClust.publish(ud.cloud)
        #clusters = self.rec.findObjectsOnFloor(pc_ccs, [0,0,0,0]) #before: pointcloud instead of pc_ccs
        #df 19.3.2015 => changed for smaller objects dfdfdfdf
        clusters = self.rec.findObjectsOnFloorSmall(pc_ccs, [0,0,0,0]) #before: pointcloud instead of pc_ccs
        print "===> pickup_import.py: DavidPickingUp.findobject(): number of object clusters on floor found: ", len(clusters)
        self.logger.execute("5113NOSGP Nr of Objects segmented (grasppos): "+str(len(clusters)))  #log nr of object segmented at pregrasp position
        
        nr_objects_segmented = len(clusters)
        i = 0
        for cluster in clusters:
            i = i+1
            self.pc = cluster
            print "===> pickup_import.py: DavidPickingUp.findobject(): publish cluster"
            self.pubClust.publish(cluster)
            if self.isGraspableObject():
                self.logger.execute("5719OPPGP Object passed position check. Segmented Object Nr: " + str(i) + "/" + str(nr_objects_segmented))  #log nr of object that passed position check
                self.pubGraspableObjectCCS.publish(cluster)
                return True
            else:
                self.logger.execute("5618OFPGP Object failed position check. Segmented Object Nr: " + str(i) + "/" + str(nr_objects_segmented))  #log nr of object that passed position check

        print "===> pickup_import.py: DavidPickingUp.findobject(): NO GRASPABLE OBJECT FOUND"
        self.logger.execute("5214RORGP Reason why Object(s) not accepted (grasppos): " + "All " + str(nr_objects_segmented) + " segmented objects did not pass position check")
        return False


    #checks if object is suitable for grasping
    def isGraspableObject(self):
        print "\n ===> pickup_import.py: DavidPickingUp.isGraspableObject() started"

        print "===> pickup_import.py: DavidPickingUp.isGraspableObject(): trying to get tf transform"
        while True:
            try:
                (trans,rot) = self.listener.lookupTransform('/headcam_rgb_optical_frame', '/base_link', rospy.Time(0))
                #print trans,rot
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "- isGraspableObject(): tf transform /headcam_rgb_optical_frame to /base_link not found"
                rospy.sleep(1)
                continue

        m = self.getCenterOfCluster()
        print "===> pickup_import.py: DavidPickingUp.isGraspableObject(): center of cluster", m
        p = PointStamped()
        p.header.frame_id = '/headcam_rgb_optical_frame'
        p.point.x = m[0]
        p.point.y = m[1]
        p.point.z = m[2]

        pnt_rcs = self.listener.transformPoint('/base_link', p)     #center of cluster in robot coordinate system
        pnt_wcs = self.listener.transformPoint('/map', p)           #center of cluster in world coordinate system

        #print "============== center of cluster in camera coordinate system: ", p
        print "===> pickup_import.py: DavidPickingUp.isGraspableObject(): center of cluster in RCS:                    : ", pnt_rcs

        #log object data: center pnt in wcs, center_pnt in rcs, segmented point cloud size
        self.logger.execute("5315ODSGP Object Data per Segmented Object (grasppos): " + "Object center in WCS: " + pnt_wcs + " Object center in RCS: "+pnt_rcs + "Number points of segmented objects: " + self.pc.height*self.pc.width)


        isgraspable = self.ispossibleobject(pnt_rcs)
        if isgraspable:
            self.pc_rcs = self.transformPointCloud('/base_link',self.pc)
            self.pc_ccs = self.pc
        return isgraspable


    def getCenterOfCluster(self):
        print "===> pickup_import.py: DavidPickingUp.getCenterOfCluster() started"
        #extract points from PointCloud2 in python....
        fmt = self._get_struct_fmt(self.pc)
        narr = list()
        offset = 0
        for i in xrange(self.pc.width * self.pc.height):
            p = struct.unpack_from(fmt, self.pc.data, offset)
            offset += self.pc.point_step
            narr.append(p[0:3])
            

        df = narr.__len__(); #length of point cloud
        print "===> pickup_import.py: DavidPickingUp.getCenterOfCluster(): LENGTH OF POINTCLOUD:", df
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
        print "===> pickup_import.py: DavidLookForObject.transformPointCloud() started"
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
                #print trans,rot
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "===> pickup_import.py: DavidLookForObject.transformPointCloud(): tf transform /frame to ", target_frame," not found"
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
        print "===> pickup_import.py: DavidPickingUp.ispossibleobject() started"
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
            y_min = -0.57
            y_max = -0.23
            z_min = 0.01
            z_max = 0.18    #only that high because of buggy camera

        #print "pnt.point.x: ", pnt.point.x
        if (pnt.point.x > x_min and pnt.point.x < x_max and pnt.point.y > y_min and pnt.point.y < y_max and pnt.point.z > z_min and pnt.point.z < z_max):
            print "===> pickup_import.py: DavidPickingUp.ispossibleobject(): OBJECT ACCEPTED"
            return True
        else:
            print "===> pickup_import.py: DavidPickingUp.ispossibleobject(): object DENIED (x,y,z coordinates of center of cluster not in acceptable range)"
            return False




def point_cloud_cb(ud, msg):
    print('===> point cloud received')
    ud.cloud = msg
    return False


class DavidCheckGrasp(State):
    """
    This state has to check if the grasping was successful or not.
    Return 'succeeded' if it was, 'aborted' if it was not.
    Returns 'preempted' on request.
    to many hardcoded values
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['cloud']
        )
        self.pubClust = rospy.Publisher("/pickup/checkgraspclusters", PointCloud2)
        self.pubClustDavid = rospy.Publisher("/davidtest", PointCloud2)
        self.limit_x1 = 0.02
        self.limit_x2 = 0.15
        self.limit_y1 = -0.55
        self.limit_y2 = -0.44
        self.limit_z1 = 0.03
        self.limit_z2 = 0.15
        self.logger = logging.DoLogScenarioAndData()
        
    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        # TODO: David please add the check of the grasping in here => Esters code (call service)
        print "\n\n\n ===========================================================\n\”\n"
        self.pubClustDavid.publish(ud.cloud)
        pnt_in_space = self.check_free_space_for_arm_pickup_movement(ud.cloud)
        print "===> DavidCheckGrasp.execute: number of points found in grasping area: ", pnt_in_space
        if (pnt_in_space > 20):
            self.logger.execute("7221GFA   Grasp Failed")    #log grasp has failed (according to check grasp)
            return 'aborted'    #aborted <=> point found, hence an object still on floor => (assumed that) object grasping failed
        else:
            self.logger.execute("7120GSF   Grasp successful")    #log grasp has failed (according to check grasp)
            return 'succeeded'  #succedded <=> no object on floor (in grasping area) => (assumed that) object grasping succeeded
            

    ''' not used due to bad performance (not reliable enough)
    def execute_ester(self, ud):
        if self.preempt_requested():
            return 'preempted'
        # TODO: David please add the check of the grasping in here => Esters code (call service)

        rospy.wait_for_service('grasp_success_check')
        try:
            grasp_success_check_fct = rospy.ServiceProxy('grasp_success_check', GraspSuccessCheck)
            resp1 = grasp_success_check_fct(ud.cloud)
            print "result of grasp success evaluation: ", resp1.result
            if ( resp1.result == True ):
                return 'succeeded'
            else:
                return 'aborted'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            #use ud.cloud and esthers code to receive succeeded/aborted
    
            return 'succeeded'
        return 'succeeded'
    '''

    # next function is defened for 2 clases => REDUNDANTLY!!
    def check_free_space_for_arm_pickup_movement(self, obstacle_cloud, x1=None, x2=None, y1=None, y2=None, z1=None, z2=None):
        print "test if there is still an obejct on the floor (in the grasping area)"
        #self.t = rospy.Time.now()
        if obstacle_cloud == None:
            print "===> DavidCheckGrasp: check_free_space_for_arm_pickup_movement: no point cloud received! "
            return -1
        #self.pc_.header.stamp = self.t
        rospy.wait_for_service('check_free_space')
        try:
            check_free_space = rospy.ServiceProxy('check_free_space', CheckFreeSpace)
            input = CheckFreeSpace()
            input.cloud = obstacle_cloud
            input.frame_id_original = String(obstacle_cloud.header.frame_id)
            print "===> DavidCheckGrasp: check_free_space_for_arm_pickup_movement: input.frame_id_original: ", input.frame_id_original
            input.frame_id_desired = String("base_link")
            print "===> DavidCheckGrasp: check_free_space_for_arm_pickup_movement: input.frame_id_desired: ",input.frame_id_desired
            if (z2 is None):
                input.x1 = self.limit_x1
                input.x2 = self.limit_x2
                input.y1 = self.limit_y1
                input.y2 = self.limit_y2
                input.z1 = self.limit_z1
                input.z2 = self.limit_z2
            else:
                input.x1=x1
                input.x2=x2
                input.y1=y1
                input.y2=y2
                input.z1=z1
                input.z2=z2
            #resp1 = check_free_space(input)
            resp1 = check_free_space(input.cloud,input.frame_id_original,input.frame_id_desired,input.x1,input.x2,input.y1,input.y2,input.z1,input.z2)
            print "===> DavidCheckGrasp: check_free_space_for_arm_pickup_movement: number of points in area with boarders \nx1: ", input.x1, "\tx2: ",input.x2,"\ny1: ",input.y1,"\ty2: ",input.y2,"\nz1: ",input.z1,"\tz2: ",input.z2,"\nnr_points: ",resp1.nr_points_in_area
            #return resp1.nr_points_in_area
        except rospy.ServiceException, e:
            print "===> DavidCheckGrasp: check_free_space_for_arm_pickup_movement: Service call failed: %s"%e
            return -1
            
        return resp1.nr_points_in_area


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


def sayPointingDeclinedByUser():
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_SAD': 'succeeded',
                                   'SAY_POINTING_DECLINED_BY_USER': 'succeeded'}}
    )

    with cc:
        Concurrence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Concurrence.add(
            'SAY_POINTING_DECLINED_BY_USER',
            speech_output.sayText(info='T_PU_PointingDeclinedByUser')
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


def sayObjectNotDetected():
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
            speech_output.sayText(info='T_PU_ObjectNotDetectedEnd')
        )
    return cc


#def sayObjectNotDetected2():
#    cc = Concurrence(
#        outcomes=['succeeded', 'preempted', 'failed'],
#        default_outcome='failed',
#        child_termination_cb=child_term_cb,
#        outcome_map={'succeeded': {'EMO_SAD': 'succeeded',
#                                   'SAY_OBJECT_NOT_DETECTED': 'succeeded'}}
#    )
#
#    with cc:
#        Concurrence.add(
#            'EMO_SAD',
#            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
#        )
#        Concurrence.add(
#            'SAY_OBJECT_NOT_DETECTED',
#            speech_output.sayText(info='T_PU_ObjectNotDetected2')
#        )
#    return cc


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


#movement of robot did not work as it should. user is asked if robot should try to find better position
def sayUnableToMove():
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


def sayMoveIssuePUstopped():
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_SAD': 'succeeded',
                                   'SAY_MOVE_ISSUE_PU_STOPPED': 'succeeded'}}
    )
    
    with cc:
        Concurrence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Concurrence.add(
            'SAY_MOVE_ISSUE_PU_STOPPED',
            speech_output.sayText(info='T_PU_MoveIssuePUStopped')
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


def sayDidNotPickupObjectTwoTimes():
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
            speech_output.sayText(info='T_PU_DidNotPickupObjectTwoTimes')
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
            DavidLookingPose(),
            transitions={'preempted': 'preempted'}
        )
        Sequence.add(
            'MOVE_TO_POSE',
            hobbit_move.goToPoseSilent(),
            transitions={'aborted': 'failed',
                         'preempted': 'preempted'},
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
        outcomes=['succeeded', 'preempted', 'failed', 'failed_arm_not_moved']
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
                         'failed': 'failed',
                         'failed_no_trajectory_for_grasping_found': 'TRAJECTORY',   #df new 11.2.2015
                         'failed_no_suitable_object_found': 'OBJECT',
                         'failed_no_sufficient_grasp_detected': 'GRASP',
                         'failed_no_space_to_move_arm': 'ARM'}
        )
        StateMachine.add(
            'TRAJECTORY',
            speech_output.emo_say_something(text='T_PICKUP_BETTER_POSITION'),
            transitions={'succeeded': 'failed_arm_not_moved',
                         'preempted': 'preempted',
                         'aborted': 'failed',}
        )
        StateMachine.add(
            'ARM',
            speech_output.emo_say_something(text='T_ARM_NO_SPACE'),
            transitions={'succeeded': 'failed',
                         'preempted': 'preempted',
                         'aborted': 'failed',}
        )
        StateMachine.add(
            'OBJECT',
            speech_output.emo_say_something(text='T_PICKUP_NO_OBJECT'),
            transitions={'succeeded': 'failed',
                         'preempted': 'preempted',
                         'aborted': 'failed',}
        )
        StateMachine.add(
            'GRASP',
            speech_output.emo_say_something(text='T_PICKUP_BETTER_POSITION'),
            transitions={'succeeded': 'failed',
                         'preempted': 'preempted',
                         'aborted': 'failed',}
        )
        return sm
        #return seq

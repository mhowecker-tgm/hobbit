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
import hobbit_smach.arm_move_import as arm_move
import hobbit_smach.speech_output_import as speech_output
from testdetector import TD
import tf
from sensor_msgs.msg import PointField
from std_msgs.msg import String
from hobbit_smach.ArmActionClient import ArmActionClient
import actionlib
import hobbit_msgs.msg
from hobbit_msgs.srv import *
import arm_simulation.GraspTrajectoryActionClient
from table_object_detector.srv import *
from hobbit_msgs import MMUIInterface as MMUI
import hobbit_smach.logging_import as logging

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

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
                logging.do_log_data("6219NFSG No Free Space to grasp")
                logging.do_log_data("5214RORGP Reason why Object(s) not accepted (grasppos): " + "No Free Space to grasp")
                print "===> DavidPickingUp.execute: arm not able to move savely for picking up => PICKING UP WAS STOPPED"
                #return 'failed_no_space_to_move_arm'
            else:
                logging.do_log_data("6118FSG   Free Space to grasp")

            # 2) call calc_grasppoints_action_server/client (calc_aS(self.pc_rcs)) and receive a grasp_representation in the format (string):
            # "(0)eval_val (1)gp1_x (2)gp1_y (3)gp1_z (4)gp2_x (5)gp2_y (6)gp2_z (7)ap_vec_x (8)ap_vec_y (9)ap_vec_z (10)gp_center_x (11)gp_center_y (12)gp_center_z (13)roll"
            gp_representation = self.calc_graspoints_client.calc_grasppoints_action_client(self.pc_ccs)#ud.cloud)#self.pc_rcs) #ud.cloud) 12.12.2014
            print "===> DavidPickingUp.execute: gp_representation: ", gp_representation
            gp_pres_str = str(gp_representation.result.data)
            gp_eval = int(gp_pres_str[0:2])
            if (gp_eval < 2):
                print "===> DavidPickingUp.execute: GRASP EVALUATION WAS TO BAD FOR RELIABLE GRASPING - GRASPING STOPPED"
                logging.do_log_data("5214RORGP Reason why Object(s) not accepted (grasppos): " + "Grasp point evaluation was too bad")
                return 'failed_no_sufficient_grasp_detected'
            # 3) (and 4)) call GraspFromFloorTrajectoryActionServer/Client to receive a trajectory (that will be directly executed) by calling the ArmActionServer/client
            grasp_traj_ac = arm_simulation.GraspTrajectoryActionClient.GraspTrajectoryActionClient()
            print "===> DavidPickingUp.execute: grasp trajectory action client generated: ",grasp_traj_ac
            #calculate grasp grajectory (way points)
            cmd = gp_representation.result #String ("81 0.04 -0.45 0.127266 0.04 -0.51 0.127266 0 0 1 0.04 -0.48 0.127266 0") #input (=> = output from calc_grasppoints_svm_action_server)

            res = grasp_traj_ac.grasp_trajectory_action_client(cmd)
            print "pickup_import.py: DavidPickingUp -> execute: res of grasp_traj_ac.grasp_trajectory_action_client(cmd): ", res
            print "pickup_import.py: DavidPickingUp -> execute: res.result.data of grasp_traj_ac.grasp_trajectory_action_client(cmd): ", res.result.data
            
            if res.result.data:
                #trajectory was found and object was grasped
                logging.do_log_data("5416GOFGP Graspable Object found (grasppos)")
                #pass    
            else:
                logging.do_log_data("5214RORGP Reason why Object(s) not accepted (grasppos): " + "No Arm Trajectory for Grasping found")
                return 'failed_no_trajectory_for_grasping_found'
                        
            #move object to tray and move arm back to home position
            # => no done via logic in pickup.py  res = self.arm_client.arm_action_client(String ("SetMoveToTrayPos"))
        else:
            return 'failed_no_suitable_object_found'   

        return 'succeeded'



    def check_free_space_for_arm_pickup_movement(self, obstacle_cloud, x1=None, x2=None, y1=None, y2=None, z1=None, z2=None):
        print "test if there is enough space for moving the arm for picking up an object"
        if obstacle_cloud == None:
            print "===> check_free_space_for_arm_pickup_movement(): check_free_space_for_arm_pickup_movement: no point cloud received!"
            return -1
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
        logging.do_log_data("5113NOSGP Nr of Objects segmented (grasppos): "+str(len(clusters)))  #log nr of object segmented at pregrasp position
        
        nr_objects_segmented = len(clusters)
        i = 0
        for cluster in clusters:
            i = i+1
            self.pc = cluster
            print "===> pickup_import.py: DavidPickingUp.findobject(): publish cluster"
            self.pubClust.publish(cluster)
            if self.isGraspableObject():
                logging.do_log_data("5719OPPGP Object passed position check. Segmented Object Nr: " + str(i) + "/" + str(nr_objects_segmented))  #log nr of object that passed position check
                self.pubGraspableObjectCCS.publish(cluster)
                return True
            else:
                logging.do_log_data("5618OFPGP Object failed position check. Segmented Object Nr: " + str(i) + "/" + str(nr_objects_segmented))  #log nr of object that passed position check

        print "===> pickup_import.py: DavidPickingUp.findobject(): NO GRASPABLE OBJECT FOUND"
        logging.do_log_data("5214RORGP Reason why Object(s) not accepted (grasppos): " + "All " + str(nr_objects_segmented) + " segmented objects did not pass position check")
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

        #print "============== center of cluster in camera coordinate system: ", p
        print "===> pickup_import.py: DavidPickingUp.isGraspableObject(): center of cluster in RCS:                    : ", pnt_rcs

        #log object data: center pnt in wcs, center_pnt in rcs, segmented point cloud size
        logging.do_log_data("5315ODSGP Object Data per Segmented Object (grasppos): " + "Object center in WCS: " + str(pnt_wcs) + " Object center in RCS: "+ str(pnt_rcs) + "Number points of segmented objects: " + str(self.pc.height*self.pc.width))


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
            

        df = narr.__len__(); #len   gth of point cloud
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
        
    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        # TODO: David please add the check of the grasping in here => Esters code (call service)
        print "\n\n\n ===========================================================\n\”\n"
        self.pubClustDavid.publish(ud.cloud)
        pnt_in_space = self.check_free_space_for_arm_pickup_movement(ud.cloud)
        print "===> DavidCheckGrasp.execute: number of points found in grasping area: ", pnt_in_space
        if (pnt_in_space > 20):
            logging.do_log_data("7221GFA   Grasp Failed")    #log grasp has failed (according to check grasp)
            return 'aborted'    #aborted <=> point found, hence an object still on floor => (assumed that) object grasping failed
        else:
            logging.do_log_data("7120GSF   Grasp successful")    #log grasp has failed (according to check grasp)
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
        Sequence.add(
            'EMO_SAY_REMOVE_OBJECT',
            sayRemoveObjectTakeObject()
        )
        return seq

def getPickupSeq():
    """
    Return a SMACH Statemachine
    """

    sm = StateMachine(
        outcomes=['succeeded', 'preempted', 'failed', 'failed_arm_not_moved']
    )

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
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'ARM',
            speech_output.emo_say_something(text='T_ARM_NO_SPACE'),
            transitions={'succeeded': 'failed',
                         'preempted': 'preempted',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'OBJECT',
            speech_output.emo_say_something(text='T_PICKUP_NO_OBJECT'),
            transitions={'succeeded': 'failed',
                         'preempted': 'preempted',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'GRASP',
            speech_output.emo_say_something(text='T_PICKUP_BETTER_POSITION'),
            transitions={'succeeded': 'failed',
                         'preempted': 'preempted',
                         'aborted': 'failed'}
        )
        return sm

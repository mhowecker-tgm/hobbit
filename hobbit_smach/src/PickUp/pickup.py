#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
PROJECT = 'Hobbit'
NAME = 'BringObject'

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import uashh_smach.util as util
import tf
import math

from smach import StateMachine, State

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction
from smach_ros import ActionServerWrapper, SimpleActionState, ServiceState
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose2D, PoseStamped, Point, Quaternion, Vector3Stamped, PoseWithCovarianceStamped, Pose
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.head_move_import as head_move
import hobbit_smach.arm_move_import as arm_move
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.pickup_import as pickup

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
def disable(self):
        self.HEADER = ''
        self.OKBLUE = ''
        self.OKGREEN = ''
        self.WARNING = ''
        self.FAIL = ''
        self.ENDC = ''

class Init(smach.State):
    """Class to initialize certain parameters"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'canceled'], input_keys=['object_name'], output_keys=['social_role'])
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)

    def execute(self,ud):
        self.pub_obstacle.publish('active')
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        return 'succeeded'

class CleanUp(smach.State):
    """Class for setting the result message and clean up persistent variables"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                input_keys=['command', 'visited_places'],
                output_keys=['result','command', 'visited_places'])

    def execute(self, ud):
        ud.visited_places = []
        ud.result = String('object not found')
        return 'succeeded'


class SetSuccess(smach.State):
    """Class for setting the success message in the actionlib result and clean up of persistent variables"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['result'],
                             output_keys=['result', 'visited_places'])
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)

    def execute(self, ud):
        ud.visited_places = []
        self.pub.publish('Stop')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('object detected')
        rospy.loginfo(ud.result)
        return 'succeeded'


class DummyGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])

    def execute(self, ud):
        rospy.loginfo('Start DUMMY grasp')
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'succeeded'
        #return 'failed'


def get_robot_pose_cb(msg, ud):
    #print bcolors.WARNING + 'received message: '+ bcolors.ENDC
    #print msg
    try:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = msg.pose.pose.position
        pose.pose.orientation = msg.pose.pose.orientation
        ud.robot_current_pose = pose
        return True
    except:
        print bcolors.FAIL + 'no robot pose message received.'+ bcolors.ENDC
        return False


def point_cloud_cb(msg, ud):
    print('point cloud received')
    ud.cloud= msg
    return True


def main():
    rospy.init_node(NAME)

    pickup_sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['object_name'],
        output_keys=['result'])

    pickup_sm.userdata.result = String('started')
    pickup_sm.userdata.detection = False

    with pickup_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'GET_OBJECTS_POSITIONS',
                         'canceled': 'CLEAN_UP'}
        )
        StateMachine.add(
            'GET_POINTING_DIRECTION',
            GetDataFromCommand(),
            transitions={'succeeded': 'GET_VECTOR',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'GET_VECTOR',
            GetVectorFromCommand(),
            transitions={}
        )
        StateMachine.add(
            'POINTING_NOT_DETECTED_1',
            pickup.sayPointingGestureNotDetected(),
            transitions={}
        )
        StateMachine.add(
            'POINTING_NOT_DETECTED_2',
            pickup.sayPointingGestureNotDetected2(),
            transitions={}
        )
        StateMachine.add(
            'START_LOOKING',
            pickup.getStartLooking(),
            transitions={}
        )
        StateMachine.add(
            'LOOK_FOR_OBJECT',
            DavidDummyLook(),
            transitions={'succeeded': 'CALC_GRASP_POSE',
                         'aborted': 'END_FAIL_OBJECT_DETECTION',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'CALC_GRASP_POSE',
            DavidDummyLook(),
            transitions={'succeeded': '',
                         'aborted': 'EMO_SAY_OBJECT_NOT_DETECTED',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'EMO_SAY_OBJECT_NOT_DETECTED',
            pickup.sayObjectNotDetected1
        )




        smach.StateMachine.add('GET_OBJECTS_POSITIONS', ServiceState('/Hobbit/ObjectService/get_object_locations', GetObjectLocations, request_key='object_name', response_key='response'), transitions={'succeeded':'CLEAN_POSITIONS', 'preempted':'preempted'})
        smach.StateMachine.add('CLEAN_POSITIONS', CleanPositions(), transitions={'succeeded':'GET_ROBOT_POSE', 'failure':'CLEAN_UP', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('GET_ROBOT_POSE', util.WaitForMsgState('/amcl_pose', PoseWithCovarianceStamped, get_robot_pose_cb, output_keys=['robot_current_pose'], timeout=5),
                transitions={'succeeded':'GET_ROBOTS_CURRENT_ROOM', 'aborted':'GET_ROBOT_POSE', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('GET_ROBOTS_CURRENT_ROOM', ServiceState('get_robots_current_room', GetName, response_key='robots_room_name'), transitions={'succeeded':'PLAN_PATH'})
        smach.StateMachine.add('PLAN_PATH', PlanPath(), transitions={'success':'MOVE_HEAD_DOWN', 'preempted':'CLEAN_UP', 'failure':'CLEAN_UP'})
        smach.StateMachine.add('MOVE_HEAD_DOWN', head_move.MoveTo(pose='down_center'), transitions={'succeeded':'MOVE_BASE', 'preempted':'CLEAN_UP', 'failed':'MOVE_BASE'})
        smach.StateMachine.add(
            'MOVE_BASE',
            hobbit_move.goToPose(),
            transitions={'succeeded':'MOVE_HEAD', 'preempted':'CLEAN_UP', 'aborted':'CLEAN_UP'},
            remapping={'x':'goal_position_x',
                       'y':'goal_position_y',
                       'yaw':'goal_position_yaw'})
        smach.StateMachine.add('MOVE_HEAD', head_move.MoveTo(pose='search_table'), transitions={'succeeded':'REC_COUNTER', 'preempted':'CLEAN_UP', 'failed':'PLAN_PATH'})
        smach.StateMachine.add('REC_COUNTER', MoveCounter(), transitions={'succeeded':'GET_POINT_CLOUD', 'preempted':'aborted', 'failure':'PLAN_PATH'})
        smach.StateMachine.add('GET_POINT_CLOUD',
                               util.WaitForMsgState('/headcam/depth_registered/points', PointCloud2, point_cloud_cb, timeout=5, output_keys=['cloud']),
                               transitions={'succeeded':'START_OBJECT_RECOGNITION', 'aborted':'GET_POINT_CLOUD', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('START_OBJECT_RECOGNITION',
                               ServiceState('mp_recognition', recognize, request_slots=['cloud'], response_slots=['ids', 'transforms']),
                               #transitions={'succeeded':'OBJECT_DETECTED', 'preempted':'CLEAN_UP', 'aborted':'START_OBJECT_RECOGNITION'})
                               transitions={'succeeded':'OBJECT_DETECTED', 'preempted':'CLEAN_UP', 'aborted':'REC_COUNTER'})
        smach.StateMachine.add('OBJECT_DETECTED', ObjectDetected(), transitions={'succeeded':'GRASP_OBJECT', 'failure':'MOVE_HEAD', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('GRASP_OBJECT', DummyGrasp(), transitions={'succeeded':'SET_SUCCESS', 'failure':'CLEAN_UP', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('SET_SUCCESS', SetSuccess(), transitions={'succeeded':'succeeded', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('CLEAN_UP', CleanUp(), transitions={'succeeded':'preempted'})

    asw = smach_ros.ActionServerWrapper(
            'pickup', GeneralHobbitAction, pickup_sm,
            ['succeeded'], ['aborted'],['preempted'],
            result_slots_map = {'result':'result'},
            goal_slots_map = {'object_name':'object_name'})

    sis = smach_ros.IntrospectionServer('smach_server', pickup_sm, '/HOBBIT/PICKUP_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

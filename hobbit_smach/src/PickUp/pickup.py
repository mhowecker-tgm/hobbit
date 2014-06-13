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
from rgbd_acquisition.msg import PointEvents
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.head_move_import as head_move
import hobbit_smach.arm_move_import as arm_move
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.pickup_import as pickup
import hobbit_smach.return_of_favour_import as return_of_favour

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


def pointevents_cb(msg, ud):
    print('pointevents_cb')
    print(msg)
    return True

class CheckHelpAccepted(smach.State):
    """
    Just a counter in a SMACH State.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['yes', 'no', 'preempted'])
        self._help_accepted = 'no'

    def execute(self, ud):
        if self.preempt_requested():
            rospy.set_param('/hobbit/help_accepted', 'no')
            return 'preempted'
        if rospy.has_param('/hobbit/help_accepted'):
             self._help_accepted = rospy.get_param('/hobbit/help_accepted')
        return self._help_accepted



class GraspCounter(smach.State):
    """
    Just a counter in a SMACH State.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['first', 'second'])
        self._counter = 0

    def execute(self, ud):
        self._counter += 1
        print('self._counter: %d' % self._counter )
        if self._counter > 1:
            return 'second'
        else:
            return 'first'


class MoveCounter(smach.State):
    """
    Just a counter in a SMACH State.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['first', 'second'])
        self._counter = 0

    def execute(self, ud):
        self._counter += 1
        print('self._counter: %d' % self._counter )
        if self._counter > 1:
            return 'second'
        else:
            return 'first'


class Init(smach.State):
    """Class to initialize certain parameters"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'canceled'], input_keys=['object_name'], output_keys=['social_role'])

    def execute(self,ud):
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
            transitions={'succeeded': 'GET_POINTING_DIRECTION',
                         'canceled': 'CLEAN_UP'}
        )
        StateMachine.add(
            'GET_POINTING_DIRECTION',
            util.WaitForMsgState(
                '/pointEvents',
                PointEvents,
                msg_cb=pointevents_cb,
                timeout=5,
                output_keys=['pointEvent']
                ),
            transitions={'succeeded': 'START_LOOKING',
                         'aborted': 'POINTING_NOT_DETECTED_1',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'POINTING_NOT_DETECTED_1',
            pickup.sayPointingGestureNotDetected1(),
            transitions={'yes': 'GET_POINTING_DIRECTION',
                         'no': 'POINTING_NOT_DETECTED_2',
                         'preempted': 'preempted',
                         'failed': 'POINTING_NOT_DETECTED_2'}
        )
        StateMachine.add(
            'POINTING_NOT_DETECTED_2',
            pickup.sayPointingGestureNotDetected2(),
            transitions={'succeeded': 'aborted',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'START_LOOKING',
            pickup.getStartLooking(),
            transitions={'succeeded': 'LOOK_FOR_OBJECT',
                         'failed': 'EMO_SAY_OBJECT_NOT_DETECTED'}
        )
        StateMachine.add(
            'LOOK_FOR_OBJECT',
            pickup.DavidDummyLook(),
            transitions={'succeeded': 'CALC_GRASP_POSE',
                         'aborted': 'EMO_SAY_OBJECT_NOT_DETECTED',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'CALC_GRASP_POSE',
            pickup.DavidDummyCalcGraspPose(),
            transitions={'succeeded': 'EMO_SAY_OBJECT_FOUND',
                         'aborted': 'EMO_SAY_OBJECT_NOT_DETECTED',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'EMO_SAY_OBJECT_NOT_DETECTED',
            pickup.sayObjectNotDetected1(),
            transitions={'yes': 'GET_POINTING_DIRECTION',
                         'no': 'POINTING_NOT_DETECTED_2',
                         'preempted': 'preempted',
                         'failed': 'POINTING_NOT_DETECTED_2'}
        )
        StateMachine.add(
            'EMO_SAY_OBJECT_FOUND',
            pickup.sayObjectFoundRepositioning(),
            transitions={'succeeded': 'MOVE_TO_GRASP_POSE',
                         'failed': 'MOVE_TO_GRASP_POSE'}
        )
        StateMachine.add(
            'MOVE_TO_GRASP_POSE',
            hobbit_move.goToPose(),
            transitions={'succeeded': 'SAY_PICKING_UP',
                         'aborted': 'MOVE_COUNTER',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'MOVE_COUNTER',
            MoveCounter(),
            transitions={'first': 'EMO_SAY_UNABLE_TO_GRASP',
                         'second': 'EMO_SAY_TRY_TO_REMOVE_OBJECT'}
        )
        StateMachine.add(
            'EMO_SAY_UNABLE_TO_GRASP',
            pickup.sayUnableToGraspObject(),
            transitions={'yes': 'MOVE_TO_GRASP_POSE',
                         'no': 'EMO_SAY_TRY_TO_REMOVE_OBJECT',
                         'failed': 'EMO_SAY_TRY_TO_REMOVE_OBJECT',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'EMO_SAY_TRY_TO_REMOVE_OBJECT',
            pickup.sayTryToRemoveObject(),
            transitions={'succeeded': 'aborted',
                         'failed': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SAY_PICKING_UP',
            #speech_output.sayText(info='T_PU_PickingUpObject')
            speech_output.sayText(info='PickingUpObject'),
            transitions={'succeeded': 'GRASP_OBJECT',
                         'failed': 'EMO_SAY_DID_NOT_PICKUP',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'GRASP_OBJECT',
            pickup.DavidDummyPickingUp(),
            transitions={'succeeded': 'COUNTER_GRASP_CHECK',
                         'aborted': 'EMO_SAY_DID_NOT_PICKUP',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SAY_CHECK_GRASP',
            #speech_output.sayText(info='T_PU_CheckingGrasp')
            speech_output.sayText(info='CheckingGrasp'),
            transitions={'succeeded': 'CHECK_GRASP',
                         'failed': 'EMO_SAY_DID_NOT_PICKUP'}
        )
        StateMachine.add(
            'CHECK_GRASP',
            pickup.DavidDummyCheckGrasp(),
            transitions={'succeeded': 'PICKUP_SEQ',
                         'aborted': 'COUNTER_GRASP_CHECK'}
        )
        StateMachine.add(
            'COUNTER_GRASP_CHECK',
            GraspCounter(),
            transitions={'first': 'EMO_SAY_DID_NOT_PICKUP',
                         'second': 'EMO_SAY_DID_NOT_PICKUP_2'}
        )
        StateMachine.add(
            'EMO_SAY_DID_NOT_PICKUP',
            pickup.sayDidNotPickupObject1(),
            transitions={'succeeded': 'aborted',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'EMO_SAY_DID_NOT_PICKUP_2',
            pickup.sayDidNotPickupObject1(),
            transitions={'succeeded': 'aborted',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'PICKUP_SEQ',
            pickup.getPickupSeq(),
            transitions={'succeeded': 'CHECK_HELP',
                         'failed': 'EMO_SAY_DID_NOT_PICKUP',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'CHECK_HELP',
            CheckHelpAccepted(),
            transitions={'yes': 'SAY_THANK_YOU',
                         'no': 'SET_SUCCESS',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SAY_THANK_YOU',
            #speech_output.sayText(info='T_PU_ThankYouPointing'),
            speech_output.sayText(info='ThankYouPointing'),
            transitions={'succeeded': 'OFFER_RETURN_OF_FAVOR',
                         'failed': 'OFFER_RETURN_OF_FAVOR',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'OFFER_RETURN_OF_FAVOR',
            return_of_favour.offer(rof='pickup'),
            transitions={'succeeded': 'SET_SUCCESS',
                         'failed': 'SET_SUCCESS',
                         'preempted': 'preempted'}
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

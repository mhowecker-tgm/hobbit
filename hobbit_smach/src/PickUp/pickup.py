#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
PROJECT = 'Hobbit'
NAME = 'pickup_object'
LASTPNTDIR = None #last saved pointing direction
MAXPNTDIRDIF = 20.0 # maximal difference between two pointing directions
pubEvent = None

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import uashh_smach.util as util
from uashh_smach.util import SleepState, WaitForMsgState  # df 30.7.2014

from smach import StateMachine
from smach_ros import ActionServerWrapper, IntrospectionServer, ServiceState,\
    MonitorState
from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction
from hobbit_msgs.srv import SwitchVision, SwitchVisionRequest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from rgbd_acquisition.msg import PointEvents
from hobbit_msgs.msg import Event, Parameter
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.head_move_import as head_move
import hobbit_smach.arm_move_import as arm_move
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.pickup_import as pickup
import hobbit_smach.logging_import as log
import hobbit_smach.locate_user_simple_import as locate_user
# from hobbit_smach.helper_import import WaitForMsgState
# import tf
# import math


def switch_vision_cb(ud, response):
    if response.result:
        return 'succeeded'
    else:
        return 'aborted'


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

def point_other_state_cb(msg, ud):
    ret = pointevents_cb(ud, msg)
    return not ret

#def pointevents_cb(msg, ud):
def pointevents_cb(ud, msg):
    global LASTPNTDIR
    print('pointevents_cb')
    print (msg)
    
    if LASTPNTDIR is None:
        LASTPNTDIR = msg
        print "pointevents_cp: first iteration (no direction for comparison)"
        return True
    if (msg.vectorY < 0.0):
        print "Pointing direction is upwards, direction NOT ACCEPTED. Will try again."
        LASTPNTDIR = msg
        return True
    if (diff_pointing_directions(msg) > MAXPNTDIRDIF):
        print "difference between two consecutive pointing directions bigger than threshold: ", diff_pointing_directions(msg)
        LASTPNTDIR = msg
        return True

    
    ud.pointing_msg = msg
    print "a valid pointing direction was found and accepted!"
    LASTPNTDIR = None   # a pointing direction was found, hence the last pointing direction is set back for the next action
    return False    

def diff_pointing_directions(dir_new):
    return abs(LASTPNTDIR.vectorX - dir_new.vectorX) + abs(LASTPNTDIR.vectorY - dir_new.vectorY) + abs(LASTPNTDIR.vectorZ - dir_new.vectorZ)

#df: logging of parameter string for PickUp
def logdata(data):
    global pubEvent
    scenario = "PickUp"
    rospy.loginfo('LOG: scenario: %s: %s' % (scenario, data))
    message = Event()
    message.event = 'E_LOG'
    params = []
    par = Parameter(name='SCENARIO',
                    value=scenario)
    params.append(par)
    par = Parameter(name='DATA',
                    value=data)
    params.append(par)
    message.params = params
    pubEvent.publish(message)
    return True


#def point_cloud_cb(msg, ud):
def point_cloud_cb(ud, msg):
    print('point cloud received')
    ud.cloud = msg
    return False


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


class PointingCounter(smach.State):
    """
    Just a counter in a SMACH State.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['first', 'second', 'preempted'])
        self._counter = 0

    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        self._counter += 1
        print('self._counter: %d' % self._counter)
        if self._counter > 1:
            self._counter = 0
            return 'second'
        else:
            return 'first'


class GraspCounter(smach.State):
    """
    Just a counter in a SMACH State.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['first', 'second', 'preempted'])
        self._counter = 0

    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        self._counter += 1
        print('self._counter: %d' % self._counter)
        if self._counter > 1:
            self._counter = 0
            return 'second'
        else:
            return 'first'


class MoveCounter(smach.State):
    """
    Just a counter in a SMACH State.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['first', 'second', 'preempted'])
        self._counter = 0

    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        self._counter += 1
        print('self._counter: %d' % self._counter)
        if self._counter > 1:
            self._counter = 0
            return 'second'
        else:
            return 'first'
        
#df 27.4.2015 counter how often it is searched for a graspable object (if arm has space, trajectory found for grasp, grasp point evaluation reasonable, object graspable)
# alternately returns "first" and "second"
class SearchGraspableObjectCounter(smach.State):
    """
    Just a counter in a SMACH State.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['first', 'second', 'preempted'])
        self._counter = 0

    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        self._counter += 1
        print('self._counter: %d' % self._counter)
        if self._counter > 1:
            self._counter = 0
            return 'second'
        else:
            return 'first'
        
#df 20.3.2015: count how often robot should move to other grasping position (1 time overall <=> blind backwards move))
class MoveBackBlindCounter(smach.State):
    """
    Just a counter in a SMACH State.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['first', 'second', 'preempted'])
        self._counter = 0

    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        self._counter += 1
        print('self._counter: %d' % self._counter)
        if self._counter > 1:
            self._counter = 0
            return 'second'
        else:
            return 'first'


class Init(smach.State):
    """Class to initialize certain parameters"""
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'canceled', 'preempted'],
            output_keys=['social_role'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        logdata("1001PUS   PickUp Started") #Pickup Started
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        return 'succeeded'


class CleanUp(smach.State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['command', 'visited_places'],
            output_keys=['result', 'command', 'visited_places'])

    def execute(self, ud):
        ud.visited_places = []
        ud.result = String('object not found')
        return 'succeeded'


class SetSuccess(smach.State):
    """
    Class for setting the success message in the
    actionlib result and clean up of persistent variables
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['result'],
                             output_keys=['result', 'visited_places'])

    def execute(self, ud):
        ud.visited_places = []
        pub = rospy.Publisher('/DiscreteMotionCmd', String, queue_size=50)
        rospy.sleep(1.0)
        pub.publish('Stop')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('object detected')
        rospy.loginfo(ud.result)
        return 'succeeded'


class DummyGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'])

    def execute(self, ud):
        rospy.loginfo('Start DUMMY grasp')
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'succeeded'


def get_robot_pose_cb(msg, ud):
    try:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = msg.pose.pose.position
        pose.pose.orientation = msg.pose.pose.orientation
        ud.robot_current_pose = pose
        return True
    except:
        print bcolors.FAIL + 'no robot pose message received.' + bcolors.ENDC
        return False


def main():
    rospy.init_node(NAME)
    global pubEvent
    pubEvent = rospy.Publisher('Event', Event, queue_size=50)


    pickup_sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        output_keys=['result'])

    pickup_sm.userdata.result = String('started')
    pickup_sm.userdata.detection = False

    with pickup_sm:
        StateMachine.add(
            'SET_HEAD',
            head_move.MoveTo(pose='center_center'),
            transitions={'succeeded': 'INIT',
                         'preempted': 'LOG_PREEMPT'}
        )
        #StateMachine.add(
        #    'SET_HEAD_SKIP_VISION_STATES',
        #    head_move.MoveTo(pose='center_center'),
        #    transitions={'succeeded': 'SAY_LOOK',
        #                 'preempted': 'LOG_PREEMPT'}
        #)
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'SWITCH_VISION',
                         'canceled': 'CLEAN_UP',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'SWITCH_VISION',
            ServiceState(
                '/vision_system/seeWhereUserIsPointing',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb

            ),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'preempted': 'LOG_PREEMPT'}

        )
        smach.StateMachine.add_auto(
            'SAY_LOOK',
            speech_output.sayText(info='T_POINT_TO_OBJECT'),
            connector_outcomes=['succeeded', 'failed'],
            transitions={'preempted': 'LOG_PREEMPT'}
        )
        smach.StateMachine.add(
             'GET_POINTING_DIRECTION',
             MonitorState(
                 '/pointEvents',
                 PointEvents,
                 cond_cb=pointevents_cb,
                 max_checks=20,
                 output_keys=['pointing_msg']
             ),
             transitions={'valid': 'POINTING_COUNTER',
                          'invalid': 'LOG_POINTING_DIRECTION_ACCEPTED',   
                          'preempted': 'LOG_PREEMPT'}
         )
        StateMachine.add(
            'POINTING_COUNTER',
            PointingCounter(),
            transitions={'first': 'LOG_POINTING_NOT_DETECTED_1',
                         'second': 'POINTING_NOT_DETECTED_2',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_POINTING_NOT_DETECTED_1',
            log.DoLogScenarioAndData(data='1203PDR   Pointing Direction Rejected'),
            transitions={'succeeded': 'POINTING_NOT_DETECTED_1'}
        )
        StateMachine.add(
            'POINTING_NOT_DETECTED_1',
            pickup.sayPointingGestureNotDetected1(),
            transitions={'yes': 'GET_POINTING_DIRECTION',
                         'no': 'POINTING_DECLINED_BY_USER',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'POINTING_NOT_DETECTED_2'}
        )
        StateMachine.add(
            'POINTING_NOT_DETECTED_2',
            pickup.sayPointingGestureNotDetected2(),
            transitions={'succeeded': 'LOG_ABORT',
                         'failed': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'POINTING_DECLINED_BY_USER',
            pickup.sayPointingDeclinedByUser(),
            transitions={'succeeded': 'LOG_ABORT',
                         'failed': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_POINTING_DIRECTION_ACCEPTED',
            log.DoLogScenarioAndData(data='1102PDA   Pointing Direction Accepted'),
            transitions={'succeeded': 'START_LOOKING'}
        )
        StateMachine.add(
            'START_LOOKING',
            pickup.getStartLooking(),
            transitions={'succeeded': 'HEAD_TO_SEARCH',
                         'failed': 'EMO_SAY_MOVE_ISSUE_PU_STOPPED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'HEAD_TO_SEARCH',
            head_move.MoveTo(pose='to_grasp'),  # wait=True <=> df
            transitions={'succeeded': 'WAIT_FINISH_HEAD_TO_SEARCH', # df
                         'aborted': 'EMO_SAY_OBJECT_NOT_DETECTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        # df 30.7.2014
        StateMachine.add(
            'WAIT_FINISH_HEAD_TO_SEARCH',
            SleepState(duration=5),
            transitions={'succeeded': 'GET_POINT_CLOUD',
                         'preempted': 'LOG_PREEMPT'}
            )
        # df END
        smach.StateMachine.add(
            'GET_POINT_CLOUD',
            MonitorState(
                '/headcam/depth_registered/points',
                PointCloud2,
                cond_cb=point_cloud_cb,
                max_checks=20,
                output_keys=['cloud']
            ),
            transitions={'valid': 'GET_POINT_CLOUD',
                         'invalid': 'LOOK_FOR_OBJECT',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOOK_FOR_OBJECT',
            pickup.DavidLookForObject(),
            transitions={'succeeded': 'EMO_SAY_OBJECT_FOUND',
                         'failed': 'EMO_SAY_OBJECT_NOT_DETECTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'EMO_SAY_OBJECT_NOT_DETECTED',
            pickup.sayObjectNotDetected(),
            transitions={'succeeded': 'SET_HEAD_CENTER_PICKUP_FAILED',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'SET_HEAD_CENTER_PICKUP_FAILED'}
        )
        StateMachine.add(
            'EMO_SAY_OBJECT_FOUND',
            pickup.sayObjectFoundRepositioning(),
            #transitions={'succeeded': 'MOVE_TO_GRASP_POSE',
            #                         'failed': 'MOVE_TO_GRASP_POSE'}
            transitions={'succeeded': 'MOVE_TO_GRASP_POSE_REL_MOVEMENT_BLIND',
                         'failed': 'MOVE_TO_GRASP_POSE_REL_MOVEMENT_BLIND',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'MOVE_TO_GRASP_POSE',
            hobbit_move.goToPoseSilent(),
            transitions={'succeeded': 'HEAD_TO_GRASP_POSITION',
                         'aborted': 'MOVE_COUNTER',
                         'preempted': 'LOG_PREEMPT'},
            remapping={'x': 'goal_position_x',
                       'y': 'goal_position_y',
                       'yaw': 'goal_position_yaw'}
        )
        #df new 27.2.2015
        StateMachine.add(
            'MOVE_TO_GRASP_POSE_REL_MOVEMENT_BLIND',        #!!!!!!!!!!!!!!!! TODO check if robot can go to position safely!!!!!!!!!!!!!!
            pickup.GoToFinalGraspPose(),
            transitions={'succeeded': 'HEAD_TO_GRASP_POSITION',
                         'aborted': 'MOVE_COUNTER',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'HEAD_TO_GRASP_POSITION',
            head_move.MoveTo(pose='to_grasp'),
            transitions={'succeeded': 'WAIT_FINISH_HEAD_TO_GRASP_POSITION', # df
                         'aborted': 'EMO_SAY_OBJECT_NOT_DETECTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        # df 30.7.2014
        StateMachine.add(
            'WAIT_FINISH_HEAD_TO_GRASP_POSITION',
            SleepState(duration=5),
            transitions={'succeeded': 'GET_POINT_CLOUD_FOR_GRASP',
                         'preempted': 'LOG_PREEMPT'}
            )
        StateMachine.add(
            'MOVE_COUNTER',
            MoveCounter(),
            transitions={'first': 'EMO_SAY_UNABLE_TO_MOVE',
                         'second': 'EMO_SAY_MOVE_ISSUE_PU_STOPPED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'EMO_SAY_UNABLE_TO_MOVE',
            pickup.sayUnableToMove(),
            transitions={'yes': 'MOVE_TO_GRASP_POSE',
                         'no': 'EMO_SAY_MOVE_ISSUE_PU_STOPPED',
                         'failed': 'EMO_SAY_MOVE_ISSUE_PU_STOPPED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'EMO_SAY_MOVE_ISSUE_PU_STOPPED',
            pickup.sayMoveIssuePUstopped(),
            transitions={'succeeded': 'LOG_MOVE_ISSUE_PU_STOPPED',
                         'failed': 'LOG_MOVE_ISSUE_PU_STOPPED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_MOVE_ISSUE_PU_STOPPED',
            log.DoLogScenarioAndData(data="2205MPF Moving to (Pregrasp)pos Failed"),
            transitions={'succeeded': 'SET_HEAD_CENTER_PICKUP_FAILED'}
        )
        #StateMachine.add(   #not used anymore!! df: 6.2.2015, done in pickup_import
        #    'NO_GRASPABLE_OBJECT_FOUND',
        #    speech_output.sayText(info='T_PU_PickingUpObject'),
        #    transitions={'succeeded': 'GET_POINT_CLOUD_FOR_GRASP',
        #                 'failed': 'EMO_SAY_DID_NOT_PICKUP',
        #                 'preempted': 'LOG_PREEMPT'}
        #)
        
        #================> NEW 10.12.2014
        smach.StateMachine.add(
            'GET_POINT_CLOUD_FOR_GRASP',
            MonitorState(
                '/headcam/depth_registered/points',
                PointCloud2,
                cond_cb=point_cloud_cb,
                max_checks=20,
                output_keys=['cloud']
            ),
            transitions={'valid': 'GET_POINT_CLOUD_FOR_GRASP',
                         'invalid': 'GRASP_OBJECT',
                         'preempted': 'LOG_PREEMPT'}
        )        
        
        # currently not used
        #smach.StateMachine.add(
        #    'MOVE_ARM_TO_PRE_GRASP_POSITION',
        #    arm_move.goToPreGraspPosition(),
        #    transitions={'succeeded': 'GRASP_OBJECT', 
        #                 'preempted': 'LOG_PREEMPT',
        #                 'failed': 'MOVE_ARM_TO_PRE_GRASP_POSITION'}    # better failure handling appreciated
        #)       
        StateMachine.add(
            'GRASP_OBJECT',
            pickup.getPickupSeq(), #
            #pickup.DavidPickingUp(),
            transitions={'succeeded': 'SAY_CHECK_GRASP',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'SEARCH_GRASPABLE_OBJECT_COUNTER',
                         'failed_arm_not_moved': 'SEARCH_GRASPABLE_OBJECT_COUNTER' #not moved away from home position
                        }
        )  
        #================> NEW 10.12.2014  ENDE      
        # counts (binary) how often the system searches for graspable objects
        StateMachine.add(
            'SEARCH_GRASPABLE_OBJECT_COUNTER',
            SearchGraspableObjectCounter(),
            transitions={'first': 'SAY_SEARCH_GRASPABLE_OBJECT_NEXT_TRY',
                         'second': 'SAY_SEARCH_GRASPABLE_OBJECT_FAILED',
                         'preempted': 'LOG_PREEMPT'}
        )       
        
        StateMachine.add(
            'SAY_SEARCH_GRASPABLE_OBJECT_NEXT_TRY',
            speech_output.sayText(info='T_PU_SAY_SEARCH_GRASPABLE_OBJECT_NEXT_TRY'),
            transitions={'succeeded': 'GET_POINT_CLOUD_FOR_GRASP',
                         'failed': 'GET_POINT_CLOUD_FOR_GRASP',
                         'preempted': 'LOG_PREEMPT'}
        ) 

        StateMachine.add(
            'SAY_SEARCH_GRASPABLE_OBJECT_FAILED',
            speech_output.sayText(info='T_PU_SAY_SEARCH_GRASPABLE_OBJECT_FAILED'),
            transitions={'succeeded': 'SET_HEAD_CENTER_PICKUP_FAILED',
                         'failed': 'SET_HEAD_CENTER_PICKUP_FAILED',
                         'preempted': 'LOG_PREEMPT'}
        ) 
        StateMachine.add(
            'SAY_CHECK_GRASP',
            speech_output.sayText(info='T_PU_CheckingGrasp'),
            transitions={'succeeded': 'MOVE_ARM_TO_CHECK_GRASP_POSITION',
                         'failed': 'EMO_SAY_DID_NOT_PICKUP',
                         'preempted': 'LOG_PREEMPT'}
        )
        
        
        StateMachine.add(
            'MOVE_ARM_TO_CHECK_GRASP_POSITION',   #position where gripper is not blocking view to floor where object was lying
            arm_move.goToCheckGraspPosition(),
            transitions={'succeeded': 'GET_POINT_CLOUD_FOR_GRASPCHECK', 
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'MOVE_ARM_TO_CHECK_GRASP_POSITION'}    # better failure handling appreciated
        )
        #================> NEW 29.1.2015 df
        # still missing: timeout procedure
        smach.StateMachine.add(
            'GET_POINT_CLOUD_FOR_GRASPCHECK',
            MonitorState(
                '/headcam/depth_registered/points',
                PointCloud2,
                cond_cb=point_cloud_cb,
                max_checks=20,
                output_keys=['cloud']
            ),
            transitions={'valid': 'GET_POINT_CLOUD_FOR_GRASPCHECK',
                         'invalid': 'MOVE_ARM_TO_PRE_GRASP_POSITION_MANUALLY',
                         'preempted': 'LOG_PREEMPT'}
        )        
        #df new 5.2.2015
        StateMachine.add(
            'MOVE_ARM_TO_PRE_GRASP_POSITION_MANUALLY',   #position where gripper is not blocking view to floor where object was lying
            arm_move.goToPreGraspPositionManually(),
            transitions={'succeeded': 'CHECK_GRASP', 
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'MOVE_ARM_TO_PRE_GRASP_POSITION_MANUALLY'}    # better failure handling appreciated
        )
        StateMachine.add(
            'CHECK_GRASP',
            pickup.DavidCheckGrasp(),
            transitions={'succeeded': 'END_PICKUP_SEQ',
                         'aborted': 'COUNTER_GRASP_CHECK',
                         'preempted': 'LOG_PREEMPT'}  #aborted <=> check result: no object grasped
        )
        StateMachine.add(
            'COUNTER_GRASP_CHECK',
            GraspCounter(),
            transitions={'first': 'EMO_SAY_DID_NOT_PICKUP',
                         'second': 'EMO_SAY_DID_NOT_PICKUP_2',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'EMO_SAY_DID_NOT_PICKUP',
            pickup.sayDidNotPickupObject1(),
            transitions={'succeeded': 'MOVE_ARM_TO_HOME_POSITION',
                         'failed': 'MOVE_ARM_TO_HOME_POSITION',
                         'preempted': 'LOG_PREEMPT'}
        )
        #df new 5.2.2015
        StateMachine.add(
            'MOVE_ARM_TO_HOME_POSITION',
            arm_move.goToHomePosition(),
            transitions={'succeeded': 'GET_POINT_CLOUD_FOR_GRASP',  #=> try to grasp again without changing position (assume: head is still lookingt to object"
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'MOVE_ARM_TO_HOME_POSITION'}    # better failure handling appreciated
        )
        #df new 17.3.2015a
        StateMachine.add(
            'MOVE_TO_BETTER_POSE_TO_REVIEW_OBJECT',        
            pickup.MoveRobotBackForBetterObjectView(),
            transitions={'succeeded': 'HEAD_TO_SEARCH',
                         'aborted': 'MOVE_COUNTER',
                         'preempted': 'LOG_PREEMPT'}
        )        
        #df enf 17.3.2015
        StateMachine.add(
            'EMO_SAY_DID_NOT_PICKUP_2',
            pickup.sayDidNotPickupObjectTwoTimes(),
            transitions={'succeeded': 'MOVE_ARM_TO_HOME_POSITION_AFTER_FAILED',
                         'failed': 'EMO_SAY_DID_NOT_PICKUP_2',
                         'preempted': 'LOG_PREEMPT'}
        )
        #df new 25.2.2015
        StateMachine.add(
            'MOVE_ARM_TO_HOME_POSITION_AFTER_FAILED',  #done after 2 times unsuccessful grasped
            arm_move.goToHomePosition(),
            transitions={'succeeded': 'MOVE_BLIND_BACK_COUNTER',  #do this only ones
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'MOVE_ARM_TO_HOME_POSITION_AFTER_FAILED'}    # better failure handling appreciated
        )   
        #new 20.3.2015: only move back blind once
        StateMachine.add(
            'MOVE_BLIND_BACK_COUNTER',
            MoveBackBlindCounter(),
            transitions={'first': 'MOVE_TO_BETTER_POSE_TO_REVIEW_OBJECT',
                         'second': 'EMO_SAY_PICKUP_FAILED',
                         'preempted': 'LOG_PREEMPT'}
        )        
        #df 23.3.2015
        StateMachine.add(
            'EMO_SAY_PICKUP_FAILED',
            pickup.sayDidNotPickupObject2(),
            transitions={'succeeded': 'SET_HEAD_CENTER_PICKUP_FAILED',
                         'failed': 'EMO_SAY_PICKUP_FAILED',
                         'preempted': 'LOG_PREEMPT'}
        )
        #df 23.3.2015
        StateMachine.add(
            'SET_HEAD_CENTER_PICKUP_FAILED',
            head_move.MoveTo(pose='center_center'),
            transitions={'succeeded': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'}
        )        
        #df 23.3.2015
        StateMachine.add(
            'SET_HEAD_CENTER_PICKUP_SUCCEEDED',
            head_move.MoveTo(pose='center_center'),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'LOG_PREEMPT'}
        )    
        StateMachine.add(
            'END_PICKUP_SEQ',
            pickup.getEndPickupSeq(),
            transitions={'succeeded': 'SAY_THANK_YOU',
                         'failed': 'EMO_SAY_DID_NOT_PICKUP',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'SAY_THANK_YOU',
            speech_output.sayText(info='T_PU_ThankYouPointing'),
            transitions={'succeeded': 'SET_SUCCESS',
                         'failed': 'SET_SUCCESS',
                         'preempted': 'LOG_PREEMPT'}
        )
        #StateMachine.add(
        #    'GO_TO_USER',
        #    locate_user.get_detect_user(),
        #    transitions={'succeeded': 'SET_SUCCESS',
        #                 'preempted': 'LOG_PREEMPT',
        #                 'aborted': 'LOG_PREEMPT'}
        #)
        StateMachine.add(
            'SET_SUCCESS',
            SetSuccess(),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'preempted': 'CLEAN_UP'}
        )
        StateMachine.add(
            'CLEAN_UP',
            CleanUp(),
            transitions={'succeeded': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Pickup'),
            transitions={'succeeded': 'SET_HEAD_CENTER_PICKUP_SUCCEEDED'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Pickup'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORT',
            log.DoLogAborted(scenario='Pickup'),
            transitions={'succeeded': 'aborted'}
        )

    asw = ActionServerWrapper(
        'pickup', GeneralHobbitAction, pickup_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        result_slots_map={'result': 'result'}
    )

    sis = IntrospectionServer(
        'smach_server',
        pickup_sm,
        '/HOBBIT/PICKUP_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

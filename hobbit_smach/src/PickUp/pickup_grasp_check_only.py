#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
PROJECT = 'Hobbit'
NAME = 'pickup_object'
LASTPNTDIR = None #last saved pointing direction
MAXPNTDIRDIF = 20.0 # maximal difference between two pointing directions

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import uashh_smach.util as util
from uashh_smach.util import SleepState, WaitForMsgState  # df 30.7.2014
# import tf
# import math
from smach import StateMachine
from smach_ros import ActionServerWrapper, IntrospectionServer, ServiceState,\
    MonitorState
from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction
from hobbit_msgs.srv import SwitchVision, SwitchVisionRequest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from rgbd_acquisition.msg import PointEvents
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.head_move_import as head_move
import hobbit_smach.arm_move_import as arm_move
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.pickup_import as pickup
import hobbit_smach.logging_import as log
# from hobbit_smach.helper_import import WaitForMsgState


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
        smach.State.__init__(self, outcomes=['first', 'second'])
        self._counter = 0

    def execute(self, ud):
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
        smach.State.__init__(self, outcomes=['first', 'second'])
        self._counter = 0

    def execute(self, ud):
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
        smach.State.__init__(self, outcomes=['first', 'second'])
        self._counter = 0

    def execute(self, ud):
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
            outcomes=['succeeded', 'canceled'],
            output_keys=['social_role'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
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

    pickup_sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        output_keys=['result'])

    pickup_sm.userdata.result = String('started')
    pickup_sm.userdata.detection = False

    with pickup_sm:
        StateMachine.add(
            'HEAD_TO_SEARCH',
            head_move.MoveTo(pose='to_grasp'),  # wait=True <=> df
            transitions={'succeeded': 'WAIT_FINISH_HEAD_TO_SEARCH', # df
                         'aborted': 'LOG_PREEMPT',
                         'preempted': 'LOG_PREEMPT'}
        )
        # df 30.7.2014
        StateMachine.add(
            'WAIT_FINISH_HEAD_TO_SEARCH',
            SleepState(duration=3),
            transitions={'succeeded': 'SAY_CHECK_GRASP',
                         'preempted': 'LOG_PREEMPT'}
            )
        # df END
        StateMachine.add(
            'SAY_CHECK_GRASP',
            speech_output.sayText(info='T_PU_CheckingGrasp'),
            transitions={'succeeded': 'GET_POINT_CLOUD_FOR_GRASPCHECK',
                         'failed': 'EMO_SAY_DID_NOT_PICKUP'}
        )
        
        
        #================> NEW 29.1.2015 df
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
                         'invalid': 'CHECK_GRASP',
                         'preempted': 'LOG_PREEMPT'}
        )        
        # df end
        
        
        StateMachine.add(
            'CHECK_GRASP',
            pickup.DavidCheckGrasp(),
            transitions={'succeeded': 'aborted',
                         'aborted': 'aborted'}
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
            transitions={'succeeded': 'SAY_PICKING_UP',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'EMO_SAY_DID_NOT_PICKUP_2',
            pickup.sayDidNotPickupObject2(),
            transitions={'succeeded': 'aborted',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'END_PICKUP_SEQ',
            pickup.getEndPickupSeq(),
            transitions={'succeeded': 'CHECK_HELP',
                         'failed': 'EMO_SAY_DID_NOT_PICKUP',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'CHECK_HELP',
            CheckHelpAccepted(),
            transitions={'yes': 'SAY_THANK_YOU',
                         'no': 'SET_SUCCESS',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'SAY_THANK_YOU',
            speech_output.sayText(info='T_PU_ThankYouPointing'),
            transitions={'succeeded': 'SET_SUCCESS',
                         'failed': 'SET_SUCCESS',
                         'preempted': 'LOG_PREEMPT'}
        )
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
            transitions={'succeeded': 'succeeded'}
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

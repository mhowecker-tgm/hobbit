#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'hobbit_move'
DEBUG = True

import roslib
roslib.load_manifest(PKG)
import rospy

from smach import State, Sequence, StateMachine
#import uashh_smach.util as util
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.speech_output_import as speech_output


class CalcPoseAtDistance(State):
    """
    Read the user's current pose from userdata and calculates the pose that
    the robot should move to before trying to get the user's attention.

    """
    def __init__(self, angle=0):
        State.__init__(
            self,
            input_keys=['user_pose', 'x', 'y', 'yaw'],
            output_keys=['x', 'y', 'yaw'],
            outcomes=['succeeded', 'preempted', 'aborted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if DEBUG:
            print(ud.user_pose)
            print(ud.x, ud.y, ud.yaw)
        # TODO: clever calculation so that we are always close to the user
        return 'succeeded'


class CalcPoseNearUser(State):
    """
    Read the user's current pose from userdata and calculates the pose that
    the robot should move to for touchscreen-based interaction.

    """
    def __init__(self, angle=0):
        State.__init__(
            self,
            input_keys=['user_pose', 'x', 'y', 'yaw'],
            output_keys=['x', 'y', 'yaw'],
            outcomes=['succeeded', 'preempted', 'aborted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if DEBUG:
            print(ud.user_pose)
            print(ud.x, ud.y, ud.yaw)
        # TODO: clever calculation so that we are always close to the user
        return 'succeeded'


class CheckSocialRole(State):
    """
    """
    def __init__(self, angle=0):
        State.__init__(
            self,
            outcomes=['0', '1', '2', '3', '4', 'preempted', 'aborted']
        )
        self._angle = angle

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        social_role = rospy.get_param('social_role')
        if DEBUG:
            print(social_role)
        if 0 <= int(social_role) < 5:
            return social_role
        else:
            return 'aborted'


def approachUser():
    """
    Return a SMACH state machine that will move the robot to the user and
    depending on the social_role stop at a given distance to make some noise
    to get the user's attention.
    """

    seq1 = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded'
    )
    seq2 = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded'
    )
    sm = StateMachine(
        outcomes=['succeeded', 'preempted', 'aborted']
    )

    with seq1:
        Sequence.add('GET_ROBOT_POSE', hobbit_move.getRobotPose())
        Sequence.add('CALC_POSE_NEAR_USER', CalcPoseNearUser())
        Sequence.add('MOVE_TO_POSE', hobbit_move.goToPose)
        # TODO: What is the correct text ID to get attention from the user
        Sequence.add('GET_ATTENTION', speech_output.sayText(info='Hey there'))

    with seq2:
        Sequence.add('GET_ROBOT_POSE', hobbit_move.getRobotPose())
        Sequence.add('CALC_POSE_AT_DISTANCE', CalcPoseAtDistance())
        Sequence.add('MOVE_TO_POSE', hobbit_move.goToPose)

    with sm:
        StateMachine.add(
            'CHECK_SOCIAL_ROLE',
            CheckSocialRole(),
            transitions={'preempted': 'preempted',
                         'aborted': 'aborted',
                         '0': 'APPROACH_WITH_PAUSE',
                         '1': 'APPROACH_WITH_PAUSE',
                         '2': 'APPROACH_WITH_PAUSE',
                         '3': 'APPROACH_DIRECT',
                         '4': 'APPROACH_DIRECT'}
        )

        StateMachine.add(
            'APPROACH_WITH_PAUSE',
            seq1,
            transitions={'succeeded': 'APPROACH_DIRECT',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )

        StateMachine.add(
            'APPROACH_DIRECT',
            seq2,
            transitions={'succeeded': 'succeeded',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )

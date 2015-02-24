#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'SocialRole'

import roslib
roslib.load_manifest(PKG)

from smach import Sequence, State, StateMachine
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.hobbit_move_import as hobbit_move



def approach_seq1():
    """
    Returns a SMACH Sequence that will take the given user pose,
    transforms it to world coordinates and based on the social role
    adapts the approach to the user.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'CALC_END_POSE',
            CalculateEndPose()
        )
        Sequence.add(
            'MOVE_TO_END_POSE',
            hobbit_move.goToPose()
        )
    return seq


def approach_seq2():
    """
    Returns a SMACH Sequence that will take the given user pose,
    transforms it to world coordinates and based on the social role
    calculates an intermediate pose.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'CALC_INT_POSE',
            CalculateEndPose()
        )
        Sequence.add(
            'MOVE_TO_INT_POSE',
            hobbit_move.goToPose()
        )
        # Sequence.add(
        #     'MAKE_SOME_NOISE',
        #     # TODO: Write functionality for this.
        # )
    return seq


def approach_seq():
    """
    Returns a SMACH StateMachine that does the approach the user scenario.
    """
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted']
    )
    with sm:
        StateMachine.add(
            'SOCIAL_ROLE_CHECK',
            socialRoleCheck(),
            transitions={'tool': 'GOTO_END_POSE',
                         'butler': 'GOTO_END_POSE',
                         'companion': 'GOTO_INT_POSE'}
        )
        StateMachine.add(
            'GOTO_INT_POSE',
            approach_seq2,
            transitions={'succeeded': 'GOTO_END_POSE',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'GOTO_END_POSE',
            approach_seq2,
            transitions={'succeeded': 'succeeded',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )

class CalculateNextPose(State):
    """
    Given the user pose in world coordinates calculate the pose at
    which the robot should stop during its approach to the user.
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            output_keys=['x', 'y', 'yaw']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'succeeded'


class CalculateEndPose(State):
    """
    Given the user pose in world coordinates calculate the pose at
    which the robot should be at the end of its approach to the user.
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            output_keys=['x', 'y', 'yaw']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'succeeded'


class socialRoleCheck(State):
    """
    This class checks the social_role of the robot and returns the correct
    outcome that corresponds with it.
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['butler', 'companion', 'tool', 'preempted'],
            input_keys=['social_role']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if ud.social_role == 0:
            return 'tool'
        elif 1 <= ud.social_role <= 3:
            return 'butler'
        else:
            return 'companion'



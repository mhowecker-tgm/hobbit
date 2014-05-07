#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'ArmMove'
DEBUG = True

import roslib
roslib.load_manifest(PKG)

from smach import Sequence, State
from uashh_smach.util import SleepState
#import uashh_smach.util as util
from ArmControllerClientFunctions import ArmClientFunctions

def getArm():
    return ArmClientFunctions('192.168.2.190')


class SetMoveToLearningPos(State):
    """
    Move the arm to the turntable, grasp it and move to the Learning position.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        arm = getArm()
        if not arm.GetArmIsEnabled():
            return 'failed'
        status = arm.SetMoveToLearningPos()
        if status[0] == 'MoveToLearningPos' and status[1] == 'COMMAND_OK':
            return 'succeeded'
        else:
            return 'failed'


class SetArmPosition(State):
    """
    Move the robotarm to a specified position
    """
    def __init__(self, position):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )

    def execute(self, ud):
        if DEBUG:
            return 'succeeded'


class ArmReachedPosition(State):
    """
    Did the arm reach the specified position yet?
    """
    def __init__(self, position):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )

    def execute(self, ud):
        if DEBUG:
            return 'succeeded'


class OpenGripper(State):
    """
    Send the Close Gripper command to the arm
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )

    def execute(self, ud):
        if DEBUG:
            return 'succeeded'


class CloseGripper(State):
    """
    Send the Close Gripper command to the arm
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )

    def execute(self, ud):
        if DEBUG:
            return 'succeeded'


class CheckGripperClosed(State):
    """
    Ccheck that the gripper is really closed
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )

    def execute(self, ud):
        if DEBUG:
            return 'succeeded'


def goToPosition(pose='storage'):
    """
    Return a SMACH Sequence that will move the arm to the specified pose.
    The default values will move the arm to the storage pose.

    pose: defaults to storage
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('MOVE_ARM_TO_POSE', SetArmPosition(position=pose))
        Sequence.add('ARM_POSE_REACHED', ArmReachedPosition(position=pose),
                     transitions={'failed': 'ARM_POSE_REACHED'})
    return seq


def closeGripper():
    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('CLOSE_GRIPPER', CloseGripper())
        Sequence.add('WAIT_FOR_GRIPPER', SleepState(duration=2))
        Sequence.add('GRIPPER_CLOSED', CheckGripperClosed())
    return seq

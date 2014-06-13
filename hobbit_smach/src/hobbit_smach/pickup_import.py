#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'pickup_import'
DEBUG = True

import roslib
roslib.load_manifest(PKG)
# import rospy
import uashh_smach.util as util

from smach import Concurrence, Sequence , State
from hobbit_user_interaction import HobbitEmotions  ,HobbitMMUI
import hobbit_smach.speech_output_import as speech_output
# import hobbit_smach.head_move_import as head_move
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.arm_move_import as arm_move

class DavidLookForObject(State):
    """
    This state is called after the robot moved to a position from where it should be
    able to observe the floor and find an object there.
    The head will already be looking down to the floor.
    The pose of an object has to be stored in the userdata key object_pose
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'],
            input_keys=['goal_position_x', 'goal_position_y', 'goal_position_yaw'],
            output_keys=['goal_position_x', 'goal_position_y', 'goal_position_yaw']
        )
    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        # TODO: David please put the pose calculations in here
        # if you need any specific input data (point cloud) you can either
        # subscribe to a topic yourself or get the data from and input key which i have
        # to provide you with
        ud.object_pose.position.x = 0
        ud.object_pose.position.y = 0
        ud.object_pose.position.z = 0
        ud.object_pose.orientation.x = 0
        ud.object_pose.orientation.y = 0
        ud.object_pose.orientation.z = 0
        ud.object_pose.orientation.w = 0

        return 'succeeded'

class DavidLookingPose(State):
    """
    This state should handle the following task.
    Given the data from the pointing gesture (x,y,z,vectorX, vectorY, vectorZ)
    a Pose is calculated to which the robot will then navigate. This pose has
    to be stored inside the userdata output keys.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['pointing_msg', 'goal_position_x', 'goal_position_y', 'goal_position_yaw'],
            output_keys=['goal_position_x', 'goal_position_y', 'goal_position_yaw']
        )
    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        print(ud.pointing_msg)
        # TODO: David please put the pose calculations in here

        ud.goal_position_x = 0
        ud.goal_position_y = 0
        ud.goal_position_yaw = 0
        return 'succeeded'

class DavidCalcGraspPose(State):
    """
    This state should handle the following task.
    Given the Pose data given in the object_pose a Pose is calculated to
    which the robot will then navigate. This pose has to be stored inside
    the userdata output keys.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['object_pose', 'goal_position_x', 'goal_position_y', 'goal_position_yaw'],
            output_keys=['goal_position_x', 'goal_position_y', 'goal_position_yaw']
        )
    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        print(ud.object_pose)
        # TODO: David please put the pose calculations in here

        ud.goal_position_x = 0
        ud.goal_position_y = 0
        ud.goal_position_yaw = 0
        return 'succeeded'


class DavidPickingUp(State):
    """
    This state moves the arm from the PreGrasping position to the object,
    closes the gripper and returns 'succeeded' afterwards.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )
    def execute(self, ud):
        if self.preempt_requested():
            return 'preempted'
        # TODO: David please add the grasping in here

        return 'succeeded'


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
            HobbitMMUI.AskYesNo(question='PointingGestureNotDetected1')
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
            speech_output.sayText(info='PointingGestureNotDetected2')
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
            # speech_output.sayText(info='T_PU_StartLooking')
            speech_output.sayText(info='StartLooking')
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
            # speech_output.askYesNo(question='ObjectNotDetected1')
            HobbitMMUI.AskYesNo(question='ObjectNotDetected1')
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
            # speech_output.sayText(info='T_PU_ObjectNotDetected2')
            speech_output.sayText(info='ObjectNotDetected2')
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
            # speech_output.sayText(info='T_PU_ObjectFoundRepositioning')
            speech_output.sayText(info='ObjectFoundRepositioning')
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
            # speech_output.sayText(info='T_PU_UnableToGraspObject')
            # speech_output.sayText(info='UnableToGraspObject')
            HobbitMMUI.AskYesNo(question='UnableToGraspObject')
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
            # speech_output.sayText(info='T_PU_TryToRemoveObject')
            speech_output.sayText(info='TryToRemoveObject')
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
            # speech_output.sayText(info='T_PU_DidNotPickupObject1')
            speech_output.sayText(info='DidNotPickupObject1')
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
            # speech_output.sayText(info='T_PU_DidNotPickupObject2')
            speech_output.sayText(info='DidNotPickupObject2')
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
            # speech_output.sayText(info='T_PU_PickedUpObject')
            speech_output.sayText(info='PickedUpObject')
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
            # speech_output.sayText(info='T_PU_RemovedObjectTakeObject')
            speech_output.sayText(info='RemovedObjectTakeObject')
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
            hobbit_move.goToPose(),
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
            Sequence.add(
                'LOCATE_USER',
                Dummy()
            )
        Sequence.add(
            'EMO_SAY_REMOVE_OBJECT',
            sayRemoveObjectTakeObject()
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
                'MOVE_ARM_TO_PRE_GRASP_POSITION',
                arm_move.goToPreGraspPosition()
            )
        Sequence.add(
            'GRASP_OBJECT',
            DavidPickingUp()
        )
        return seq

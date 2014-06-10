#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'pickup_import'

import roslib
roslib.load_manifest(PKG)
# import rospy

from smach import Concurrence, Sequence # , State
from hobbit_user_interaction import HobbitEmotions  # ,HobbitMMUI
import hobbit_smach.speech_output_import as speech_output
# import hobbit_smach.head_move_import as head_move
import hobbit_smach.hobbit_move_import as hobbit_move


def child_term_cb(outcome_map):
    return True


def sayPointingGestureNotDetected():
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
            speech_output.sayText(info='PointingGestureNotDetected1')
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
            # speech_output.sayText(info='T_PU_ObjectNotDetected1')
            speech_output.sayText(info='ObjectNotDetected1')
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
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_map={'succeeded': {'EMO_SAD': 'succeeded',
                                   'SAY_UNABLE_TO_GRASP_OBJECT': 'succeeded'}}
    )

    with cc:
        Concurrence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Concurrence.add(
            'SAY_UNABLE_TO_GRASP_OBJECT',
            # speech_output.sayText(info='T_PU_UnableToGraspObject')
            speech_output.sayText(info='UnableToGraspObject')
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

    with seq:
        Sequence.add(
        'EMO_SAY_START_LOOKING',
        sayStartLooking()
        )
        Sequence.add(
            'CALCULATE_LOOKING_POSE',
            DummyLookingPose()
        )
        Sequence.add(
            'MOVE_TO_POSE',
            hobbit_move.goToPose()
        )

    return seq

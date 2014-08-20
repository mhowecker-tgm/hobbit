#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'service_disable'

import roslib
roslib.load_manifest(PKG)
# import rospy

from smach import Sequence
from smach_ros import SimpleActionState
from std_msgs.srv import Empty


def enable_gestures():

    """
    Returns a Sequence that will resume the following services.
    - face detection
    - hand gesture recognition
    - skeleton (person) detection
    """

    seq = Sequence(
        ['succeeded', 'aborted', 'preempted']
    )
    with seq:
        Sequence.add(
            'ENABLE_FACE_DETECTION',
            SimpleActionState(
                '/face_detection/resume',
                Empty
            )
        )
        Sequence.add(
            'ENABLE_GESTURES',
            SimpleActionState(
                '/hand_gestures/resume',
                Empty
            )
        )
        Sequence.add(
            'ENABLE_SKELETON_DETECTION',
            SimpleActionState(
                '/skeleton_detector/resume',
                Empty
            )
        )
    return seq


def disable_gestures():

    """
    Returns a Sequence that will disable the following services.
    - face detection
    - hand gesture recognition
    - skeleton (person) detection
    """

    seq = Sequence(
        ['succeeded', 'aborted', 'preempted']
    )
    with seq:
        Sequence.add(
            'ENABLE_FACE_DETECTION',
            SimpleActionState(
                '/face_detection/pause',
                Empty
            )
        )
        Sequence.add(
            'ENABLE_GESTURES',
            SimpleActionState(
                '/hand_gestures/pause',
                Empty
            )
        )
        Sequence.add(
            'ENABLE_SKELETON_DETECTION',
            SimpleActionState(
                '/skeleton_detector/pause',
                Empty
            )
        )
    return seq

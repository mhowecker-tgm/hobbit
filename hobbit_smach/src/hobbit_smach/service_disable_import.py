#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'service_disable'

import roslib
roslib.load_manifest(PKG)
# import rospy

from smach import Sequence
from smach_ros import ServiceState
from std_srvs.srv import Empty


def enable_gestures():

    """
    Returns a Sequence that will resume the following services.
    - face detection
    - hand gesture recognition
    - skeleton (person) detection
    """

    seq = Sequence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        connector_outcome='succeeded'

    )
    with seq:
        Sequence.add(
            'ENABLE_FACE_DETECTION',
            ServiceState(
                '/face_detection/resume',
                Empty
            )
        )
        Sequence.add(
            'ENABLE_GESTURES',
            ServiceState(
                '/hand_gestures/resume',
                Empty
            )
        )
        Sequence.add(
            'ENABLE_SKELETON_DETECTION',
            ServiceState(
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
        outcomes=['succeeded', 'aborted', 'preempted'],
        connector_outcome='succeeded'

    )
    with seq:
        Sequence.add(
            'ENABLE_FACE_DETECTION',
            ServiceState(
                '/face_detection/pause',
                Empty
            )
        )
        Sequence.add(
            'ENABLE_GESTURES',
            ServiceState(
                '/hand_gestures/pause',
                Empty
            )
        )
        Sequence.add(
            'ENABLE_SKELETON_DETECTION',
            ServiceState(
                '/skeleton_detector/pause',
                Empty
            )
        )
    return seq

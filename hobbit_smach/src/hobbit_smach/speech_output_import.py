#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'SpeechOutput'
DEBUG = True

import roslib
roslib.load_manifest(PKG)

from smach import Sequence
from hobbit_user_interaction import HobbitMMUI
from hobbit_msgs.msg import Event


def sayText(info='Text is missing'):
    """
    Return a SMACH Sequence for speech output on the MMUI.
    The second State is needed to wait until the spoken text
    is completely done. Otherwise the next State can disturb
    the speech output.

    info: defaults to 'Text is missing' which indicates that
    no textID was specified.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded'
    )

    with seq:
            Sequence.add(
                'TALK',
                HobbitMMUI.ShowInfo(info=info))
            Sequence.add(
                'WAIT_FOR_MMUI',
                HobbitMMUI.WaitforSoundEnd('/Event', Event),
                transitions={'aborted': 'WAIT_FOR_MMUI'})
    return seq

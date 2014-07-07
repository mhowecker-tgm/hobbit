#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'SpeechOutput'
DEBUG = True

import roslib
roslib.load_manifest(PKG)

from smach import Sequence, State
from hobbit_user_interaction import HobbitMMUI
from hobbit_msgs.msg import Event
from hobbit_msgs import MMUIInterface as MMUI


def askYesNo(question='Text is missing'):
    """
    Return a SMACH Sequence for speech output on the MMUI.
    We wait for an answer, Yes or No, from the MMUI.
    The second State is needed to wait until the spoken text
    is completely done. Otherwise the next State can disturb
    the speech output.

    info: defaults to 'Text is missing' which indicates that
    no textID was specified.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
            Sequence.add(
                'TALK',
                HobbitMMUI.AskYesNo(question=question)
            )
            Sequence.add(
                'WAIT_FOR_MMUI',
                HobbitMMUI.WaitforSoundEnd('/Event', Event),
                transitions={'aborted': 'WAIT_FOR_MMUI',
                             'succeeded': 'succeeded'})
    return seq


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
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
            Sequence.add(
                'TALK',
                HobbitMMUI.ShowInfo(info=info)
            )
            Sequence.add(
                'WAIT_FOR_MMUI',
                HobbitMMUI.WaitforSoundEnd('/Event', Event),
                transitions={'aborted': 'WAIT_FOR_MMUI',
                             'succeeded': 'succeeded'})
    return seq


class AskForName(State):
    """
    Wrap the askForName function from MMUIInterface in a SMACH state.
    The returned String is stored in the userdata key object_name.
    """

    # def __init__(self, text='T_LO_WHAT_IS_THE_NAME_OF_THIS_OBJECT'):
    def __init__(self, text='WHAT IS THE NAME OF THIS OBJECT'):
        State.__init__(
            self,
            output_keys=['object_name'],
            outcomes=['succeeded', 'failed', 'preempted']
        )
        self.text = text

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.askForName(text=self.text)
        print(resp)
        if not resp:
            return 'failed'
        else:
            for item in resp.params:
                if item.name == 'name':
                    ud.object_name = item.value.lower()
                    return 'succeeded'
        return 'failed'


def playMoveOut():
    """
    Return a SMACH Sequence that will play a sound file and later
    return to the main menu of the MMUI.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'PLAY_SOUND',
            MMUI.ShowInfo(info='Moving out',
                          audio='moveout.mp3')
        )
        Sequence.add(
            'WAIT_FOR_MMUI',
            HobbitMMUI.WaitforSoundEnd('/Event', Event),
            transitions={'aborted': 'WAIT_FOR_MMUI',
                         'succeeded': 'succeeded'}
        )
        Sequence.add(
            'MAIN_MENU',
            HobbitMMUI.ShowMenu(menu='MAIN')
        )
        return seq

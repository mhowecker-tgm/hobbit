#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'SpeechOutput'
DEBUG = False

import rospy
from smach import Sequence, State, Concurrence
from smach_ros import ServiceState
from hobbit_msgs.srv import GetName
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
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


class TestData(State):
    """
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('TEST_DATA_SPEECH: ' + str(ud.robots_room_name))
        return 'succeeded'


def sayTextObject(info='Text is missing', object_name='object'):
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
            HobbitMMUI.ShowInfo(
                info=info,
                object_name=object_name
            )
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
    seq.userdata.robots_room_name = 'roomname'

    with seq:
        if info in ['T_GT_ReachedMyDestination',
                    'T_GT_WayBlocked',
                    'T_GT_GoingToPlace',
                    'T_GT_ConfirmGoToPlace']:
            rospy.loginfo('SPEECH OUTPUT: Sending room name to MMUI.')
            Sequence.add(
                'GET_ROBOTS_CURRENT_ROOM',
                ServiceState(
                    'get_robots_current_room',
                    GetName,
                    response_key='robots_room_name'),
                transitions={'aborted': 'failed'}
            )
            Sequence.add(
                'TEST_DATA_SPEECH',
                TestData()
            )
            Sequence.add(
                'TALK',
                HobbitMMUI.ShowInfo(
                    info=info,
                    place=seq.userdata.robots_room_name
                )
            )
        else:
            Sequence.add(
                'TALK',
                HobbitMMUI.ShowInfo(
                    info=info
                )
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

    def __init__(self, text='T_LO_WHAT_IS_THE_NAME_OF_THIS_OBJECT'):
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
        print(type(self.text))
        print(self.text)
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


class PlayAudio(State):
    """
    Wrap the askForName function from MMUIInterface in a SMACH state.
    The returned String is stored in the userdata key object_name.
    """

    def __init__(self, text='Moving out', audio='move_out.mp3'):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )
        self.text = text
        self.audio = audio

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.showMMUI_Info(text=self.text, audio=self.audio)
        print(resp)
        if not resp:
            return 'failed'
        return 'succeeded'


def playMoveOut():
    """
    Return a SMACH Sequence that will play a sound file and later
    return to the main menu of the MMUI.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'PLAY_SOUND',
            PlayAudio(text='Moving out',
                      audio='moveout.mp3'),
            transitions={'failed': 'aborted'}
        )
        Sequence.add(
            'WAIT_FOR_MMUI',
            HobbitMMUI.WaitforSoundEnd('/Event', Event),
            transitions={'aborted': 'WAIT_FOR_MMUI'}
        )
        Sequence.add(
            'MAIN_MENU',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'failed': 'aborted'}
        )
        return seq


def say_child_term_cb(outcome_map):
    return False


def say_outcome_cb(outcome_map):
    if outcome_map['EMOTION'] == 'succeeded' and\
            outcome_map['SAY_SOMETHING'] == 'succeeded':
        return 'succeeded'
    else:
        return 'aborted'


def emo_say_something(emo='NEUTRAL', time=1, text='textID missing'):
    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        default_outcome='succeeded',
        child_termination_cb=say_child_term_cb,
        outcome_cb=say_outcome_cb
    )

    with cc:
        Concurrence.add(
            'EMOTION',
            HobbitEmotions.ShowEmotions(
                emotion=emo,
                emo_time=time)
        )
        Concurrence.add(
            'SAY_SOMETHING',
            sayText(info=text)
        )
    return cc


def yes_no_child_term_cb(outcome_map):
    return False


def yes_no_outcome_cb(outcome_map):
    if outcome_map['EMOTION'] == 'succeeded' and\
            outcome_map['SAY_SOMETHING'] == 'yes':
        return 'yes'
    else:
        return 'no'


def emo_yes_no_something(emo='NEUTRAL', time=1, text='textID missing'):
    cc = Concurrence(
        outcomes=['yes', 'preempted', 'no'],
        default_outcome='no',
        child_termination_cb=say_child_term_cb,
        outcome_cb=say_outcome_cb
    )

    with cc:
        Concurrence.add(
            'EMOTION',
            HobbitEmotions.ShowEmotions(
                emotion=emo,
                emo_time=time)
        )
        Concurrence.add(
            'SAY_SOMETHING',
            HobbitMMUI.AskYesNo(
                question=text)
        )
    return cc

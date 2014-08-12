#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'SpeechOutput'
DEBUG = False

import roslib
roslib.load_manifest(PKG)

from smach import Sequence, State
from hobbit_user_interaction import HobbitMMUI
from hobbit_msgs import MMUIInterface as MMUI
import hobbit_smach.speech_output_import as speech_output


def offer(rof='pickup'):
    """
    Return a SMACH Sequence to 'Return a Favour'

    rof: defaults to 'None' which indicates that
    no scenario was specified. No ROF will be done.
    """

    favours = {'pickup': 'T_PU_OfferReturnOfFavour',
               'callhobbit': 'T_CA_OfferReturnOfFavour',
               'learnobject': 'T_LO_OfferReturnOfFavour',
               'bringobject': 'T_BM_PLEASE_LET_ME_RETURN_THE_FAVOUR'}

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='yes'
    )

    with seq:
            Sequence.add(
                'OFFER_ROF',
                HobbitMMUI.AskYesNo(question=favours[rof]),
                transitions={'no': 'failed',
                             'timeout': 'OFFER_ROF',
                             '3times': 'failed'}
            )
            Sequence.add(
                'EXECUTE_ROF',
                Games()
            )
    return seq


class Music(State):
    """
    Play music on the MMUI. Wrapped in a SMACH state.
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
        mmui = MMUI.MMUIInterface()
        resp = mmui.GoToMenu('M_AUDIO')
        print(resp)
        return 'succeeded'
        return 'failed'


class Games(State):
    """
    Switch to the games menu on the MMUI. Wrapped in a SMACH state.
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
        mmui = MMUI.MMUIInterface()
        resp = mmui.GoToMenu('M_GAMES')
        print(resp)
        return 'succeeded'
        return 'failed'


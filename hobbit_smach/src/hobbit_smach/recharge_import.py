#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'recharge_import'
DEBUG = False

import roslib
roslib.load_manifest(PKG)
import rospy

from smach import Sequence, State, StateMachine, Concurrence
from uashh_smach.util import SleepState, WaitForMsgState
# from uashh_smach.platform.move_base import HasMovedState
from mira_msgs.msg import BatteryState
import hobbit_smach.hobbit_move_import as hobbit_move
from hobbit_user_interaction import HobbitMMUI
import hobbit_smach.speech_output_import as speech_output


def battery_cb(msg, ud):
    print('Received battery_state message')
    print(msg.charging)
    rospy.sleep(2.0)
    if msg.charging:
        print('I am charging')
        return True
    else:
        print('I am NOT charging')
        return False


class PreemptChecker(State):
    """
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )

    def execute(self, ud):
        rospy.sleep(1.0)
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'aborted'


def getRecharge():
    """This function handles the autonomous charging sequence.
    It is without the user interaction and is mainly used during
    the night or as part of the recharging scenario.
    """

    seq = Sequence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        connector_outcome='succeeded',
    )
    seq.userdata.room_name = 'dock'
    seq.userdata.location_name = 'dock'

    with seq:
        if not DEBUG:
            Sequence.add(
                'SET_NAV_GOAL',
                hobbit_move.SetNavGoal(room='dock', place='dock')
            )
            Sequence.add(
                'MOVE_TO_DOCK',
                hobbit_move.goToPose())
            Sequence.add(
                'DOCKING',
                startDockProcedure())
        else:
            pass
        Sequence.add('MMUI_MAIN_MENU', HobbitMMUI.ShowMenu(menu='MAIN'),
                     transitions={'failed': 'aborted'})
    return seq


def getEndRecharge():
    """This function handles the autonomous charging sequence.
    It is without the user interaction and is mainly used during
    the night or as part of the recharging scenario.
    """

    seq = Sequence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['room_name', 'location_name'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'UNDOCK',
            hobbit_move.Undock()
        )
        #  Sequence.add(
        #      'SOUND',
        #      speech_output.playMoveOut()
        #  )
        Sequence.add(
            'WAIT_FOR_MIRA',
            SleepState(duration=30)
        )
        Sequence.add(
            'SET_NAV_GOAL_DOCK',
            hobbit_move.SetNavGoal(room='dock', place='dock')
        )
        Sequence.add(
            'MOVE_AWAY_FROM_DOCK',
            hobbit_move.goToPose())
        Sequence.add(
            'SET_NAV_GOAL_RANDOM',
            hobbit_move.SetNavGoal(room='maincorridor', place='default')
        )
        Sequence.add(
            'MOVE_AWAY',
            hobbit_move.goToPose())
    return seq


def startDockProcedure():
    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded'
    )

    seq1 = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded'
    )

    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'])

    with seq:
        Sequence.add(
            'CHARGE_CHECK',
            WaitForMsgState('/battery_state', BatteryState, msg_cb=battery_cb),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'PREEMPT',
                         'preempted': 'preempted'})
        Sequence.add(
            'PREEMPT',
            PreemptChecker(),
            transitions={'preempted': 'preempted',
                         'aborted': 'CHARGE_CHECK'})

    def child_term_cb(outcome_map):
            return True

    def out_cb(outcome_map):
        print('CHECK: outcome_map')
        print(outcome_map)
        if outcome_map['CHARGE_CHECK'] == 'succeeded':
            return 'succeeded'
        elif outcome_map['WAIT'] == 'succeeded':
            return 'failed'

    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_cb=out_cb
    )

    with cc:
        Concurrence.add(
            'WAIT', SleepState(duration=25))
        Concurrence.add(
            'CHARGE_CHECK',
            seq)

    with seq1:
        Sequence.add('START_DOCK', hobbit_move.Dock())
        Sequence.add('WAIT', SleepState(duration=30))
        Sequence.add('CHECK', cc,
                         transitions={'succeeded': 'aborted',
                                      'failed': 'RETRY'})
        Sequence.add('RETRY', hobbit_move.Undock())
        Sequence.add('WAIT1', SleepState(duration=10))
        Sequence.add('CHECK_1', cc,
                         transitions={'succeeded': 'aborted',
                                      'failed': 'START_DOCK'})

    with sm:
        StateMachine.add('DOCKING_PROCEDURE', seq1,
                         transitions={'succeeded': 'succeeded',
                                      'aborted': 'STOP'})
        StateMachine.add('STOP', hobbit_move.Stop(),
                         transitions={'succeeded': 'aborted'})
    return sm

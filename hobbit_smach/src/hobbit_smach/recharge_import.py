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
from uashh_smach.platform.move_base import HasMovedState
from std_msgs.msg import String
from mira_msgs.msg import BatteryState
import hobbit_smach.hobbit_move_import as hobbit_move
from hobbit_user_interaction import HobbitMMUI
# import hobbit_smach.speech_output_import as speech_output


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
                startDockProcedure2(),
                transitions={'aborted': 'SET_NAV_GOAL'})
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
        # Sequence.add(
        #     'WAIT_FOR_MIRA',
        #     SleepState(duration=5)
        # )
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

    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'])

    with seq:
        Sequence.add(
            'CHARGE_CHECK',
            WaitForMsgState('/battery_state', BatteryState, msg_cb=battery_cb),
            transitions={'aborted': 'PREEMPT',
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

    with sm:
        StateMachine.add('START_DOCK', hobbit_move.Dock(),
                         transitions={'succeeded': 'WAIT'})
        StateMachine.add('WAIT', SleepState(duration=20),
                         transitions={'succeeded': 'CHECK'})
        # StateMachine.add('DID_WE_MOVE',
        #                  hobbit_move.HasMovedFromPreDock(minimum_distance=0.5),
        #                  transitions={'movement_exceeds_distance': 'CHECK',
        #                               'movement_within_distance': 'DID_WE_MOVE',
        #                               'counter': 'CHECK'})
        StateMachine.add('CHECK', cc,
                         transitions={'succeeded': 'STOP',
                                      'failed': 'RETRY'})
        StateMachine.add('RETRY', hobbit_move.Undock(),
                         transitions={'succeeded': 'WAIT1'})
        StateMachine.add('WAIT1', SleepState(duration=10),
                         transitions={'succeeded': 'START_DOCK'})
        StateMachine.add('STOP', hobbit_move.Stop(),
                         transitions={'succeeded': 'succeeded'})
    return sm


def startDockProcedure2():
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'])

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'CHARGE_CHECK',
            WaitForMsgState('/battery_state', BatteryState, msg_cb=battery_cb),
            transitions={'aborted': 'PREEMPT',
                         'preempted': 'preempted'})
        Sequence.add(
            'PREEMPT',
            PreemptChecker(),
            transitions={'preempted': 'preempted',
                         'aborted': 'CHARGE_CHECK'})

    with sm:
        StateMachine.add(
            'SET_POSE',
            HasMovedState(distance=0.2),
            transitions={'movement_exceeds_distance': 'MIRA_DOCK',
                         'movement_within_distance': 'MIRA_DOCK'}
        )
        StateMachine.add(
            'MIRA_DOCK',
            hobbit_move.Dock(),
            transitions={'succeeded': 'DID_WE_MOVE',
                         'aborted': 'DID_WE_MOVE'}
        )
        StateMachine.add(
            'DID_WE_MOVE',
            HasMovedState(distance=0.2),
            transitions={'movement_exceeds_distance': 'WAIT_20',
                         'movement_within_distance': 'COUNT'}
        )
        StateMachine.add(
            'WAIT_20',
            SleepState(durarion=20),
            transitions={'succeeded': 'CHARGE_CHECK',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'CHARGE_CHECK',
            seq,
            transitions={'succeeded': 'succeeded',
                         'aborted': 'MIRA_DOCK'}
        )
        StateMachine.add(
            'COUNT',
            Count(),
            transitions={'first': 'CCW',
                         'second': 'CW',
                         'third': 'aborted'}
        )
        StateMachine.add(
            'CCW',
            RotateCCW(),
            transitions={'succeeded': 'SET_POSE',
                         'aborted': 'SET_POSE',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'CW',
            RotateCW(),
            transitions={'succeeded': 'SET_POSE',
                         'aborted': 'SET_POSE',
                         'preempted': 'preempted'}
        )
        return sm


class Count(State):
    """
    """
    def __init__(self, angle=0):
        State.__init__(
            self,
            outcomes=['first', 'second', 'third', 'preempted']
        )
        self.counter = 0

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        self.counter += 1
        if self.counter == 1:
            return 'first'
        elif self.counter == 2:
            return 'second'
        else:
            self.counter = 0
            return 'third'


class RotateCCW(State):
    """
    """
    def __init__(self, angle=0):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted']
        )
        self.motion_pub = rospy.Publisher('DiscreteMotionCmd', String,
                                          latch=False, queue_size=50)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        self.motion_pub.publish('Turn 15')
        return 'succeeded'


class RotateCW(State):
    """
    """
    def __init__(self, angle=0):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted']
        )
        self.motion_pub = rospy.Publisher('DiscreteMotionCmd', String,
                                          latch=False, queue_size=50)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        self.motion_pub.publish('Turn -30')
        return 'succeeded'

#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'call_hobbit'
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
import hobbit_smach.logging_import as log


def battery_cb(msg, ud):
    print('Received battery_state message')
    print(msg.charging)
    #print(ud.params)
    rospy.sleep(2.0)
    if msg.charging:
        print('I am charging')
        return True
    else:
        print('I am NOT charging')
        return False


class ExtractGoal(State):
    """
    """
    def __init__(self):
        State.__init__(
            self,
            input_keys=['params', 'room_name', 'location_name'],
            output_keys=['room_name', 'location_name'],
            outcomes=['succeeded', 'aborted', 'preempted']
        )

    def execute(self, ud):
        rospy.sleep(1.0)
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('NAME: ', ud.params[0].name)
        print('VALUE: ', ud.params[0].value)
        ud.room_name  = ud.params[0].name.lower()
        ud.location_name = ud.params[0].value.lower()
        print(ud.room_name)
        print(ud.location_name)
        return 'succeeded'


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


# def get_end_recharge():
#     """This function handles the autonomous charging sequence.
#     It is without the user interaction and is mainly used during
#     the night or as part of the recharging scenario.
#     """
#
#     seq = Sequence(
#         outcomes=['succeeded', 'aborted', 'preempted'],
#         input_keys=['room_name', 'location_name'],
#         connector_outcome='succeeded'
#     )
#
#     with seq:
#         Sequence.add(
#             'UNDOCK',
#             hobbit_move.Undock()
#         )
#         #  Sequence.add(
#         #      'SOUND',
#         #      speech_output.playMoveOut()
#         #  )
#         Sequence.add(
#             'WAIT_FOR_MIRA',
#             SleepState(duration=5)
#         )
#         Sequence.add(
#             'SET_NAV_GOAL_DOCK',
#             hobbit_move.SetNavGoal(room='dock', place='dock')
#         )
#         Sequence.add(
#             'MOVE_AWAY_FROM_DOCK',
#             hobbit_move.goToPose())
#     return seq
#
#
def call_hobbit():
    """
    First check if Hobbit is charging and in the docking station,
    if so move out. Then go to the user.
    """

    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command', 'params', 'parameters'],
        output_keys=['command', 'params', 'parameters']
    )

    with sm:
        #StateMachine.add(
        #    'CHARGE_CHECK',
        #    WaitForMsgState('/battery_state', BatteryState, msg_cb=battery_cb, input_keys=['params']),
        #    transitions={'succeeded': 'UNDOCK',
        #                 'aborted': 'EXTRACT_GOAL',
        #                 'preempted': 'preempted'}
        #)
        #StateMachine.add(
        #    'UNDOCK',
        #    hobbit_move.get_undock(),
        #    transitions={'succeeded': 'EXTRACT_GOAL',
        #                 'aborted': 'aborted',
        #                 'preempted': 'preempted'}
        #)
        StateMachine.add_auto(
            'EXTRACT_GOAL',
            ExtractGoal(),
            connector_outcomes=['succeeded']
        )
        StateMachine.add_auto(
            'SET_NAV_GOAL',
            hobbit_move.SetNavigationGoal(),
            connector_outcomes=['succeeded']
        )
        StateMachine.add(
            'MOVE_TO_GOAL',
            hobbit_move.goToPose(),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'aborted': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Call Hobbit'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORT',
            log.DoLogPreempt(scenario='Call Hobbit'),
            transitions={'succeeded': 'aborted'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogPreempt(scenario='Call Hobbit'),
            transitions={'succeeded': 'succeeded'}
        )

        return sm

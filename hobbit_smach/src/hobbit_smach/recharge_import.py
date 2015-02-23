#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'recharge_import'
DEBUG = False

import roslib
roslib.load_manifest(PKG)
import rospy

from smach import Sequence, State, StateMachine, Concurrence
from smach_ros import ServiceState
from uashh_smach.util import SleepState, WaitForMsgState
# from uashh_smach.platform.move_base import HasMovedState
from mira_msgs.msg import BatteryState
from hobbit_msgs.srv import ChargeCheck
import hobbit_smach.hobbit_move_import as hobbit_move
from hobbit_user_interaction import HobbitMMUI
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.logging_import as log


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


def undock_if_needed():
    """
    Returns a SMACH StateMachine that check if it is needed to undock
    before any movement is done.
    """
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'])
    with sm:
        StateMachine.add(
            WaitForMsgState(
                '/battery_state',
                BatteryState,
                msg_cb=battery_cb
                ),
            transitions={'succeeded': 'UNDOCK',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'UNDOCK',
            # hobbit_move.get_undock(),
            hobbit_move.get_undock_action(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        return sm


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
        Sequence.add(
            'LOG_RECHARGE',
            log.do_log_battery_state(),
            transitions={'aborted': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'}
        )
        if not DEBUG:
            Sequence.add(
                'SAY_TIRED',
                speech_output.emo_say_something(
                    emo='TIRED',
                    time=0,
                    text='T_CH_MovingToChargingStation'
                ),
                transitions={'aborted': 'LOG_ABORT',
                             'preempted': 'LOG_PREEMPT'}
            )
            Sequence.add(
                'SET_NAV_GOAL',
                hobbit_move.SetNavGoal(room='dock', place='dock'),
                transitions={'preempted': 'LOG_PREEMPT'}
            )
            Sequence.add(
                'MOVE_TO_DOCK',
                hobbit_move.goToPoseSilent(),
                transitions={'aborted': 'LOG_ABORT',
                             'preempted': 'LOG_PREEMPT'})
            Sequence.add(
                'DOCKING',
                startDockProcedure(),
                transitions={'aborted': 'LOG_ABORT',
                             'preempted': 'LOG_PREEMPT'})
        else:
            pass
        Sequence.add('MMUI_MAIN_MENU', HobbitMMUI.ShowMenu(menu='MAIN'),
                     transitions={'failed': 'LOG_ABORT',
                                  'succeeded': 'LOG_SUCCESS',
                                  'preempted': 'LOG_PREEMPT'})
        Sequence.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Recharge'),
            transitions={'succeeded': 'preempted'}
        )
        Sequence.add(
            'LOG_ABORT',
            log.DoLogPreempt(scenario='Recharge'),
            transitions={'succeeded': 'aborted'}
        )
        Sequence.add(
            'LOG_SUCCESS',
            log.DoLogPreempt(scenario='Recharge'),
            transitions={'succeeded': 'succeeded'}
        )
    return seq


def getEndRecharge():
    """
    This function handles the autonomous undocking sequence.
    """

    seq = Sequence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['room_name', 'location_name'],
        connector_outcome='succeeded'
    )

    seq1 = Sequence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['room_name', 'location_name'],
        connector_outcome='succeeded'
    )

    with seq1:
        Sequence.add(
            'UNDOCK',
            hobbit_move.get_undock_action())
        #  Sequence.add(
        #      'SOUND',
        #      speech_output.playMoveOut()
        #  )
    return seq1

    # with seq:
    #     Sequence.add(
    #         'UNDOCK ',
    #         hobbit_move.get_undock()
    #     )
    #     #  Sequence.add(
    #     #      'SOUND',
    #     #      speech_output.playMoveOut()
    #     #  )
    #     Sequence.add(
    #         'WAIT_FOR_MIRA',
    #         SleepState(duration=30)
    #     )
    #     Sequence.add(
    #         'SET_NAV_GOAL_DOCK',
    #         hobbit_move.SetNavGoal(room='dock', place='dock')
    #     )
    #     Sequence.add(
    #         'MOVE_AWAY_FROM_DOCK',
    #         hobbit_move.goToPose())
    # return seq


def startDockProcedureOld():
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

    def child_term_cb(outcome_map):
            return True

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
            'WAIT', SleepState(duration=5))
        Concurrence.add(
            'CHARGE_CHECK',
            seq)


    with seq1:
        Sequence.add('START_DOCK', hobbit_move.get_dock_action())
        # Sequence.add('START_DOCK', hobbit_move.Dock())
        Sequence.add('WAIT', SleepState(duration=10))
        Sequence.add('CHECK',
                     cc,
                     transitions={'succeeded': 'aborted',
                                  'failed': 'RETRY'})
        Sequence.add('RETRY', hobbit_move.get_undock_action())
        # Sequence.add('RETRY', hobbit_move.get_undock())
        Sequence.add('WAIT1', SleepState(duration=10))
        Sequence.add('CHECK_1', cc,
                     transitions={'succeeded': 'aborted',
                                  'failed': 'START_DOCK'})

    with sm:
        StateMachine.add('DOCKING_PROCEDURE', seq1,
                         transitions={'succeeded': 'succeeded',
                                      'aborted': 'aborted'})
        # StateMachine.add('STOP', hobbit_move.Stop(),
        #                 transitions={'succeeded': 'aborted'})
    return sm

class FirstSecondThird(State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['first', 'second', 'third', 'aborted', 'preempted'],
            input_keys=['counter'],
            output_keys=['counter']
        )

    def execute(self, ud):
        if not ud.counter:
            ud.counter = 0
        if ud.counter == 0:
            ud.counter += 1
            return 'first'
        elif ud.counter == 1:
            ud.counter += 1
            return 'second'
        elif ud.counter == 2:
            return 'third'
        else:
            return 'aborted'


def charge_response_cb(userdata, response):
    print(response.response)
    if response.response:
        return 'succeeded'
    else:
        return 'aborted'


def startDockProcedure():
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        StateMachine.add('START_DOCKING',
                         hobbit_move.get_dock_action(),
                         transitions={'succeeded': 'CHARGE_CHECK',
                                      'aborted': 'FIRST_SECOND_CHECK',
                                      'preempted': 'preempted'}
                         )
        StateMachine.add('CHARGE_CHECK',
                         ServiceState('/hobbit/charge_check',
                                      ChargeCheck,
                                      response_cb=charge_response_cb
                         ),
                         transitions={'succeeded': 'succeeded',
                                      'aborted': 'UNDOCK',
                                      'preempted': 'preempted'}
        )
        StateMachine.add('UNDOCK',
                         hobbit_move.get_undock_action(),
                         transitions={'succeeded': 'START_DOCKING',
                                      'aborted': 'START_DOCKING',
                                      'preempted': 'preempted'})
        StateMachine.add('FIRST_SECOND_CHECK',
                         FirstSecondThird(),
                         transitions={'first': 'ROTATE_CCW',
                                      'second': 'ROTATE_CW',
                                      'third': 'aborted',
                                      'preempted': 'preempted'}
        )
        StateMachine.add('ROTATE_CCW',
                         hobbit_move.rotateRobot(angle=-25),
                         transitions={'succeeded': 'START_DOCKING',
                                      'aborted': 'ROTATE_CW',
                                      'preempted': 'preempted'}
                         )
        StateMachine.add('ROTATE_CW',
                         hobbit_move.rotateRobot(angle=50),
                         transitions={'succeeded': 'START_DOCKING',
                                      'aborted': 'aborted',
                                      'preempted': 'preempted'}
                         )
    return sm

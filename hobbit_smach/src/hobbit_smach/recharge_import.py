#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'recharge_import'
DEBUG = False

import roslib
roslib.load_manifest(PKG)
import rospy

from smach import Sequence, State, StateMachine
from uashh_smach.util import SleepState, WaitForMsgState
from mira_msgs.msg import BatteryState
import hobbit_smach.hobbit_move_import as hobbit_move
from hobbit_user_interaction import HobbitMMUI
from uashh_smach.util import SleepState


def battery_cb(msg, ud):
    print('Received battery_state message')
    print(msg.charging)
    if msg.charging == True:
        print('I am charging')
        return True
    else:
        print('I am NOT charging')
        return False


class Dummy(State):
    """Dummy SMACH State class that sleeps for 2 seconds and returns succeeded
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded'],
            output_keys=['room_name', 'location_name']
        )

    def execute(self, ud):
        rospy.sleep(2.0)
        ud.room_name = 'maincorridor'
        ud.location_name = 'default'
        return 'succeeded'


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
                'MOVE_TO_DOCK',
                hobbit_move.goToPosition(frame='/map', place='dock'))
            Sequence.add(
                'DOCKING',
                # hobbit_move.Dock())
                startDockProcedure())
        else:
            pass
        #    Sequence.add('MOVE_TO_DOCK', Dummy())
        #    Sequence.add('DOCKING', Dummy())
        # TODO: Add check to make sure we are charging
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
    # seq.userdata.room_name = 'maincorridor'
    # seq.userdata.location_name = 'default'

    with seq:
        Sequence.add(
            'UNDOCK',
            hobbit_move.Undock()
        )
        Sequence.add(
            'WAIT_FOR_MIRA',
            SleepState(duration=5)
        )
        Sequence.add(
            'MOVE_AWAY_FROM_DOCK',
            hobbit_move.goToPosition(frame='/map', room='dock', place='dock'))
        # Sequence.add(
        #     'WAIT_FOR_MIRA_2',
        #     SleepState(duration=3)
        # )
        # Sequence.add(
        #     'SET_SOME_GOAL',
        #     Dummy())
        Sequence.add(
            'MOVE_AWAY',
            hobbit_move.goToPosition(frame='/map', room='maincorridor',
                                     place='default'))
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
        )

    with sm:
        StateMachine.add('START_DOCK', hobbit_move.Dock(),
                         transitions={'succeeded': 'WAIT'})
        StateMachine.add('WAIT', SleepState(duration=30),
                         transitions={'succeeded': 'CHECK'})
        StateMachine.add('CHECK', seq,
                         transitions={'succeeded': 'STOP',
                                      'aborted': 'RETRY'})
        StateMachine.add('RETRY', hobbit_move.Undock(),
                         transitions={'succeeded': 'GOTO_DOCK'})
        StateMachine.add('GOTO_DOCK', hobbit_move.Undock(),
                         transitions={'succeeded': 'WAIT1'})
        StateMachine.add('WAIT1', SleepState(duration=10),
                         transitions={'succeeded': 'START_DOCK'})
        StateMachine.add('STOP', hobbit_move.Stop(),
                         transitions={'succeeded': 'succeeded'})
        
        
    return sm

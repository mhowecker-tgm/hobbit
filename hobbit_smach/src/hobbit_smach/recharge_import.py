#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'recharge_import'
<<<<<<< HEAD
<<<<<<< HEAD
DEBUG = False
=======
DEBUG = True
>>>>>>> master handles time now
=======
DEBUG = False
>>>>>>> small changes to test dock/undock at the charging station.

import roslib
roslib.load_manifest(PKG)
import rospy

from smach import Sequence, State
import hobbit_smach.hobbit_move_import as hobbit_move
from hobbit_user_interaction import HobbitMMUI
<<<<<<< HEAD
<<<<<<< HEAD
from uashh_smach.util import SleepState
=======
>>>>>>> master handles time now
=======
from uashh.util import SleepState
>>>>>>> small changes to test dock/undock at the charging station.


class Dummy(State):
    """Dummy SMACH State class that sleeps for 2 seconds and returns succeeded
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded'],
<<<<<<< HEAD
            output_keys=['room_name', 'location_name']
=======
>>>>>>> master handles time now
        )

    def execute(self, ud):
        rospy.sleep(2.0)
<<<<<<< HEAD
        ud.room_name = 'maincorridor'
        ud.location_name = 'default'
=======
>>>>>>> master handles time now
        return 'succeeded'


def getRecharge():
    """This function handles the autonomous charging sequence.
    It is without the user interaction and is mainly used during
    the night or as part of the recharging scenario.
    """

    seq = Sequence(
<<<<<<< HEAD
        outcomes=['succeeded', 'aborted', 'preempted'],
        connector_outcome='succeeded',
    )
    seq.userdata.room_name = 'dock'
    seq.userdata.location_name = 'dock'
=======
        outcomes=['succeeded', 'failed', 'preempted'],
        connector_outcome='succeeded'
    )
>>>>>>> master handles time now

    with seq:
        if not DEBUG:
            Sequence.add(
                'MOVE_TO_DOCK',
<<<<<<< HEAD
                hobbit_move.goToPosition(frame='/map'))
            Sequence.add(
                'DOCKING',
                hobbit_move.Dock())
            Sequence.add(
                'WAIT_FOR_MIRA',
                SleepState(duration=10)
            )
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
        connector_outcome='succeeded'
    )
    seq.userdata.room_name = 'maincorridor'
    seq.userdata.location_name = 'default'

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
            hobbit_move.goToPosition(frame='/map'))
        #Sequence.add(
        #    'WAIT_FOR_MIRA_2',
        #    SleepState(duration=3)
        #)
        #Sequence.add(
        #    'SET_SOME_GOAL',
        #    Dummy())
        #Sequence.add(
        #    'MOVE_AWAY',
        #    hobbit_move.goToPosition(frame='/map'))
    return seq

=======
                hobbit_move.goToPosition(frame='/map', place='dock'))
            Sequence.add(
                'DOCKING',
                hobbit_move.Dock())
        else:
            Sequence.add('MOVE_TO_DOCK', Dummy())
            Sequence.add('DOCKING', Dummy())
        Sequence.add('MMUI_MAIN_MENU', HobbitMMUI.ShowMenu(menu='MAIN'))
    return seq
<<<<<<< HEAD
>>>>>>> master handles time now
=======

def getRecharge():
    """This function handles the autonomous charging sequence.
    It is without the user interaction and is mainly used during
    the night or as part of the recharging scenario.
    """

    seq = Sequence(
        outcomes=['succeeded', 'failed', 'preempted'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'UNDOCK',
            hobbit_move.Undock()
        )
        Sequence.add(
            'WAIT_FOR_MIRA',
            SleepState(duration=3)
        )

>>>>>>> small changes to test dock/undock at the charging station.

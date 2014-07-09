#!/usr/bin/env python

import rospy
from smach import State, Sequence
from datetime import datetime, time
from hobbit_user_interaction import HobbitMMUI
import hobbit_smach.hobbit_move_import as hobbit_move


class TimeCheck(State):
    """
    Class to check if we were activated during the night.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['day', 'night', 'canceled'],
            input_keys=['night'],
            output_keys=['night'])
        if rospy.has_param('/sleep_time'):
            self.sleep_time = rospy.get_param('/sleep_time')
        else:
            self.sleep_time = '22:00'
        if rospy.has_param('/wakeup_time'):
            self.wakeup_time = rospy.get_param('/wakeup_time')
        else:
            self.wakeup_time = '06:30'

    def execute(self, ud):
        wake = self.wakeup_time.split(':')
        sleep = self.sleep_time.split(':')
        now = datetime.now()
        if time(int(wake[0]), int(wake[1]))\
                <= now.time()\
                <= time(int(sleep[0]), int(sleep[1])):
            print('yes, within the interval')
            return 'day'
        else:
            return 'night'


def IsItNight():
    if rospy.has_param('/sleep_time'):
        sleep_time = rospy.get_param('/sleep_time')
    else:
        sleep_time = '22:00'
    if rospy.has_param('/wakeup_time'):
        wakeup_time = rospy.get_param('/wakeup_time')
    else:
        wakeup_time = '06:30'

    wake = wakeup_time.split(':')
    sleep = sleep_time.split(':')
    now = datetime.now()
    if time(int(wake[0]), int(wake[1]))\
            <= now.time()\
            <= time(int(sleep[0]), int(sleep[1])):
        return False
    else:
        return True


def get_hobbit_full_stop():
    """
    Return a SMACH sequence that will stop all movement of Hobbit and
    returns to the main menu of the MMUI.
    """
    seq = Sequence(
        outcomes=['succeeded', 'failed', 'preempted'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'STOP_MOVEMENT',
            hobbit_move.Stop()
        )
        Sequence.add(
            'MAIN_MENU',
            HobbitMMUI.ShowMenu(menu='MAIN')
        )
    return seq

#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'hobbit_move'
DEBUG = False

import roslib
roslib.load_manifest(PKG)

from smach import State
from crontab import CronTab

cron = CronTab(user=True)
tasks = {'wakeup': {'name': 'cron_GETUP.py',
                    'hour': latest_getup.hour,
                    'minutes': latest_getup.minutes},
         'clearfloor': {'name': 'cron_GETUP.py',
                    'hour': latest_getup.hour,
                    'minutes': latest_getup.minutes},
         'safetycheck': {'name': 'cron_GETUP.py',
                    'hour': latest_getup.hour,
                    'minutes': latest_getup.minutes},
         'patrol': {'name': 'cron_GETUP.py',
                    'hour': latest_getup.hour,
                    'minutes': latest_getup.minutes},
         'socialrole': {'name': 'cron_GETUP.py',
                    'hour': latest_getup.hour,
                    'minutes': latest_getup.minutes}}


class SetCronJob(State):
    """
    Set a cronjob as the user demo.
    """
    def __init__(self, ):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'succeeded'


class RemoveCronJob(State):
    """
    Remove a cronjob from the users crontab
    """
    def __init__(self, angle=0):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'succeeded'

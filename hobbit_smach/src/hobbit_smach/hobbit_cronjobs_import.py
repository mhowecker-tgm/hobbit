#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'hobbit_move'
DEBUG = False
PATH = '/opt/ros/hobbit_hydro/src/hobbit_smach/scripts/'

from datetime import datetime, time
import rospy
from smach import State
from crontab import CronTab

cron = CronTab(user=True)
tasks = {'wakeup': {'name': 'cron_GETUP.sh',
                    'hour': latest_getup.hour,
                    'minutes': latest_getup.minutes},
         'clearfloor': {'name': 'cron_CF.sh',
                        'hour': latest_getup.hour,
                        'minutes': latest_getup.minutes},
         'safetycheck': {'name': 'cron_SF.sh',
                        'hour': latest_getup.hour,
                        'minutes': latest_getup.minutes},
         'patrol': {'name': 'cron_P.sh',
                    'hour': latest_getup.hour,
                    'minutes': latest_getup.minutes},
         'socialrole': {'name': 'cron_SR.sh',
                        'hour': latest_getup.hour,
                        'minutes': latest_getup.minutes}}


class SetCronJob(State):
    """
    Set a cronjob as the user demo.
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
        return 'succeeded'


class RemoveCronJob(State):
    """
    Remove a cronjob from the users crontab
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'succeeded'


class ThreeHours(State):
    """
    Remove the current cronjob for patrol from the crontab,
    calculate the next time it should run and set the job
    to the new time.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        next_time = get_next_time()
        cron.remove_all(comment='3hours')
        if check_sleep_times(next_time):
            job = cron.new(
                command=PATH+'cron_P.sh',
                comment='3hours'
            )
            job.hour.on(next_time[0])
            job.minute.on(next_time[1])
        else:
            return 'aborted'
        return 'succeeded'


def get_next_time():
    now = datetime.now()
    hour = now.hour + 3
    minute = now.minute
    return (hour, minute)


def check_sleep_times(next_time):
    if rospy.has_param('/sleep_time'):
        sleep_time = rospy.get_param('sleep_time')
    else:
        sleep_time = '22:00'
    if rospy.has_param('wakeup_time'):
        wakeup_time = rospy.get_param('wakeup_time')
    else:
        wakeup_time = '06:30'
    sleep = sleep_time.split(':')
    wakeup = wakeup_time.split(':')
    if time(int(wakeup[0]), int(wakeup[1]))\
            <= next_time\
            <= time(sleep[0], sleep[1]):
        return True
    else:
        False

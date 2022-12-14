#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'hobbit_move'
DEBUG = False
PATH = '. $HOME/.profile; /opt/ros/hobbit_hydro/src/hobbit_smach/scripts/'

from datetime import datetime, time
import rospy
from smach import State
from crontab import CronTab

cron = CronTab(user=True)
# tasks = {'wakeup': {'name': 'cron_GETUP.sh',
#                     'hour': latest_getup.hour,
#                     'minutes': latest_getup.minutes},
#          'clearfloor': {'name': 'cron_CF.sh',
#                         'hour': latest_getup.hour,
#                         'minutes': latest_getup.minutes},
#          'safetycheck': {'name': 'cron_SF.sh',
#                         'hour': latest_getup.hour,
#                         'minutes': latest_getup.minutes},
#          'patrol': {'name': 'cron_P.sh',
#                     'hour': latest_getup.hour,
#                     'minutes': latest_getup.minutes},
#          'socialrole': {'name': 'cron_SR.sh',
#                         'hour': latest_getup.hour,
#                         'minutes': latest_getup.minutes}}


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


class SleepAway(State):
    """
    Remove the current cronjob for patrol from the crontab,
    calculate the next time it should run and set the job
    to the new time.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['interval']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        next_time = get_next_time(interval=ud.interval)
        rospy.loginfo("Remove old cronjobs")
        cron.remove_all(comment='3hours')
        cron.remove_all(comment='sleep_away')
        cron.write()
        rospy.loginfo(str(cron.render()))
        if check_sleep_times(next_time):
            rospy.loginfo("Setting cronjob.")
            job = cron.new(
                command=PATH+'cron_P.sh',
                comment='sleep_away'
            )
            job.hour.on(next_time[0])
            job.minute.on(next_time[1])
            rospy.loginfo("setting cronjob for patrol to: "+str(next_time[0])+":"+str(next_time[1]))
            cron.write()
            rospy.loginfo(str(cron.render()))
        else:
            return 'aborted'
        return 'succeeded'


def check_sleep_times(next_time):
    if rospy.has_param('/Hobbit/sleep_time'):
        sleep_time = rospy.get_param('/Hobbit/sleep_time')
    else:
        sleep_time = '21:00'
    if rospy.has_param('/Hobbit/wakeup_time'):
        wakeup_time = rospy.get_param('/Hobbit/wakeup_time')
    else:
        wakeup_time = '09:00'
    if type(wakeup_time) is int or ':' not in wakeup_time:
         wakeup_time = '09:00'
    if type(sleep_time) is int or ':' not in sleep_time:
         sleep_time = '21:00'
    try:
        sleep = sleep_time.split(':')
        wakeup = wakeup_time.split(':')
    except Exception:
        rospy.loginfo("Exception thrown while trying to use sleep/wakeup time. Using default values.")
        wakeup = [9, 00]
        sleep = [21, 00]
    if next_time[0] >= 24:
        rospy.loginfo("cronjob not set. Reason: after midnight")
	return False
    elif time(int(wakeup[0]), int(wakeup[1]))\
            <= time(next_time[0], next_time[1])\
            <= time(int(sleep[0]), int(sleep[1])):
        return True
    else:
        rospy.loginfo("cronjob not set. Reason: no within time limits")
        False


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
        next_time = get_next_time(interval=3)
        cron.remove_all(comment='3hours')
        cron.remove_all(comment='sleep_away')
        if check_sleep_times(next_time):
            job = cron.new(
                command=PATH+'cron_P.sh',
                comment='3hours'
            )
            job.hour.on(next_time[0])
            job.minute.on(next_time[1])
            rospy.loginfo("setting cronjob for patrol to: "+str(next_time[0])+":"+str(next_time[1]))
            cron.write()
            rospy.loginfo(str(cron.render()))
        else:
            return 'aborted'
        return 'succeeded'


def get_next_time(interval):
    #FIXME: Do not use hardcoded times
    if int(interval) == 12:
        return (22, 00)
    elif int(interval == 24):
        return (9, 00)
    now = datetime.now()
    hour = now.hour + interval
    minute = now.minute
    return (hour, minute)

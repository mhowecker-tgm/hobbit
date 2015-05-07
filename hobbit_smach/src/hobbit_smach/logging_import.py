#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from smach import State, Sequence
from hobbit_msgs.msg import Event, Parameter
from uashh_smach.util import WaitForMsgState
from mira_msgs.msg import BatteryState
from std_msgs.msg import String


class DoLog(State):

    """
    The logging to the Event topic can be done from here
    """

    def __init__(self, scenario=None, data=None):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted'],
            input_keys=['scenario', 'data', 'command'] if
            (scenario is None and data is None) else ['command']
        )
        self.scenario = scenario
        self.data = data
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)

    def execute(self, ud):               
        scenario = ud.scenario if self.scenario is not None else ud.command
        data = ud.data if self.data is None else self.data
        if scenario == 'IDLE':
            return 'aborted'
        rospy.loginfo('LOG: scenario: %s: %s' % (scenario, data))
        message = Event()
        message.event = 'E_LOG'
        params = []
        par = Parameter(name='SCENARIO',
                        value=scenario)
        params.append(par)
        par = Parameter(name='DATA',
                        value=data)
        params.append(par)
        message.params = params
        self.pubEvent.publish(message)
        return 'succeeded'


class DoLogPreempt(State):
    """
    Inherit from DoLog but specialize in logging of preemption
    """

    def __init__(self, scenario=None):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['scenario', 'data', 'command'] if
            (scenario is not None) else []
        )
        self.scenario = scenario
        self.data = 'Task has preempted.'
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)

    def execute(self, ud):
        rospy.loginfo('LOG: scenario: %s: %s' % (self.scenario, self.data))
        message = Event()
        message.event = 'E_LOG'
        params = []
        par = Parameter(name='SCENARIO',
                        value=self.scenario)
        params.append(par)
        par = Parameter(name='DATA',
                        value=self.data)
        params.append(par)
        message.params = params
        self.pubEvent.publish(message)
        return 'succeeded'

class DoLogSuccess(State):
    """
    Inherit from DoLog but specialize in logging of preemption
    """

    def __init__(self, scenario=None):
        State.__init__(
            self,
            outcomes=['succeeded'],
            output_keys=['result'],
            input_keys=['scenario', 'data', 'command'] if
            (scenario is not None) else []
        )
        self.scenario = scenario
        self.data = 'Task has succeeded.'
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)

    def execute(self, ud):
        rospy.loginfo('LOG: scenario: %s: %s' % (self.scenario, self.data))
        message = Event()
        message.event = 'E_LOG'
        params = []
        par = Parameter(name='SCENARIO',
                        value=self.scenario)
        params.append(par)
        par = Parameter(name='DATA',
                        value=self.data)
        params.append(par)
        message.params = params
        self.pubEvent.publish(message)
        ud.result = String('success')
        return 'succeeded'


class DoLogAborted(State):
    """
    logging: aborted
    """

    def __init__(self, scenario=None):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['scenario', 'data', 'command'] if
            (scenario is not None) else []
        )
        self.scenario = scenario
        self.data = 'Task has aborted.'
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)

    def execute(self, ud):
        rospy.loginfo('LOG: scenario: %s: %s' % (self.scenario, self.data))
        message = Event()
        message.event = 'E_LOG'
        params = []
        par = Parameter(name='SCENARIO',
                        value=self.scenario)
        params.append(par)
        par = Parameter(name='DATA',
                        value=self.data)
        params.append(par)
        message.params = params
        self.pubEvent.publish(message)
        return 'succeeded'

class DoLogFail(State):
    """
    Logging of overall fail (4 failed grasps) for pickup
    """

    def __init__(self, scenario='PickUp'):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['scenario', 'data']
        )
        self.scenario = scenario
        self.data = 'Pickup has failed all 4 times to grasp.'
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)

    def execute(self, ud):
        rospy.loginfo('LOG: scenario: %s: %s' % (self.scenario, self.data))
        message = Event()
        message.event = 'E_LOG'
        params = []
        par = Parameter(name='SCENARIO',
                        value=self.scenario)
        params.append(par)
        par = Parameter(name='DATA',
                        value=self.data)
        params.append(par)
        message.params = params
        self.pubEvent.publish(message)
        return 'succeeded'

class DoLogStart(DoLog):
    """
    Inherit from DoLog but specialize in logging of Task started
    """

    def __init__(self):
        DoLog.__init__(self)

        self.scenario = None
        self.data = 'Task has started.'
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)


class DoLogSocialRoleChange(DoLog):

    """
    The logging to the Event topic can be done from here
    """

    def __init__(self, change=None):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted']
        )
        self.change = change
        self.scenario = 'Social role change'
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)

    def execute(self, ud):
        if rospy.has_param('/social_role'):
            sr = rospy.get_param('/social_role')
        else:
            return 'aborted'
        data = 'Social role has changed:'
        rospy.loginfo('LOG: scenario: %s: %s' % (self.scenario, data))
        message = Event()
        message.event = 'E_LOG'
        params = []
        par = Parameter(name='SCENARIO',
                        value=self.scenario)
        params.append(par)
        par = Parameter(name='DATA',
                        value=data)
        params.append(par)
        if self.change == 'up':
            par = Parameter(name='OLD VALUE',
                            value=str(sr - 1))
        elif self.change == 'down':
            par = Parameter(name='OLD VALUE',
                            value=str(sr + 1))
        else:
            return 'aborted'
        params.append(par)
        par = Parameter(name='NEW VALUE',
                        value=str(sr))
        params.append(par)
        message.params = params
        self.pubEvent.publish(message)
        return 'succeeded'

#df
class DoLogScenarioAndData(State): #scenario=Pickup!, data: string

    """
    The logging to the Event topic can be done from here for Pickup scenario and data given as string
    """

    def __init__(self, data=""):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['logscenario', 'logdata']
        )
        self.scenario = 'Pickup'
        self.data = data
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)

    def execute(self, ud):
         
        print "DoLogScenarioAndData(): "
        print "scenario: ", self.scenario
        print "data: ", self.data
        
        message = Event()
        message.event = 'E_LOG'
        params = []
        par = Parameter(name='SCENARIO',
                        value=self.scenario)
        params.append(par)
        par = Parameter(name='DATA',
                        value=self.data)
        params.append(par)
        message.params = params
        self.pubEvent.publish(message)
        return 'succeeded'
   
    
        

def battery_log_cb(msg, ud):
    pubEvent = rospy.Publisher('Event', Event, queue_size=1)
    message = Event()
    message.event = 'E_LOG'
    params = []
    par = Parameter(name='SCENARIO',
                    value='Recharge')
    params.append(par)
    par = Parameter(name='Percentage',
                    value=str(msg.lifePercent))
    params.append(par)
    par = Parameter(name='Voltage',
                    value=str(msg.voltage))
    params.append(par)
    message.params = params
    pubEvent.publish(message)
    return True


def do_log_battery_state():
    seq = Sequence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        connector_outcome='succeeded')
    with seq:
        Sequence.add(
            'LOG_BATTERY_LEVEL',
            WaitForMsgState(
                '/battery_state',
                BatteryState,
                msg_cb=battery_log_cb)
        )
    return seq

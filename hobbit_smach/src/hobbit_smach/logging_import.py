#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from smach import State
from hobbit_msgs.msg import Event, Parameter


class DoLog(State):

    """
    The logging to the Event topic can be done from here
    """

    def __init__(self, scenario=None, data=None):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['scenario', 'data'] if
            (scenario is None and data is None) else []
        )
        self.scenario = scenario
        self.data = data
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)

    def execute(self, ud):
        scenario = ud.scenario if self.scenario is None else self.scenario
        data = ud.data if self.data is None else self.data
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


class DoLogPreempt(DoLog):
    """
    Inherit from DoLog but specialize in logging of preemption
    """

    def __init__(self, scenario=None):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['scenario', 'data'] if
            (scenario is None) else []
        )
        self.scenario = scenario
        self.data = 'Task has preempted.'
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)


class DoLogSuccess(DoLog):
    """
    Inherit from DoLog but specialize in logging of preemption
    """

    def __init__(self, scenario=None):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['scenario', 'data'] if
            (scenario is None) else []
        )
        self.scenario = scenario
        self.data = 'Task has succeeded.'
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)


class DoLogAborted(DoLog):
    """
    Inherit from DoLog but specialize in logging of aborted
    """

    def __init__(self, scenario=None):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['scenario', 'data'] if
            (scenario is None) else []
        )
        self.scenario = scenario
        self.data = 'Task has aborted.'
        self.pubEvent = rospy.Publisher('Event', Event, queue_size=50)

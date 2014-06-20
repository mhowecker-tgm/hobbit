#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'aal_lights'

import roslib
roslib.load_manifest(PKG)
import rospy

from smach import State
from hobbit_msgs.msg import Command, Parameter


class SetLights(State):
    """
    Publish the on or off message to the AAL node on topic
    topic: /AAL

    'on' switch the power outlet on
    'off' switch the power outlet off
    """
    def __init__(self, active=True):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'failed']
        )
        self.obstacles = rospy.Publisher('AAL', Command,
                                         latch=False, queue_size=50)
        self.active = active
        self.data = Command()
        self.data.command = 'AAL'

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        p = Parameter('busid', '1')
        self.data.params.append(p)
        p = Parameter('actuatorid', 'FLOORLAMP1')
        self.data.params.append(p)
        if self.active:
            p = Parameter('value', 'on')
            self.data.params.append(p)
        else:
            p = Parameter('value', 'off')
            self.data.params.append(p)
        self.obstacles.publish(self.data)
        return 'succeeded'

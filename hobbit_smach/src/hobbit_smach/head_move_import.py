#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'head_move'

import roslib
roslib.load_manifest(PKG)
import rospy

from smach import State
from std_msgs.msg import String


class MoveTo(State):
    """
    Publish a message to the head_move topic.
    """
    def __init__(self, pose='center_center'):
        State.__init__(self,
                       outcomes=['succeeded', 'preempted', 'failed'])
        self._publisher = rospy.Publisher('/head_move', String)
        self._pose = pose
        self._available_poses = ['center_center', 'up_center', 'down_center',
                                 'up_right', 'center_right', 'down_right',
                                 'up_left', 'center_left', 'down_left']

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if self._pose in self._available_poses:
            self._publisher.publish(self._pose)
            return 'succeeded'
        else:
            return 'failed'

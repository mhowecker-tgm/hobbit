#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'SocialRole'

import rospy
import roslib
roslib.load_manifest(PKG)

from smach import State
from hobbit_msgs import MMUIInterface as MMUI


class GetSocialRole(State):
    """
    Load social role parameter from ROS parameter server and
    return it as outcome
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['butler', 'tool', 'companion', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if rospy.has_param('social_role'):
            social_role = rospy.get_param('social_role')
        else:
            social_role = 2
        if social_role == 0:
            return 'tool'
        elif 1 <= social_role <= 3:
            return 'butler'
        else:
            return 'companion'


class CalculateNextPose(State):
    """
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            output_keys=['x', 'y', 'yaw']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'succeeded'


class Games(State):
    """
    Switch to the games menu on the MMUI. Wrapped in a SMACH state.
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.GoToMenu('M_GAMES')
        print(resp)
        return 'succeeded'
        return 'failed'


class CheckHelpAccepted(State):
    """
    Check the userdata key
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['help_accepted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if ud.help_accepted:
            return 'succeeded'
        else:
            return 'aborted'

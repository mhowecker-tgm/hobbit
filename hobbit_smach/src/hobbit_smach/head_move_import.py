#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'head_move'

import rospy

from smach import State
from std_msgs.msg import String
from std_srvs.srv import Empty


class MoveTo(State):
    """
    Publish a message to the head_move topic.
    """
    def __init__(self, pose='center_center'):
        State.__init__(self,
                       outcomes=['succeeded', 'preempted', 'aborted'])
        self._publisher = rospy.Publisher('/head/move', String, queue_size=50)
        self._pose = pose
        self._available_poses = ['center_center', 'up_center', 'down_center',
                                 'up_right', 'center_right', 'down_right',
                                 'up_left', 'center_left', 'down_left',
                                 'to_grasp', 'to_turntable', 'search_table',
                                 'littledown_center']

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.wait_for_service('/emergency_detector/startHeadMotion', timeout=2)
        startHeadMotion = rospy.ServiceProxy('/emergency_detector/startHeadMotion', Empty)
        try:
            resp = startHeadMotion()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        if self._pose in self._available_poses:
            print(self._pose)
            self._publisher.publish(self._pose)
            rospy.sleep(2)
            rospy.wait_for_service('/emergency_detector/stopHeadMotion', timeout=2)
            stopHeadMotion = rospy.ServiceProxy('/emergency_detector/stopHeadMotion', Empty)
            try:
                resp = stopHeadMotion()
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
            return 'succeeded'
        else:
            print(self._pose)
            return 'aborted'

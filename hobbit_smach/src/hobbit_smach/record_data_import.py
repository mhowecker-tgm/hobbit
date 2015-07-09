#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'record_data'

__author__ = 'Markus Bajones'


import rospy
import subprocess
from sensor_msgs.msg import PointCloud2
from smach import State


class GrabAndSendData(State):
    """
    grab one message from a topic and send it to another one. On the output topic is a listener waiting that will
    save the incoming message as a pcd file.
    """

    def __init__(self, topic_in=None, topic_out=None):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )
        assert isinstance(topic_in, str)
        self.input = topic_in
        assert isinstance(topic_out, str)
        self.output = topic_out

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if not all([self.input, self.output]):
            rospy.loginfo('No input/output topic set')
            return 'aborted'
        try:
            msg = rospy.wait_for_message(self.input, PointCloud2, timeout=10)
            pub = rospy.Publisher(self.output, PointCloud2)
            pub.publish(msg)
            return 'succeeded'
        except rospy.ROSException as e:
            rospy.loginfo('Timeout occurred while waiting for data: ', e)
        except rospy.ROSInterruptException as e:
            rospy.loginfo('Shutdown initiated while waiting for data: ', e)
        return 'aborted'



class GrabScreenContent(State):
    """
    execute screenshot utility
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
        ret = subprocess.call(['/opt/ros/hobbit_hydro/src/hobbit_launch/scripts/miracenter_screenshot.sh'])
        rospy.loginfo('SCREENSHOT call returned: ',ret)
        return 'succeeded'

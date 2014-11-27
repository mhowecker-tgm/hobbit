#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_user_interaction'
NAME = 'hobbit_emotions'

import roslib
roslib.load_manifest(PKG)
import rospy
from smach import State
from std_msgs.msg import String


class ShowEmotions(State):
    """
    Class to display emotions via the two displays in
    HOBBIT's head
    Possible emotions are:
        HAPPY VHAPPY LTIRED VTIRED CONCERNED SAD WONDERING NEUTRAL SLEEPING
    """
    def __init__(self, emotion='NEUTRAL', emo_time=4):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'],
            input_keys=['emotion', 'emo_time']
        )
        self.pub_emo = rospy.Publisher('/head/emo', String, queue_size=50)
        self.emotion = emotion
        self.emo_time = emo_time
        self.default_emotion = 'NEUTRAL'

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
	if rospy.has_param('/Hobbit/current_emotion'):
            self.default_emotion = rospy.get_param('/Hobbit/current_emotion')
        rospy.set_param('/Hobbit/current_emotion', self.emotion)
        self.pub_emo.publish(self.emotion)
        if self.emo_time == 0:
            return 'succeeded'
        else:
            rospy.sleep(self.emo_time)
        self.pub_emo.publish(self.default_emotion)
        return 'succeeded'


if __name__ == '__main__':
    print('You should not call this directly. Import the needed classes')

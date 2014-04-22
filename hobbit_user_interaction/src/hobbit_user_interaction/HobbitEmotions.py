#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_user_interaction'
NAME = 'hobbit_emotions'

import roslib
roslib.load_manifest(PKG)
import rospy
import smach

from std_msgs.msg import String


class ShowEmotions(smach.State):
    """
    Class to display emotions via the two displays in
    HOBBIT's head
    Possible emotions are:
        HAPPY VHAPPY LTIRED VTIRED CONCERNED SAD WONDERING NEUTRAL SLEEPING
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['emotion', 'emo_time']
                             )
        self.pub_emo = rospy.Publisher('/head/emo', String)
        self.emotion = String('NEUTRAL')
        self.emo_time = 4

    def execute(self, ud):
        print(ud.emotion)
        print(ud.emo_time)
        if ud.emotion:
            self.emotion = ud.emotion
            self.emo_time = ud.emo_time
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        self.pub_emo.publish(self.emotion)
        if not ud.emo_time:
            return 'succeeded'
        else:
            rospy.sleep(self.emo_time)
            return 'succeeded'


if __name__ == '__main__':
    print('You should not call this directly. Import the needed classes')

#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import uashh_smach.util as util
from smach import StateMachine
from hobbit_msgs.msg import Event

def event_cb(msg, userdata):
    #rospy.loginfo('Got message with event: '+str(msg.event))
    if msg.event.upper() in ['B_CALLENDED']:
        return True
    else:
        return False


def wait_for_end_of_call():
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with sm:
        StateMachine.add(
            'WAIT_FOR_END_OF_CALL',
            util.WaitForMsgState(
                '/Event',
                Event,
                msg_cb=event_cb),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'WAIT_FOR_END_OF_CALL',
                         'preempted': 'preempted'}
        )
    return sm

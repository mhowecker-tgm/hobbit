#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import uashh_smach.util as util

from smach import StateMachine, State
from hobbit_msgs.msg import Event
from hobbit_msgs.srv import SwitchVision, SwitchVisionRequest
from std_msgs.msg import String
from smach_ros import ServiceState
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.head_move_import as head_move
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.logging_import as log


def switch_vision_cb(ud, response):
    if response.result:
        return 'succeeded'
    else:
        return 'aborted'


class Init(State):
    """
    Class to initialize certain parameters
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'canceled'],
            input_keys=['command', 'parameters'],
            output_keys=['social_role', 'object_name'])

    def execute(self, ud):
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        if ud.command.data == 'cancel':
            return 'canceled'
        return 'succeeded'


class CleanUp(State):
    """
    Class for setting the result message and clean up persistent variables
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['command', 'visited_places'],
            output_keys=['result', 'command', 'visited_places'])
        self.init = None


    def execute(self, ud):
        if not self.init:
            self.init = True
            self.pub_face = rospy.Publisher(
                '/Hobbit/Emoticon',
                String,
                queue_size=50)
            rospy.sleep( 1.0 )
        self.pub_face.publish('EMO_SAD')
        ud.visited_places = []
        ud.result = String('object not detected')
        return 'succeeded'


class SetSuccess(State):
    """
    Class for setting the success message in the actionlib result and clean
    up of persistent variables
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            output_keys=['result', 'visited_places'])

    def execute(self, ud):
        ud.visited_places = []
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        return 'succeeded'


def event_cb(msg, userdata):
    rospy.loginfo('Got message with event: '+str(msg.event))
    if msg.event.upper() == 'E_FITNESS_CLOSED':
        return True
    else:
        return False


def get_do_fitness():
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command', 'parameters'],
        output_keys=['result'])

    sm.userdata.result = String('started')
    sm.userdata.detection = False

    with sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'MOVE_BASE',
                         'canceled': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'MOVE_BASE',
            hobbit_move.goToPosition(
                frame='/map',
                room='None',
                place='fitness'
            ),
            transitions={'succeeded': 'SWITCH_VISION_TO_FITNESS',
                         'preempted': 'CLEAN_UP',
                         'aborted': 'LOG_ABORTED'},
        )
        StateMachine.add_auto(
            'SWITCH_VISION_TO_FITNESS',
            ServiceState(
                '/vision_system/fitnessFunction',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb
            ),
            connector_outcomes=['succeeded', 'preempted', 'aborted']
        )
        StateMachine.add(
            'MOVE_HEAD_UP',
            head_move.MoveTo(pose='littledown_center'),
            transitions={'succeeded': 'DO_FITNESS',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'DO_FITNESS'}
        )
        StateMachine.add(
            'DO_FITNESS',
            HobbitMMUI.ShowMenu(
                menu='FITNESS'
            ),
            transitions={'succeeded': 'WAIT_FOR_END',
                         'failed': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'WAIT_FOR_END',
            util.WaitForMsgState(
                '/Event',
                Event,
                msg_cb=event_cb),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'aborted': 'WAIT_FOR_END',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Fitness'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Fitness'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORTED',
            log.DoLogAborted(scenario='Fitness'),
            transitions={'succeeded': 'aborted'}
        )
        StateMachine.add(
            'SET_SUCCESS',
            SetSuccess(),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'CLEAN_UP'}
        )
        StateMachine.add(
            'CLEAN_UP',
            CleanUp(),
            transitions={'succeeded': 'preempted'}
        )
    return sm

if __name__ == '__main__':
    print("Do not call this directly. Import it into your node.")

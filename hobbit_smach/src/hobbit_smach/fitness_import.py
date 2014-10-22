#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import uashh_smach.util as util

from smach import StateMachine, State
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
        ud.object_name = ud.parameters[0]

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
        self.pub_face = rospy.Publisher(
            '/Hobbit/Emoticon',
            String,
            queue_size=50)

    def execute(self, ud):
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
            transitions={'succeeded': 'MMUI_ConfirmPlace',
                         'canceled': 'CLEAN_UP'}
        )
        # TODO: implement ConfirmPlace with roomname in the question
        StateMachine.add(
            'MMUI_ConfirmPlace',
            HobbitMMUI.AskYesNo(question='T_GT_ConfirmGoToPlace'),
            transitions={'yes': 'EMO_HAPPY',
                         'no': 'LOG_ABORTED',
                         'failed': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT',
                         'timeout': 'MMUI_ConfirmPlace',
                         '3times': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'EMO_HAPPY',
            HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=4),
            transitions={'preempted': 'LOG_PREEMPT',
                         'succeeded': 'MOVE_BASE',
                         'failed': 'LOG_ABORTED'}
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
                         'aborted': 'CLEAN_UP'},
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
            head_move.MoveTo(pose='center_center'),
            transitions={'succeeded': 'DO_FITNESS',
                         'preempted': 'CLEAN_UP',
                         'aborted': 'DO_FITNESS'}
        )
        StateMachine.add(
            'DO_FITNESS',
            HobbitMMUI.ShowMenu(
                menu='FITNESS'
            ),
            transitions={'succeeded': 'succeeded',
                         'failed': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'SAY_MOVING_TO_YOU',
            speech_output.sayText(info='T_FI_MovingToYou'),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'failed': 'LOG_ABORTED',
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

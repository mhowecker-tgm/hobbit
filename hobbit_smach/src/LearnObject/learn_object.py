#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'learn_object'
DEBUG = False

import roslib
roslib.load_manifest(PKG)
import rospy
from uashh_smach.util import SleepState

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction
from hobbit_msgs.srv import SwitchVision, SwitchVisionRequest
from smach_ros import ActionServerWrapper, IntrospectionServer, ServiceState
from smach import StateMachine, State, Sequence, Concurrence
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
import hobbit_smach.learn_object_import as learn_object
import hobbit_smach.head_move_import as head_move
import hobbit_smach.arm_move_import as arm_move
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.logging_import as log


def switch_vision_cb(ud, response):
    if response.result:
        rospy.loginfo('response from vision system was: True')
        return 'succeeded'
    else:
        rospy.loginfo('response from vision system was: False')
        return 'aborted'


class TestData(State):
    """
    """

    def __init__(self, ):
        State.__init__(
            self,
            input_keys=['object_name'],
            outcomes=['succeeded', 'failed', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('TestData')
        print(ud.object_name)
        return 'succeeded'


class Init(State):
    """Class to initialize certain parameters"""
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded'],
            output_keys=['social_role', 'block_counter'])

    def execute(self, ud):
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        ud.block_counter = 0
        return 'succeeded'


class FailCount(State):
    """Class to initialize certain parameters"""
    def __init__(self):
        State.__init__(
            self,
            outcomes=['3times', 'less']
        )
        self._count = 0

    def execute(self, ud):
        if self._count < 3:
            return 'less'
        else:
            return '3times'


class CleanUp(State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['command'],
            output_keys=['result', 'command']
        )
        self.pub_face = rospy.Publisher('/head/emo', String, queue_size=50)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
        ud.result = String('Unable to go to location')
        return 'succeeded'


class SetSuccess(State):
    """
    Class for setting the success message in the actionlib result \
        and clean up of persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            output_keys=['result']
        )
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String, queue_size=50)
        self.pub_face = rospy.Publisher('/head/emo', String, queue_size=50)

    def execute(self, ud):
        self.pub_face.publish('EMO_HAPPY')
        self.pub.publish('Stop')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('Location reached')
        return 'succeeded'


class SetFailure(State):
    """
    Class for setting the failure message in the actionlib result \
        and clean up of persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            output_keys=['result']
        )
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String, queue_size=50)

    def execute(self, ud):
        self.pub.publish('Stop')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('failure')
        return 'succeeded'


class Dummy(State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['command'],
            output_keys=['result', 'command']
        )

    def execute(self, ud):
        return 'succeeded'


def main():
    rospy.init_node(NAME)

    learn_object_sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command', 'previous_state', 'parameters'],
        output_keys=['result'])

    seq1 = Sequence(
        outcomes=['succeeded', 'failed', 'preempted', 'aborted'],
        connector_outcome='succeeded'
    )

    seq2 = Sequence(
        outcomes=['succeeded', 'failed', 'preempted', 'aborted'],
        connector_outcome='succeeded',
        input_keys=['object_name']
    )
    learn_object_sm.userdata.result = String('started')
    learn_object_sm.userdata.emotion = 'WONDERING'
    learn_object_sm.userdata.emo_time = 4

    cc1 = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed', 'aborted'],
        default_outcome='failed',
        input_keys=['object_name'],
        outcome_map={'succeeded': {'EMO_WONDERING': 'succeeded',
                                   'SAY_LEARN_NEW_OBJECT': 'succeeded'}}
    )

    with cc1:
        Concurrence.add(
            'EMO_WONDERING',
            HobbitEmotions.ShowEmotions(emotion='WONDERING', emo_time=4)
        )
        Concurrence.add(
            'SAY_LEARN_NEW_OBJECT',
            speech_output.sayText(
                info='T_LO_LEARNING_NEW_SET_OF_OBJECTS_I_AM_THINKING')
        )

    with seq1:
        Sequence.add(
            'SAY_HAPPY',
            speech_output.emo_say_something(
                emo='HAPPY',
                time=4,
                text='T_LO_ATTENTION_I_AM_MOVING_MY_ARM_OUT'
            )
        )
        Sequence.add(
            'HEAD_MOVE',
            head_move.MoveTo(pose='littledown_center'),
            transitions={'aborted': 'failed'}
        )
        Sequence.add(
            'HEAD_MOVE_2',
            head_move.MoveTo(pose='down_center'),
            transitions={'aborted': 'failed'})
        Sequence.add(
            'MMUI_SAY_GRASPING',
            speech_output.sayText(info='T_LO_GRASPING_THE_TURNTABLE'))
        Sequence.add(
            'MOVE_TT_LEARN_POSITION',
            arm_move.goToLearnPosition()
        )
        Sequence.add(
            'HEAD_MOVE_3',
            head_move.MoveTo(pose='to_turntable'),
            transitions={'aborted': 'failed'})

        with seq2:
            Sequence.add(
                'SAY_LEARN_NEW_OBJECT',
                cc1
            )
            Sequence.add(
                'WAIT_3',
                SleepState(duration=1)
            )
            Sequence.add(
                'SAY_DONE',
                speech_output.sayTextObject(
                    info='T_LO_I_AM_DONE'
                )
            )
            Sequence.add(
                'SAY_THANKS',
                speech_output.sayTextObject(
                    info='T_LO_ThankYouTeachingNewObject_O')
            )
            Sequence.add('MMUI_MAIN_MENU', HobbitMMUI.ShowMenu(menu='MAIN'))

    with learn_object_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'SEQ1'}
        )
        StateMachine.add(
            'SEQ1',
            seq1,
            transitions={'succeeded': 'CONFIRM_PUT_OBJECT_ON_TRAY',
                         'failed': 'SET_FAILURE',
                         'aborted': 'SET_FAILURE',
                         'preempted': 'LOG_PREEMPTED'}
        )
        # StateMachine.add(
        #     'MMUI_ASK_HELP_OBJECT',
        #     HobbitMMUI.AskYesNo(question='HELP_PUTTING_OBJECT_ONTO_TURNTABLE'),
        #     transitions={'yes': 'CONFIRM_PUT_OBJECT_ON_TRAY',
        #                  'no': 'SHALL_I_STOP_1',
        #                  'timeout': 'MMUI_ASK_HELP_OBJECT',
        #                  '3times': 'SHALL_I_STOP_1',
        #                  'failed': 'SHALL_I_STOP_1'}
        # )
        # StateMachine.add(
        #     'SHALL_I_STOP_1',
        #     HobbitMMUI.AskYesNo(question='SHALL_I_STOP_LEARNING'),
        #     transitions={'yes': 'STOP_SEQ',
        #                  'no': 'EMO_HAPPY',
        #                  'timeout': 'SHALL_I_STOP_1',
        #                  '3times': 'STOP_SEQ',
        #                  'failed': 'STOP_SEQ'}
        # )
        # StateMachine.add(
        #     'EMO_HAPPY',
        #     HobbitEmotions.ShowEmotions(emotion='HAPPY', emo_time=4),
        #     transitions={'succeeded': 'CONFIRM_PUT_OBJECT_ON_TRAY',
        #                  'failed': 'SET_FAILURE'}
        # )
        StateMachine.add(
            'STOP_SEQ',
            learn_object.unloadReturnTurntable(),
            transitions={'succeeded': 'STOP_SEQ_2',
                         'failed': 'SET_FAILURE',
                         'preempted': 'SET_FAILURE'}
        )
        StateMachine.add(
            'CONFIRM_PUT_OBJECT_ON_TRAY',
            HobbitMMUI.ConfirmInfo(
                info='T_LO_PLEASE_PUT_OBJECT_ONTO_TURNTABLE'),
            transitions={'succeeded': 'CHECK_FOR_OBJECT_1',
                         'failed': 'FAIL_COUNT_CONFIRM_1'}
        )
        StateMachine.add(
            'CHECK_FOR_OBJECT_1',
            Dummy(),
            transitions={'succeeded': 'SAY_I_AM_LEARNING',
                         'failed': 'FAIL_COUNT_OBJECT_1'}
        )
        StateMachine.add(
            'CHECK_FOR_OBJECT_2',
            Dummy(),
            transitions={'succeeded': 'SAY_LEARN_FROM_DIFFERENT_ANGLE',
                         'failed': 'FAIL_COUNT_OBJECT_2'}
        )
        StateMachine.add(
            'FAIL_COUNT_CONFIRM_1',
            FailCount(),
            transitions={'3times': 'STOP_SEQ_2',
                         'less': 'CONFIRM_PUT_OBJECT_ON_TRAY'}
        )
        StateMachine.add(
            'FAIL_COUNT_OBJECT_1',
            FailCount(),
            transitions={'3times': 'STOP_SEQ_2',
                         'less': 'SAY_UNABLE_TO_SEE_OBJECT'}
        )
        StateMachine.add(
            'SAY_UNABLE_TO_SEE_OBJECT',
            speech_output.sayText(
                info='T_LO_I_COULD_NOT_FIND_OBJECT_ON_TURNTABLE'),
            transitions={'succeeded': 'CONFIRM_PUT_OBJECT_ON_TRAY',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'SAY_I_AM_LEARNING',
            speech_output.sayText(info='T_LO_I_AM_LEARNING_THE_OBJECT'),
            transitions={'succeeded': 'SWITCH_VISION',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add_auto(
            'SWITCH_VISION',
            ServiceState(
                '/vision_system/startScanning3DObject',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb

            ),
            connector_outcomes=['succeeded', 'preempted', 'aborted']
        )
        StateMachine.add(
            'LEARN_OBJECT_1',
            learn_object.getDataCCW(),
            transitions={'succeeded': 'CONFIRM_TURN_OBJECT',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'CONFIRM_TURN_OBJECT',
            HobbitMMUI.ConfirmInfo(
                info='T_LO_TURN_OBJECT_UPSIDE_DOWN_OR_LAY_IT_DOWN'),
            transitions={'succeeded': 'CHECK_FOR_OBJECT_2',
                         'failed': 'FAIL_COUNT_CONFIRM_2'}
        )
        StateMachine.add(
            'FAIL_COUNT_CONFIRM_2',
            FailCount(),
            transitions={'3times': 'SHALL_I_STOP_2',
                         'less': 'CONFIRM_TURN_OBJECT'}
        )
        StateMachine.add(
            'FAIL_COUNT_OBJECT_2',
            FailCount(),
            transitions={'3times': 'STOP_SEQ_2',
                         'less': 'SAY_UNABLE_TO_SEE_OBJECT_2'}
        )
        StateMachine.add(
            'SAY_UNABLE_TO_SEE_OBJECT_2',
            speech_output.sayText(
                info='T_LO_I_COULD:NOT_FIND_OBJECT_ON_TURNTABLE'),
            transitions={'succeeded': 'CONFIRM_TURN_OBJECT',
                         'failed': 'STOP_SEQ_2'}
        )
        StateMachine.add(
            'FAIL_COUNT_2',
            FailCount(),
            transitions={'3times': 'SHALL_I_STOP_2',
                         'less': 'CONFIRM_TURN_OBJECT'}
        )
        StateMachine.add(
            'SHALL_I_STOP_2',
            HobbitMMUI.AskYesNo(question='T_LO_SHALL_I_STOP_LEARNING'),
            transitions={'yes': 'STOP_SEQ',
                         'no': 'CONFIRM_TURN_OBJECT',
                         'failed': 'STOP_SEQ',
                         '3times': 'STOP_SEQ',
                         'timeout': 'SHALL_I_STOP_2'}
        )
        StateMachine.add(
            'LEARN_OBJECT_2',
            learn_object.getDataCW(),
            transitions={'succeeded': 'SWITCH_VISION_BACK',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add_auto(
            'SWITCH_VISION_BACK',
            ServiceState(
                '/vision_system/stopScanning3DObject',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb
            ),
            connector_outcomes=['succeeded', 'preempted', 'aborted']
        )
        StateMachine.add(
            'HEAD_UP',
            head_move.MoveTo(pose='littledown_center'),
            transitions={'succeeded': 'SAY_WHAT_IS_THE_NAME',
                         'aborted': 'SET_FAILURE'}
        )
        StateMachine.add(
            'SAY_WHAT_IS_THE_NAME',
            speech_output.sayText(
                info='T_LO_WHAT_IS_THE_NAME_OF_THIS_OBJECT'),
            transitions={'succeeded': 'GET_THE_NAME_OF_THE_OBJECT',
                         'failed': 'STOP_SEQ'}
        )
        StateMachine.add(
            'SAY_LEARN_FROM_DIFFERENT_ANGLE',
            speech_output.sayText(
                info='T_LO_WAIT_LEARNING_OBJECT_FROM_NEW_ANGLE'),
            transitions={'succeeded': 'LEARN_OBJECT_2',
                         'failed': 'STOP_SEQ'}
        )
        StateMachine.add(
            'GET_THE_NAME_OF_THE_OBJECT',
            speech_output.AskForName(),
            transitions={'succeeded': 'SET_PCD_NAME',
                         'failed': 'STOP_SEQ'}
        )
        StateMachine.add(
            'SET_PCD_NAME',
            learn_object.SetName(),
            transitions={'succeeded': 'END_SEQ',
                         'failure': 'STOP_SEQ'}
        )
        StateMachine.add(
            'END_SEQ',
            seq2,
            transitions={'succeeded': 'STOP_SEQ_2',
                         'failed': 'STOP_SEQ',
                         'aborted': 'STOP_SEQ'}
        )
        StateMachine.add(
            'SET_FAILURE',
            SetFailure(),
            transitions={'succeeded': 'MAIN_MENU',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'MAIN_MENU',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'succeeded': 'LOG_ABORTED',
                         'preempted': 'preempted',
                         'failed': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'CLEAN_UP',
            CleanUp(),
            transitions={'succeeded': 'preempted'})
        StateMachine.add(
            'STOP_SEQ_2',
            learn_object.returnTurntable(),
            transitions={'succeeded': 'succeeded',
                         'failed': 'SET_FAILURE',
                         'preempted': 'SET_FAILURE'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='learn object'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_ABORTED',
            log.DoLogAborted(scenario='learn object'),
            transitions={'succeeded': 'aborted'}
        )
        StateMachine.add(
            'LOG_PREEMPTED',
            log.DoLogPreempt(scenario='learn object'),
            transitions={'succeeded': 'PREEMPT_MENU'}
        )
        StateMachine.add(
            'PREEMPT_MENU',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'succeeded': 'preempted',
                         'preempted': 'preempted',
                         'failed': 'preempted'}
        )

    asw = ActionServerWrapper(
        'learn_object',
        GeneralHobbitAction,
        learn_object_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command',
                        'previous_state': 'previous_state',
                        'parameters': 'parameters'}
    )

    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()

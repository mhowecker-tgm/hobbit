#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'learn_object'
DEBUG =  False
MMUI_IS_DOING_IT = True

import roslib
roslib.load_manifest(PKG)
import rospy
from uashh_smach.util import SleepState
import hobbit_smach.hobbit_move_import as hobbit_move

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction
from smach_ros import ActionServerWrapper, IntrospectionServer
from smach import StateMachine, State, Sequence, Concurrence
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
import hobbit_smach.learn_object_import as learn_object
import hobbit_smach.head_move_import as head_move
import hobbit_smach.arm_move_import as arm_move
import hobbit_smach.speech_output_import as speech_output


class Init(State):
    """Class to initialize certain parameters"""
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failure'],
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
        outcomes=['succeeded', 'failure', 'preempted'],
        input_keys=['command', 'previous_state', 'parameters'],
        output_keys=['result'])

    seq1 = Sequence(
        outcomes=['succeeded', 'failed', 'preempted'],
        connector_outcome='succeeded'
    )

    seq2 = Sequence(
        outcomes=['succeeded', 'failed', 'preempted'],
        connector_outcome='succeeded'
    )
    learn_object_sm.userdata.result = String('started')
    learn_object_sm.userdata.emotion = 'WONDERING'
    learn_object_sm.userdata.emo_time = 4

    cc1 = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed'
    )

    cc2 = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed'
    )

    with cc1:
        Concurrence.add(
            'EMO_HAPPY',
            HobbitEmotions.ShowEmotions(emotion='HAPPY', emo_time=4)
        )
        Concurrence.add(
            'SAY_ATTENTION',
            speech_output.sayText(info='T_LO_ATTENTION_I_AM_MOVING_MY_ARM_OUT')
        )

    with cc2:
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
        #Sequence.add(
        #    'SAY_HAPPY',
        #    cc1
        #)
        Sequence.add(
            'HEAD_MOVE',
            head_move.MoveTo(pose='center_center'),
            transitions={'aborted': 'failed'}
        )
        #if not DEBUG:
        #    Sequence.add(
        #        'MOVE_BACK',
        #        hobbit_move.Move(goal='back', distance=0.25))
        Sequence.add(
            'HEAD_MOVE_2',
            head_move.MoveTo(pose='down_center'),
            transitions={'aborted': 'failed'})
        Sequence.add(
            'MMUI_SAY_GRASPING',
            speech_output.sayText(info='T_LO_GRASPING_THE_TURNTABLE'))
        # Sequence.add(
        #     'ARM_MOVE_TT_POSE',
        #     arm_move.goToPosition(pose='storage'))
        # Sequence.add(
        #     'Close_GRIPPER',
        #     arm_move.CloseGripper())
        # Sequence.add(
        #     'ARM_MOVE_LEARN_POSE',
        #     arm_move.goToPosition(pose='learn'))
        Sequence.add(
            'MOVE_TT_LEARN_POSITION',
            arm_move.goToLearnPosition()
        )
        #if not DEBUG:
        #    Sequence.add(
        #        'MOVE_FRONT',
        #        hobbit_move.Move(goal='front', distance=0.25))
        Sequence.add(
            'HEAD_MOVE_3',
            head_move.MoveTo(pose='to_turntable'),
            transitions={'aborted': 'failed'})

        with seq2:
            Sequence.add(
                'SAY_LEARN_NEW_OBJECT',
                cc2
            )
            Sequence.add(
                'WAIT_3',
                SleepState(duration=1)
            )
            Sequence.add(
                'SAY_DONE',
                speech_output.sayText(info='T_LO_I_AM_DONE')
            )
            Sequence.add(
                'SAY_THANKS',
                speech_output.sayText(info='T_LO_ThankYouTeachingNewObject')
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
                         'preempted': 'SET_FAILURE'}
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
            HobbitMMUI.ConfirmInfo(info='T_LO_PLEASE_PUT_OBJECT_ONTO_TURNTABLE'),
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
            speech_output.sayText(info='T_LO_I_COULD_NOT_FIND_OBJECT_ON_TURNTABLE'),
            transitions={'succeeded': 'CONFIRM_PUT_OBJECT_ON_TRAY',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'SAY_I_AM_LEARNING',
            speech_output.sayText(info='T_LO_I_AM_LEARNING_THE_OBJECT'),
            transitions={'succeeded': 'LEARN_OBJECT_1',
                         'failed': 'SET_FAILURE'}
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
            transitions={'succeeded': 'HEAD_UP',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'HEAD_UP',
            head_move.MoveTo(pose='center_center'),
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
                         'failed': 'STOP_SEQ'}
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
            transitions={'succeeded': 'failure',
                         'preempted': 'preempted',
                         'failed': 'failure'}
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

    asw = ActionServerWrapper(
        'learn_object',
        GeneralHobbitAction,
        learn_object_sm,
        ['succeeded'], ['failure'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command',
                        'previous_state': 'previous_state',
                        'parameters': 'parameters'}
    )

    sis = IntrospectionServer('smach_server',
                              learn_object_sm, '/HOBBIT/LEARN_OBJECT_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

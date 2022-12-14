#!/usr/bin/python
# -*- coding: utf-8 -*-

from smach import StateMachine, State
from smach_ros import ServiceState
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
import hobbit_smach.logging_import as log
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.sos_call_import as sos_call
from hobbit_msgs.srv import SetSafetycheckStateRequest, SetSafetycheckState

def get_safety_check():
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command'],
        output_keys=['result'])

    with sm:
        StateMachine.add_auto(
            'T_SC_CHECK_INITIATED',
            HobbitMMUI.ConfirmOk(text='T_SC_CHECKINITIATED'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_CHECK_INITIATED',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_CHECK_CLOTHING',
            HobbitMMUI.ConfirmOk(text='T_SC_CHECKCLOTHING'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_CHECK_CLOTHING',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_CHECK_CLOTHING2',
            HobbitMMUI.ConfirmOk(text='T_SC_CHECKCLOTHING2'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_CHECK_CLOTHING2',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_CHECK_CLOTHING3',
            HobbitMMUI.ConfirmOk(text='T_SC_CHECKCLOTHING3'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_CHECK_CLOTHING3',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_CHECK_FLOORS',
            HobbitMMUI.ConfirmOk(text='T_SC_CHECKFLOORS'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_CHECK_FLOORS',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_CHECK_FLOORS2',
            HobbitMMUI.ConfirmOk(text='T_SC_FLOORS2'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_CHECK_FLOORS2',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_LIGHTING',
            HobbitMMUI.ConfirmOk(text='T_SC_LIGHTING'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_LIGHTING',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_LIGHTING2',
            HobbitMMUI.ConfirmOk(text='T_SC_LIGHTING2'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_LIGHTING2',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_HEALTH',
            HobbitMMUI.ConfirmOk(text='T_SC_HEALTH'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_HEALTH',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_HEALTH2',
            HobbitMMUI.ConfirmOk(text='T_SC_HEALTH2'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_HEALTH2',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_HEALTH3',
            HobbitMMUI.ConfirmOk(text='T_SC_HEALTH3'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_HEALTH3',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_ROOMS',
            HobbitMMUI.ConfirmOk(text='T_SC_ROOMS'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_ROOMS',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_LIVINGROOM',
            HobbitMMUI.ConfirmOk(text='T_SC_LIVINGROOM'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_LIVINGROOM',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_LIVINGROOM2',
            HobbitMMUI.ConfirmOk(text='T_SC_LIVINGROOM2'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_LIVINGROOM2',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_LIVINGROOM3',
            HobbitMMUI.ConfirmOk(text='T_SC_LIVINGROOM3'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_LIVINGROOM3',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_BEDROOM',
            HobbitMMUI.ConfirmOk(text='T_SC_BEDROOM'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_BEDROOM',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_BEDROOM2',
            HobbitMMUI.ConfirmOk(text='T_SC_BEDROOM2'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_BEDROOM2',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_BEDROOM3',
            HobbitMMUI.ConfirmOk(text='T_SC_BEDROOM3'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_BEDROOM3',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_KITCHEN',
            HobbitMMUI.ConfirmOk(text='T_SC_KITCHEN'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_KITCHEN',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_KITCHEN2',
            HobbitMMUI.ConfirmOk(text='T_SC_KITCHEN2'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_KITCHEN2',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_KITCHEN3',
            HobbitMMUI.ConfirmOk(text='T_SC_KITCHEN3'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_KITCHEN3',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_ANTEROOM',
            HobbitMMUI.ConfirmOk(text='T_SC_ANTEROOM'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_ANTEROOM',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_ANTEROOM2',
            HobbitMMUI.ConfirmOk(text='T_SC_ANTEROOM2'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_ANTEROOM2',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_BATHROOM',
            HobbitMMUI.ConfirmOk(text='T_SC_BATHROOM'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_BATHROOM',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_BATHROOM2',
            HobbitMMUI.ConfirmOk(text='T_SC_BATHROOM2'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_BATHROOM2',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_BATHROOM3',
            HobbitMMUI.ConfirmOk(text='T_SC_BATHROOM3'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_BATHROOM3',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'T_SC_CHECKFINISHED',
            HobbitMMUI.ConfirmOk(text='T_SC_CHECKFINISHED'),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'timeout': 'T_SC_CHECKFINISHED',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'HAPPY_SAY',
            HobbitEmotions.ShowEmotions(
                emotion='HAPPY',
                emo_time=4
            ),
            connector_outcomes=['succeeded', 'failed'],
            transitions={'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'SET_CHECK_FINISHED_TO_TRUE',
            ServiceState('/user/set_safetycheck_state',
                         SetSafetycheckState,
                         request=SetSafetycheckStateRequest(state=True)
            ),
            connector_outcomes=['succeeded', 'aborted'],
            transitions={'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='safety check'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='safety check'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORT',
            log.DoLogAborted(scenario='safety check'),
            transitions={'succeeded': 'aborted'}
        )
        StateMachine.add(
            'USER_NOT_RESPONDING',
            sos_call.get_call_sos_simple(),
            transitions={'succeeded': 'LOG_ABORT',
                         'failed': 'LOG_SUCCESS',
                         'aborted': 'LOG_SUCCESS',
                         'preempted': 'LOG_PREEMPT'}
        )

    return sm

if __name__ == '__main__':
    print("Do not call this directly. Import it into your node.")

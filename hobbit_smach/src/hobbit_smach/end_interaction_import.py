#!/usr/bin/python
# -*- coding: utf-8 -*-

from smach import StateMachine
from smach_ros import ServiceState
from hobbit_msgs.srv import GetName
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.social_role_import as social_role
import hobbit_smach.hobbit_cronjobs_import as cronjobs
import hobbit_smach.logging_import as log
import hobbit_smach.recharge_import as recharge
from hobbit_user_interaction import HobbitMMUI
from uashh_smach.util import SleepState


def end_interaction_muc_pickup():
    """
    Returns a SMACH StateMachine that will ask the user if they want to give
    a new command. If not the robot goes to the docking station.
    """

    sm = StateMachine(
        outcomes=['succeeded', 'preempted', 'aborted']
    )

    with sm:
        StateMachine.add(
            'OFFER_RETURN_OF_FAVOUR',
            HobbitMMUI.AskYesNo(question='T_PU_OfferReturnOfFavour_2'),
            transitions={'yes': 'LOG_ACCEPT_ROF',
                         'timeout': 'OFFER_RETURN_OF_FAVOUR',
                         '3times': 'LOG_DID_NOT_ACCEPT_ROF',
                         'no': 'LOG_DID_NOT_ACCEPT_ROF',
                         'preempted': 'preempted',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'LOG_ACCEPT_ROF',
            log.DoLogROF(
                #scenario='Return of favour',
                data='User accepted return of favour'
            ),
            transitions={'succeeded': 'MUSIC'}
        )
        StateMachine.add(
            'MUSIC',
            HobbitMMUI.ShowMenu(menu='Audio'),
            transitions={'succeeded': 'succeeded',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'LOG_DID_NOT_ACCEPT_ROF',
            log.DoLogROF(
               # scenario='Return of favour',
                data='User did not accept return of favour'
            ),
            transitions={'succeeded': 'aborted'}
        )
        StateMachine.add(
            'MOVE_AWAY',
            move_away(),
            transitions={'succeeded': 'aborted'}

        )
    return sm

def end_interaction_muc_reward():
    """
    Returns a SMACH StateMachine that will ask the user if they want to give
    a new command. If not the robot goes to the docking station.
    """

    sm = StateMachine(
        outcomes=['succeeded', 'preempted', 'aborted']
    )

    with sm:
        StateMachine.add(
            'OFFER_RETURN_OF_FAVOUR',
            HobbitMMUI.AskYesNo(question='T_CA_OfferReturnOfFavour_2'),
            transitions={'yes': 'LOG_ACCEPT_ROF',
                         'timeout': 'OFFER_RETURN_OF_FAVOUR',
                         '3times': 'LOG_DID_NOT_ACCEPT_ROF',
                         'no': 'LOG_DID_NOT_ACCEPT_ROF',
                         'preempted': 'preempted',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'LOG_ACCEPT_ROF',
            log.DoLogROF(
                #scenario='Return of favour',
                data='User accepted return of favour'
            ),
            transitions={'succeeded': 'SURPRISE_MUC'}
        )
        StateMachine.add(
            'SURPRISE_MUC',
            social_role.get_surprise(),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_DID_NOT_ACCEPT_ROF',
            log.DoLogROF(
               # scenario='Return of favour',
                data='User did not accept return of favour'
            ),
            transitions={'succeeded': 'aborted'}
        )
        StateMachine.add(
            'MOVE_AWAY',
            move_away(),
            transitions={'succeeded': 'aborted'}

        )
    return sm

def end_interaction_muc_learn():
    """
    Returns a SMACH StateMachine that will ask the user if they want to give
    a new command. If not the robot goes to the docking station.
    """

    sm = StateMachine(
        outcomes=['succeeded', 'preempted', 'aborted']
    )

    with sm:
        StateMachine.add(
            'OFFER_RETURN_OF_FAVOUR',
            HobbitMMUI.AskYesNo(question='T_LO_OfferReturnOfFavour_2'),
            transitions={'yes': 'LOG_ACCEPT_ROF',
                         'timeout': 'OFFER_RETURN_OF_FAVOUR',
                         '3times': 'LOG_DID_NOT_ACCEPT_ROF',
                         'no': 'LOG_DID_NOT_ACCEPT_ROF',
                         'preempted': 'preempted',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'LOG_ACCEPT_ROF',
            log.DoLogROF(
                #scenario='Return of favour',
                data='User accepted return of favour'
            ),
            transitions={'succeeded': 'SOLITAIRE'}
        )
        StateMachine.add(
            'SOLITAIRE',
            HobbitMMUI.ShowMenu(menu='Solitaire'),
            transitions={'succeeded': 'succeeded',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'LOG_DID_NOT_ACCEPT_ROF',
            log.DoLogROF(
               # scenario='Return of favour',
                data='User did not accept return of favour'
            ),
            transitions={'succeeded': 'aborted'}
        )
        StateMachine.add(
            'MOVE_AWAY',
            move_away(),
            transitions={'succeeded': 'aborted'}

        )
    return sm


def move_away():
    """
    Returns a SMACH StateMachine that checks the social role and moves the
    robot away from the user. Depending on the social role the goal is the
    docking station or the center of the room the robot and the user are in.
    """

    sm = StateMachine(
        outcomes=['succeeded', 'preempted', 'aborted'],
        input_keys=['social_role']
    )

    with sm:
        StateMachine.add(
            'WAIT_60',
            SleepState(duration=1800), #30*60 seconds
            transitions={'succeeded': 'SOCIAL_ROLE_CHECK',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SOCIAL_ROLE_CHECK',
            social_role.GetSocialRole(),
            transitions={'tool': 'MOVE_TO_DOCK',
                         'butler': 'MOVE_TO_DOCK',
                         'companion': 'SET_3_HOURS_TIMER'}
        )
        StateMachine.add(
            'MOVE_TO_DOCK',
            recharge.getRecharge(),
            transitions={'succeeded': 'SET_3_HOURS_TIMER',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'GET_CURRENT_ROOM_NAME',
            ServiceState(
                'get_robots_current_room',
                GetName,
                response_key='room_name'
            ),
            transitions={'succeeded': 'SET_GOAL',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SET_GOAL',
            social_role.GetSocialPose(),
            transitions={'succeeded': 'MOVE_TO_CENTER',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'MOVE_TO_CENTER',
            hobbit_move.goToPose(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SET_3_HOURS_TIMER',
            cronjobs.ThreeHours(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
    return sm

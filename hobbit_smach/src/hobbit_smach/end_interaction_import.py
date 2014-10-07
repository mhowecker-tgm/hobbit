#!/usr/bin/python
# -*- coding: utf-8 -*-

from smach import StateMachine
from smach_ros import ServiceState
from hobbit_msgs.srv import GetName
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.social_role_import as social_role
import hobbit_smach.hobbit_cronjobs_import as cronjobs
from hobbit_user_interaction import HobbitMMUI


def end_interaction_muc():
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
            HobbitMMUI.AskYesNo(question='T_CA_OfferReturnOfFavour'),
            transitions={'yes': 'aborted',
                         'timeout': 'OFFER_RETURN_OF_FAVOUR',
                         '3times': 'aborted',
                         'no': 'MOVE_AWAY',
                         'preempted': 'preempted',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'MOVE_AWAY',
            move_away()
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
            'SOCIAL_ROLE_CHECK',
            social_role.GetSocialRole(),
            transitions={'tool': 'MOVE_TO_DOCK',
                         'butler': 'MOVE_TO_DOCK',
                         'companion': 'GET_CURRENT_ROOM_NAME'}
        )
        StateMachine.add(
            'MOVE_TO_DOCK',
            hobbit_move.goToPosition(
                frame='/map',
                room='dock',
                place='dock'
            ),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'GET_CURRENT_ROOM_NAME',
            ServiceState(
                'get_robots_current_room',
                GetName,
                response_key='room_name'
            )
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

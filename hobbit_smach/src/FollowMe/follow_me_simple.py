#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'follow_me_simple'
DEBUGfollow_me = False
MMUI_IS_DOING_IT = True
PREEMPT_TIMEOUT = 5
SERVER_TIMEOUT = 5

import rospy
import smach
import hobbit_smach.speech_output_import as speech_output

from std_msgs.msg import String
from smach_ros import ActionServerWrapper, IntrospectionServer
from uashh_smach.util import SleepState
from follow_user.msg import Person as FollowPerson
from hobbit_user_interaction import HobbitMMUI


import rospy
from smach import StateMachine, Concurrence, Sequence
from smach_ros import IntrospectionServer, ActionServerWrapper, MonitorState, SimpleActionState
from hobbit_msgs.msg import GeneralHobbitAction, FollowMeAction, FollowMeGoal
import hobbit_smach.logging_import as log


class Init(smach.State):
    """
    Class to initialize certain parameters
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'canceled'],
            input_keys=['command'],
            output_keys=['social_role'])
        self.pub_face = rospy.Publisher(
            '/Hobbit/Emoticon', String,
            queue_size=50)

    def execute(self, ud):
        self.pub_face.publish('EMO_HAPPY')
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        if ud.command.data == 'cancel':
            return 'canceled'
        return 'succeeded'


def main():
    rospy.init_node(NAME)

    fm_sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command'],
        output_keys=['result'])

    def child_term_cb(outcome_map):
        if outcome_map['DETECTION'] == 'succeeded':
            return True

    def out_cb(outcome_map):
        if outcome_map['DETECTION'] == 'succeeded':
            return 'succeeded'

    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        default_outcome='aborted',
        child_termination_cb=child_term_cb,
        outcome_cb=out_cb
    )


    def msg_cb(ud, msg):
        print('person (x,y,z) {}, {}, {}'.format(msg.pose.x, msg.pose.y, msg.pose.z))
        if msg.pose.x: 
            return False


    with cc:
        Concurrence.add(
            'GET_USER',
            MonitorState(
                '/follow_user/persons',
                FollowPerson,
                cond_cb=msg_cb
            ),
            remapping={'invalid': 'aborted',
                       'valid': 'succeeded'}
        )
        Concurrence.add(
            'TIMER',
            SleepState(10)
        )

    with fm_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'SAY_START',
                         'canceled': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'SAY_START',
            speech_output.sayText(
                info="Please stand in front of me so i can follow you."),
            transitions={'succeeded': 'GET_USER',
                         'failed': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'GET_USER',
            cc,
            transitions={'succeeded': 'SAY_FOUND_YOU',
                         'aborted': 'SAY_CANT_SEE'}
        )
        StateMachine.add(
            'SAY_FOUND_YOU',
            speech_output.sayText(
                info="Found you. Will start following you."),
            transitions={'succeeded': 'FOLLOW',
                         'failed': 'LOG_ABORTED'}
        )
        follow_goal=FollowMeGoal()
        follow_goal.command='start'
        StateMachine.add(
            'FOLLOW',
            SimpleActionState(
                'follow_navigation',
                FollowMeAction,
                goal=follow_goal,
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'SAY_STOP',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'SAY_CANT_SEE',
            speech_output.sayText(
                info="I did not see you and will stay here."),
            transitions={'succeeded': 'LOG_ABORTED',
                         'failed': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'SAY_STOP',
            speech_output.sayText(
                info="I will stop moving now."),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'failed': 'LOG_ABORTED'}
        )
        #Sequence.add('MMUI_MAIN_MENU', HobbitMMUI.ShowMenu(menu='MAIN'))

        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Locate user'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Locate user'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORTED',
            log.DoLogAborted(scenario='Locate user'),
            transitions={'succeeded': 'aborted'}
        )
    asw = ActionServerWrapper(
        'follow_me_simple', GeneralHobbitAction, fm_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        #result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command'})
    sis = IntrospectionServer(
        'smach_server', fm_sm, '/HOBBIT/follow_me_simple_sm_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

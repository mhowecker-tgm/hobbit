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
from smach_ros import ServiceState
from std_msgs.msg import String
from std_srvs.srv import Empty
from smach_ros import ActionServerWrapper, IntrospectionServer
from uashh_smach.util import SleepState, WaitForMsgState
from follow_user.msg import Person as FollowPerson
from follow_user.msg import TrackerTarget
from hobbit_user_interaction import HobbitMMUI
import hobbit_smach.head_move_import as head_move
from smach import StateMachine, Concurrence, Sequence
from smach_ros import IntrospectionServer, ActionServerWrapper, MonitorState, SimpleActionState
from hobbit_msgs.msg import GeneralHobbitAction, FollowMeAction, FollowMeGoal
import hobbit_smach.logging_import as log
import hobbit_smach.hobbit_move_import as hobbit_move
from mira_msgs.srv import UserNavMode, ObsNavMode
from hobbit_msgs.srv import SwitchVision, SwitchVisionRequest


def switch_vision_cb(ud, response):
    if response.result:
        rospy.loginfo('switch vision reported: '+str(response.result))
        return 'succeeded'
    else:
        return 'aborted'

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
        print(outcome_map)
        #return True
        if outcome_map['GET_USER'] == 'succeeded':
            return True

    def out_cb(outcome_map):
        return 'succeeded'
        if outcome_map['GET_USER'] == 'invalid':
            return 'succeeded'

    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        default_outcome='aborted',
        child_termination_cb=child_term_cb,
        outcome_cb=out_cb
    )


    def msg_cb(msg, ud):
        print('person (x,y) {}, {}'.format(msg.x, msg.y))
        rospy.loginfo('person (x,y) {}, {}'.format(msg.x, msg.y))
        if msg.x: 
            return True


    with cc:
        Concurrence.add(
            'GET_USER',
            MonitorState(
                '/follow_user/trackedTargets',
                TrackerTarget,
                cond_cb=msg_cb
            ),
            remapping={'invalid': 'aborted',
                       'valid': 'succeeded'}
        )
        Concurrence.add(
            'TIMER',
            SleepState(10)
        )

    fm_sm.userdata.result = 'ok_test'
    with fm_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'ENABLE_TRACKING',
                         'canceled': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'UNDOCK_IF_NEEDED',
            hobbit_move.undock_if_needed(),
            transitions={'succeeded': 'BACK_IF_NEEDED',
                         'aborted': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'BACK_IF_NEEDED',
            hobbit_move.back_if_needed(),
            transitions={'succeeded': 'ROTATE_180',
                         'aborted': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'ROTATE_180',
            hobbit_move.MoveDiscrete(motion='Rotate', value=180),
            transitions={'succeeded': 'ENABLE_TRACKING',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORTED'}
        )
        StateMachine.add_auto(
            'PREPARE_MOVEMENT',
            ServiceState(
                '/obs_nav_mode',
                ObsNavMode
            ),
            connector_outcomes=['succeeded', 'aborted']
        )
        StateMachine.add_auto(
            'SWITCH_VISION',
            ServiceState(
                '/vision_system/navigating',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb
            ),
            connector_outcomes=['succeeded', 'aborted']
        )
        StateMachine.add_auto(
            'ENABLE_TRACKING',
            ServiceState(
                '/follow_user/resume',
                Empty
            ),
            connector_outcomes=['succeeded', 'aborted']
        )
        StateMachine.add(
            'SAY_START',
            speech_output.sayText(
                info='Bitte stell dich vor mich.'),
                #info="Please stand in front of me so i can follow you."),
            transitions={'succeeded': 'GET_USER',
                         'failed': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'GET_USER',
            #cc,
            WaitForMsgState(
                '/trackedTargets',
                TrackerTarget,
                msg_cb=msg_cb,
                timeout=25
            ),
            transitions={'succeeded': 'SAY_FOUND_YOU',
                         'aborted': 'SAY_CANT_SEE'}
        )
        StateMachine.add(
            'SAY_FOUND_YOU',
            speech_output.sayText(
                info="Ich habe dich gefunden werde dir jetzt folgen."),
                #info="Found you. Will start following you."),
            transitions={'succeeded': 'HEAD_DOWN',
                         'failed': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'HEAD_DOWN',
            head_move.MoveTo(pose='down_center'),
            transitions={'succeeded': 'FOLLOW',
                         'aborted': 'LOG_ABORTED'}
        )
        follow_goal=FollowMeGoal()
        follow_goal.command='start'
        StateMachine.add(
            'FOLLOW',
            SimpleActionState(
                'person_following',
                FollowMeAction,
                goal=follow_goal,
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'HEAD_UP',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'HEAD_UP1'}
        )
        StateMachine.add(
            'SAY_CANT_SEE',
            speech_output.sayText(
                info="Ich konnte dich nicht erkennen und bleibe deshalb hier stehen."),
                #info="I did not see you and will stay here."),
            transitions={'succeeded': 'LOG_ABORTED',
                         'failed': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'HEAD_UP',
            head_move.MoveTo(pose='littledown_center'),
            transitions={'succeeded': 'SAY_STOP',
                         'aborted': 'SAY_STOP'}
        )
        StateMachine.add(
            'HEAD_UP1',
            head_move.MoveTo(pose='littledown_center'),
            transitions={'succeeded': 'SAY_STOP1',
                         'aborted': 'SAY_STOP1'}
        )
        StateMachine.add(
            'SAY_STOP',
            speech_output.sayText(
                info="Ich bleibe jetzt stehen."),
                #info="I will stop moving now."),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'failed': 'LOG_ABORTED'}
        )
        StateMachine.add(
            'SAY_STOP1',
            speech_output.sayText(
                info="Ich sehe dich nicht mehr und bleibe deshalb hier stehen."),
                #info="I will stop moving now."),
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

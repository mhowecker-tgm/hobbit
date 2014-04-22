#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'goto'
DEBUG = True
MMUI_IS_DOING_IT = True

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
#import uashh_smach.util as util
import uashh_smach.platform.move_base as move_base

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction, Event
from hobbit_msgs.srv import GetCoordinates
from smach_ros import ActionServerWrapper, IntrospectionServer, \
    ServiceState
from smach import StateMachine, State
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

    def disable(self):
        self.HEADER = ''
        self.OKBLUE = ''
        self.OKGREEN = ''
        self.WARNING = ''
        self.FAIL = ''
        self.ENDC = ''


class Init(State):
    """Class to initialize certain parameters"""
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'canceled'],
            input_keys=['command'], output_keys=['social_role'])
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)
        self.pub_face.publish('EMO_NEUTRAL')

    def execute(self, ud):
        self.pub_face.publish('EMO_NEUTRAL')
        self.pub_head.publish('down')
        self.pub_obstacle.publish('active')
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        return 'succeeded'


class CallCheck(State):
    """
    Class which checks from where the goto scenario was called.
    command: The user pressed the away/break button.
    event: A calendar entry triggered the execution.
    """
    def __init__(self):
        State.__init__(self,
                       outcomes=['voice', 'touch', 'preempted', 'failure'],
                       input_keys=['command', 'parameters'],
                       output_keys=['question', 'timeframe']
                       )

    def execute(self, ud):
        if self.preempt_requested():
            ud.result = String('preempted')
            return 'preempted'
        print(ud.parameters)
        print(ud.command)
        return 'touch'
        if ud.parameters[0].data.lower() == 'command':
            # TODO: Change hardcoded question to one from the translation pack
            pass
        else:
            rospy.loginfo(
                'Unknown type received in GeneralHobbitAction: %s' % ud.command.data)
            return 'failure'


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
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
        ud.result = String('user not detected')
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
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_HAPPY')
        self.pub.publish('Stop')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('user detected')
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
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
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
            outcomes=['succeeded'],
            input_keys=['command'],
            output_keys=['result', 'command']
        )

    def execute(self, ud):
        #ud.result = String('')
        #rospy.sleep(2.0)
        return 'succeeded'


@smach.cb_interface(input_keys=['room_name'])
def set_nav_goal_cb(userdata, request):
    nav_request = GetCoordinates().Request
    nav_request.room_name = String(userdata.room_name)
    return nav_request


def main():
    rospy.init_node(NAME)

    goto_sm = StateMachine(
        outcomes=['succeeded', 'failure', 'preempted'],
        input_keys=['command', 'previous_state', 'parameters'],
        output_keys=['result'])

    goto_sm.userdata.result = String('started')
    goto_sm.userdata.emotion = 'WONDERING'
    goto_sm.userdata.emo_time = 4

    with goto_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'voice': 'EMO_WONDERING',
                         'touch': 'EMO_HAPPY'}
        )
        StateMachine.add(
            'EMO_WONDERING',
            HobbitEmotions.ShowEmotions(),
            transitions={'preempted': 'preempted',
                         'succeeded': 'MMUI_CONFIRM',
                         'aborted': 'SET_FAILURE'}
        )
        goto_sm.userdata.question = String('T_GT_ConfirmGoToPlace')
        StateMachine.add(
            'MMUI_CONFIRM',
            HobbitMMUI.AskYesNo(),
            transitions={'yes': 'SET_NAV_GOAL',
                         'no': 'SET_FAILURE',
                         'aborted': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        if not DEBUG:
            StateMachine.add(
                'SET_NAV_GOAL',
                ServiceState('Hobbit/ObjectService/get_coordinates',
                             GetCoordinates,
                             request_cb=set_nav_goal_cb,
                             input_keys=['room_name']),
                transitions={'preempted': 'preempted',
                             'succeeded': 'MOVE_BASE_GO',
                             'aborted': 'SET_FAILURE'}
            )
            StateMachine.add(
                'MOVE_BASE_GO',
                move_base.MoveBaseState(),
                transitions={'succeeded': 'EMO_HAPPY',
                             'preempted': 'preempted',
                             'aborted': 'SET_FAILURE'}
            )
        else:
            StateMachine.add(
                'SET_NAV_GOAL',
                Dummy(),
                transitions={'succeeded': 'MOVE_BASE_GO'}
            )
            StateMachine.add(
                'MOVE_BASE_GO',
                Dummy(),
                transitions={'succeeded': 'EMO_HAPPY'}
            )
        StateMachine.add(
            'EMO_HAPPY',
            HobbitEmotions.ShowEmotions(),
            transitions={'preempted': 'preempted',
                         'succeeded': 'MMUI_REACHED',
                         'aborted': 'SET_FAILURE'}
        )
        StateMachine.add(
            'MMUI_REACHED',
            HobbitMMUI.ShowInfo(),
            transitions={'succeeded': 'WAIT_FOR_MMUI',
                         'preempted': 'preempted',
                         'aborted': 'SET_FAILURE'}
        )
        StateMachine.add(
            'WAIT_FOR_MMUI',
            HobbitMMUI.WaitforSoundEnd('/Event', Event),
            transitions={'aborted': 'WAIT_FOR_MMUI',
                         'succeeded': 'SET_SUCCESS'}
        )
        StateMachine.add(
            'SET_SUCCESS',
            SetSuccess(),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'CLEAN_UP'}
        )
        StateMachine.add(
            'SET_FAILURE',
            SetFailure(),
            transitions={'succeeded': 'failure',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'CLEAN_UP',
            CleanUp(),
            transitions={'succeeded': 'preempted'})

    asw = ActionServerWrapper(
        'goto',
        GeneralHobbitAction,
        goto_sm,
        ['succeeded'], ['failure'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command',
                        'previous_state': 'previous_state',
                        'parameters': 'parameters'}
    )

    sis = IntrospectionServer('smach_server', goto_sm, '/HOBBIT/goto_sm_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

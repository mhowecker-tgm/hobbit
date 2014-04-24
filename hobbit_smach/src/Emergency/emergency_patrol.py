#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'reminder'
DEBUG = True

import roslib
roslib.load_manifest(PKG)
import rospy
#import smach
#import uashh_smach.util as util
import uashh_smach.platform.move_base as move_base

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction,\
    GeneralHobbitGoal, LocateUserAction, LocateUserGoal,\
    Event
from hobbit_msgs.srv import GetCoordinates
from smach_ros import ActionServerWrapper, \
    IntrospectionServer, ServiceState, SimpleActionState
from smach import StateMachine, State, cb_interface
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

    def execute(self, ud):
        self.pub_head.publish('down')
        self.pub_obstacle.publish('active')
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        return 'succeeded'


class Dummy(State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'failed'],
            input_keys=['command'],
            output_keys=['result', 'command']
        )
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
        #ud.result = String('')
        #rospy.sleep(2.0)
        return 'succeeded'


class CallCheck(State):
    """
    Class which checks from where the reminder scenario was called.
    command: The user pressed the away/break button.
    event: A calendar entry triggered the execution.
    """
    def __init__(self):
        State.__init__(self,
                       outcomes=['command', 'event', 'preempted', 'failure'],
                       input_keys=['command', 'parameters'],
                       output_keys=['question', 'timeframe']
                       )

    def execute(self, ud):
        if self.preempt_requested():
            ud.result = String('preempted')
            return 'preempted'
        print(ud.parameters)
        print(ud.command)
        if ud.parameters[0].data.lower() == 'command':
            # TODO: Change hardcoded question to one from the translation pack
            ud.question = 'Do you want to see the appointments \
            for the next day?'
            ud.timeframe = '24:00'
            return 'command'
        elif ud.parameters[0].data.lower() == 'event':
            ud.timeframe = '03:00'
            return 'event'
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
        ud.result = String('')
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
        ud.result = String('success')
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


class TimeCheck(State):
    """
    Class to check if we were activated during the night.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['day', 'night', 'canceled'],
            input_keys=['night'],
            output_keys=['night'])
        self.sleep_time = '22:00'
        self.wakeup_time = '06:30'

    def execute(self, ud):
        if rospy.has_param('sleep_time') and rospy.has_param('wakeup_time'):
            self.sleep_time = rospy.get_param('sleep_time')
            self.wakeup_time = rospy.get_param('wakeup_time')

        wake = self.wakeup_time.split(':')
        sleep = self.sleep_time.split(':')
        now = datetime.now()
        if time(int(wake[0]), int(wake[1]))\
                <= now.time()\
                <= time(int(sleep[0]), int(sleep[1])):
            print('yes, within the interval')
            #ud.night = False
            return 'day'
        else:
            #ud.night = True
            return 'night'


class ActivateLights(State):
    """
    Class to activate the ligths in the users appartement during the night
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        # TODO: Call tcp client to communicate with the light switch
        return 'succeeded'


@cb_interface(input_keys=['room_name'])
def set_nav_goal_cb(userdata, request):
    nav_request = GetCoordinates().Request
    nav_request.room_name = String(userdata.room_name)
    return nav_request


def main():
    rospy.init_node(NAME)

    empa_sm = StateMachine(
        outcomes=['succeeded', 'failure', 'preempted'],
        input_keys=['command', 'previous_state', 'parameters'],
        output_keys=['result'])

    empa_sm.userdata.result = String('started')
    empa_sm.userdata.detection = False

    with empa_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'LOCATE_USER',
                         'canceled': 'CLEAN_UP'})
        if DEBUG:
            StateMachine.add(
                'LOCATE_USER',
                Dummy(),
                transitions={'succeeded': 'APPROACH_USER',
                             'failed': 'CLEAN_UP',
                             'preempted': 'preempted'}
            )
            StateMachine.add(
                'APPROACH_USER',
                Dummy(),
                transitions={'succeeded': 'SAY_CanIDoAnythingForYou',
                             'failed': 'CLEAN_UP',
                             'preempted': 'preempted'}
            )
        else:
            StateMachine.add(
                'LOCATE_USER',
                SimpleActionState(
                    'locate_user',
                    LocateUserAction,
                    goal=LocateUserGoal(command=String('locateUser'))),
                transitions={'succeeded': '',
                             'aborted': 'CLEAN_UP',
                             'preempted': 'preempted'}
            )
            StateMachine.add(
                'APPROACH_USER',
                SimpleActionState(
                    'approach_user',
                    GeneralHobbitAction,
                    goal=GeneralHobbitGoal(command=String('approach'))),
                transitions={'succeeded': 'SAY_CanIDoAnythingForYou',
                             'aborted': 'CLEAN_UP',
                             'preempted': 'preempted'}
            )
        StateMachine.add(
            'SAY_CanIDoAnythingForYou',
            HobbitMMUI.AskYesNo('T_CanIDoAnythingForYou'),
            transitions={'yes': 'EMO_VHAPPY',
                         'no': 'MMUI_MAIN_MENU',
                         'timeout': 'SAY_CanIDoAnythingForYou',
                         '3times': 'CALL_USER',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'EMO_VHAPPY',
            HobbitEmotions.ShowEmotions(emotion='VHAPPY', emo_time=4),
            transitions={'succeeded': 'SAY_WhatCanIDoForYou',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SAY_WhatCanIDoForYou',
            HobbitMMUI.ShowInfo('T_HM_WhatCanIDoForYou'),
            transitions={'succeeded': 'WAIT_FOR_MMUI',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'WAIT_FOR_MMUI',
            HobbitMMUI.WaitforSoundEnd('/Event', Event),
            transitions={'succeeded': 'MMUI_MAIN_MENU',
                         'aborted': 'WAIT_FOR_MMUI'}
        )
        if not DEBUG:
            StateMachine.add(
                '',
                ServiceState(
                    'Hobbit/ObjectService/get_coordinates',
                    GetCoordinates,
                    request_cb=set_nav_goal_cb,
                    input_keys=['bathroom_door']))
            StateMachine.add(
                'MOVE_BASE',
                move_base.MoveBaseState())
        else:
            StateMachine.add(
                'CALL_USER',
                Dummy(),
                transitions={'succeeded': 'EMO_NEUTRAL',
                             'failed': 'EMERGENCY_CALL',
                             'preempted': 'preempted'}
            )
        StateMachine.add(
            'EMERGENCY_CALL',
            Dummy(),
            #SimpleActionState(),
            transitions={'succeeded': 'EMO_NEUTRAL',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'EMO_NEUTRAL',
            HobbitEmotions.ShowEmotions(emotion='NEUTRAL', emo_time=0),
            transitions={'succeeded': 'MMUI_MAIN_MENU',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'MMUI_MAIN_MENU',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'succeeded': 'SET_SUCCESS',
                         'preempted': 'preempted',
                         'failed': 'SET_FAILURE'}
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
        'emergency_patrol',
        GeneralHobbitAction,
        empa_sm,
        ['succeeded'], ['failure'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command',
                        'previous_state': 'previous_state',
                        'parameters': 'parameters'}
    )

    sis = IntrospectionServer('smach_server', empa_sm, '/HOBBIT/EMPA_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

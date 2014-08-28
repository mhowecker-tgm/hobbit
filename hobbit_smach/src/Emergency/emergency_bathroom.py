#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'emergency_bathroom'
DEBUG = False

import roslib
roslib.load_manifest(PKG)
import rospy

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction
from smach_ros import ActionServerWrapper, IntrospectionServer
from smach import StateMachine, State
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
from datetime import datetime, time
import hobbit_smach.sos_call_import as sos_call
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.aal_lights_import as aal_lights


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
        # ud.result = String('')
        # rospy.sleep(2.0)
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
                'Unknown type in GeneralHobbitAction: %s' % ud.command.data)
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
            # ud.night = False
            return 'day'
        else:
            # ud.night = True
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


def main():
    rospy.init_node(NAME)

    embr_sm = StateMachine(
        outcomes=['succeeded', 'failure', 'preempted'],
        input_keys=['command', 'previous_state', 'parameters'],
        output_keys=['result'])

    embr_sm.userdata.result = String('started')
    embr_sm.userdata.detection = False

    with embr_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'TIME_CHECK',
                         'canceled': 'CLEAN_UP'})
        StateMachine.add(
            'TIME_CHECK',
            TimeCheck(),
            transitions={'day': 'MMUI_SAY_YesIAmComing',
                         'night': 'ACTIVATE_LIGHTS',
                         'canceled': 'CLEAN_UP',
                         }
        )
        StateMachine.add(
            'ACTIVATE_LIGHTS',
            aal_lights.SetLights(active=True),
            transitions={'succeeded': 'MMUI_SAY_YesIAmComing',
                         'failed': 'MMUI_SAY_YesIAmComing',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'MMUI_SAY_YesIAmComing',
            speech_output.sayText(info='T_HM_YesIAmComing'),
            transitions={'succeeded': 'MOVE_TO_BATHROOM',
                         'preempted': 'preempted',
                         'failed': 'SET_FAILURE'}
        )
        if not DEBUG:
            StateMachine.add(
                'MOVE_TO_BATHROOM',
                hobbit_move.goToPosition(frame='/map', room='corridor', place='bathroom'),
                transitions={'aborted': 'SET_FAILURE',
                             'succeeded': 'MMUI_CONFIRM_DoYouNeedHelp'}
            )
        else:
            StateMachine.add(
                'MOVE_TO_BATHROOM',
                Dummy(),
                transitions={'succeeded': 'MMUI_CONFIRM_DoYouNeedHelp',
                             'failed': 'SET_FAILURE',
                             'preempted': 'preempted'}
            )
        # TODO: Change volume to maximum
        StateMachine.add(
            'MMUI_CONFIRM_DoYouNeedHelp',
            HobbitMMUI.AskYesNo(question='T_HM_DoYouNeedHelp'),
            transitions={'yes': 'MMUI_CONFIRM_ShallICallHelp',
                         'no': 'EMO_NEUTRAL1',
                         'timeout': 'MMUI_CONFIRM_ShallICallHelp',
                         '3times': 'EMERGENCY_CALL',
                         'failed': 'SET_FAILURE'}
        )
        # TODO: Change volume to maximum
        StateMachine.add(
            'MMUI_CONFIRM_ShallICallHelp',
            HobbitMMUI.AskYesNo(question='T_HM_StayCalmShallICallHelp'),
            transitions={'yes': 'EMERGENCY_CALL',
                         'no': 'EMO_NEUTRAL',
                         'timeout': 'MMUI_CONFIRM_ShallICallHelp',
                         '3times': 'EMERGENCY_CALL',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'EMERGENCY_CALL',
            sos_call.get_call_sos_simple(),
            transitions={'succeeded': 'MMUI_MAIN_MENU_1',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted',
                         'aborted': 'SET_SUCCESS'}
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
            transitions={'succeeded': 'MMUI_SAY_YouCanGetHelpAnytime',
                         'preempted': 'preempted',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'SAY_NEUTRAL',
            speech_output.emo_say_something(
                emo='NEUTRAL',
                time=0,
                text='T_HM_YouCanGetHelpAnytime'
            ),
            transitions={'succeeded': 'SET_SUCCESS',
                         'aborted': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'MMUI_SAY_YouCanGetHelpAnytime',
            speech_output.sayText(info='T_HM_YouCanGetHelpAnytime'),
            transitions={'succeeded': 'MMUI_MAIN_MENU_1',
                         'preempted': 'preempted',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'MMUI_MAIN_MENU_1',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'succeeded': 'SET_SUCCESS',
                         'preempted': 'preempted',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'EMO_NEUTRAL1',
            HobbitEmotions.ShowEmotions(emotion='NEUTRAL', emo_time=0),
            transitions={'succeeded': 'succeeded',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
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
        'emergency_bathroom',
        GeneralHobbitAction,
        embr_sm,
        ['succeeded'], ['failure'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command',
                        'previous_state': 'previous_state',
                        'parameters': 'parameters'}
    )

    sis = IntrospectionServer('smach_server', embr_sm, '/HOBBIT/EMBR_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

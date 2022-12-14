#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'away_break'

import rospy
import smach

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction, Event
from hobbit_msgs.srv import GetCoordinates, SetAwayStateRequest, SetAwayState
from smach_ros import ActionServerWrapper, IntrospectionServer, ServiceState
from smach import StateMachine, State, Sequence
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.hobbit_cronjobs_import as cronjobs
import hobbit_smach.logging_import as log
import hobbit_smach.recharge_import as recharge


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


class EventCheck(State):
    """
    Class which checks from where the away scenario was called.
    sleep: The user takes a nap.
    away: The user is leaving the apartment.
    """
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'preempted', 'failed'],
                       input_keys=['command', 'parameters'],
                       output_keys=['event', 'timeframe']
                       )

    def execute(self, ud):
        if self.preempt_requested():
            ud.result = String('preempted')
            return 'preempted'
        print(ud.command)
        if ud.command.data == 'sleep':
            ud.event = 'sleep'
            return 'succeeded'
        elif ud.command.data == 'away':
            ud.event = 'away'
            return 'succeeded'
        else:
            rospy.loginfo(
                'Unknown type in GeneralHobbitAction: %s' % ud.command.data)
            return 'failure'


class ParameterCheck(State):
    """
    Class which checks from where the away scenario was called.
    command: The user pressed the away/break button.
    event: A calendar entry triggered the execution.
    """
    def __init__(self):
        State.__init__(self,
                       outcomes=['yes', 'no', 'preempted', 'failure'],
                       input_keys=['command', 'parameters'],
                       output_keys=['question', 'interval', 'timeframe']
                       )

    def execute(self, ud):
        if self.preempt_requested():
            ud.result = String('preempted')
            return 'preempted'
        print(ud.command)
        for i in ud.parameters:
            ud.interval = int(i.data)
            if int(i.data) < 6:
                ud.timeframe = '03:00'
                return 'no'
            elif int(i.data) == 6:
                ud.timeframe = '07:00'
                return 'no'
            elif int(i.data) > 6:
                ud.timeframe = '24:00'
                return 'yes'
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
        ud.result = String('User is stays in.')
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
        ud.result = String('User sleeping/gone out')
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
    """Class to initialize certain parameters"""
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed']
        )

    def execute(self, ud):
        return 'succeeded'
        return 'failed'


@smach.cb_interface(input_keys=['room_name'])
def set_nav_goal_cb(userdata, request):
    nav_request = GetCoordinates().Request
    nav_request.room_name = String(userdata.room_name)
    return nav_request

def resp_cb(userdata, response):
    rospy.loginfo(response)
    if response.result:
        return 'succeeded'
    else:
        return 'aborted'

def main():
    rospy.init_node(NAME)

    away_sm = StateMachine(
        outcomes=['succeeded', 'failure', 'preempted'],
        input_keys=['command', 'previous_state', 'parameters'],
        output_keys=['result'])

    seq1 = Sequence(
        outcomes=['succeeded', 'failed', 'preempted'],
        connector_outcome='succeeded',
        input_keys=['interval']
    )
    seq2 = Sequence(
        outcomes=['succeeded', 'failed', 'preempted'],
        connector_outcome='succeeded',
        input_keys=['interval']
    )

    away_sm.userdata.result = String('started')
    away_sm.userdata.emotion = 'NEUTRAL'
    away_sm.userdata.emo_time = 4

    with away_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'EVENT_CHECK',
                         'canceled': 'CLEAN_UP'}
        )
        StateMachine.add(
            'EVENT_CHECK',
            EventCheck(),
            transitions={'succeeded': 'UNTIL_MORNING',
                         'failed': 'CLEAN_UP'}
        )
        StateMachine.add(
            'UNTIL_MORNING',
            ParameterCheck(),
            transitions={'preempted': 'LOG_PREEMPT',
                         'yes': 'ASK_Y_N_ACTIVITIES',
                         'no': 'ASK_Y_N_REMIND_SWITCH_OFF',
                         'failure': 'SET_FAILURE'}
        )
        StateMachine.add(
            'ASK_Y_N_ACTIVITIES',
            HobbitMMUI.AskYesNo(question='T_BR_RemindYouTomorrowActivities'),
            transitions={'yes': 'SEQ1',
                         'no': 'ASK_Y_N_REMIND_SWITCH_OFF',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'SET_FAILURE',
                         'timeout': 'ASK_Y_N_ACTIVITIES',
                         '3times': 'SET_FAILURE'}
        )
        with seq1:
            Sequence.add(
                'EMO_HAPPY',
                HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=4))
            # TODO: Replace Dummy stub with real function
            #Sequence.add('MMUI_SHOW_CAL', HobbitMMUI.ShowCalendar())
            Sequence.add('MMUI_SHOW_CAL', Dummy())
            Sequence.add(
                'CONFIRM_REMINDERS_TOMORROW',
                #HobbitMMUI.ShowInfo(info='These are the appointments')
                HobbitMMUI.ConfirmInfo(info='T_BR_RemindersTomorrow')
            )
        StateMachine.add(
            'ASK_Y_N_REMIND_SWITCH_OFF',
            HobbitMMUI.AskYesNo(question='T_BR_RemindYouSwitchOff'),
            transitions={'yes': 'CONFIRM_SWITCH_OFF_OVEN',
                         'no': 'SEQ2',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'SET_FAILURE',
                         'timeout': 'ASK_Y_N_ACTIVITIES',
                         '3times': 'SET_FAILURE'}
        )
        StateMachine.add(
            'CONFIRM_SWITCH_OFF_OVEN',
            HobbitMMUI.ConfirmInfo(info='T_BR_SwitchOffOven'),
            transitions={'succeeded': 'CONFIRM_CLOSE_WINDOWS',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'CONFIRM_CLOSE_WINDOWS',
            HobbitMMUI.ConfirmInfo(info='T_BR_CloseAllWindows'),
            transitions={'succeeded': 'CONFIRM_BRING_KEYS',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'CONFIRM_BRING_KEYS',
            HobbitMMUI.ConfirmInfo(info='T_BR_BringYourKeys'),
            transitions={'succeeded': 'SEQ2',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'SEQ1',
            seq1,
            transitions={'succeeded': 'ASK_Y_N_REMIND_SWITCH_OFF',
                         'failed': 'SET_FAILURE',
                         'preempted': 'LOG_PREEMPT'}
        )
        with seq2:
            Sequence.add(
                'EMO_HAPPY',
                HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=4))
            Sequence.add(
                'SAY_GOOD_BYE_SLEEP',
                speech_output.sayText(info='T_BR_Goodbye'),
                transitions={'failed': 'failed'})
            Sequence.add(
                'RECHARGE',
                recharge.getRecharge(),
                transitions={'aborted': 'failed'}
            )
            seq2.userdata.text = 'Tell me when you are back/awake again.'
            Sequence.add(
                'MMUI_SHOW_MENU_ME_BACK_AWAKE',
                HobbitMMUI.ShowMenu(menu='AWAY'))
            Sequence.add(
                'SET_CRONJOB',
                cronjobs.SleepAway(),
                #transitions={'aborted': 'failed'}
                transitions={'aborted': 'succeeded'}
            )
        StateMachine.add(
            'SEQ2',
            seq2,
            transitions={'succeeded': 'SET_SUCCESS',
                         'failed': 'SET_FAILURE',
                         'preempted': 'LOG_PREEMPT'}
        )

        StateMachine.add(
            'SET_SUCCESS',
            SetSuccess(),
            transitions={'succeeded': 'USER_IS_AWAY',
                         'preempted': 'CLEAN_UP'}
        )
        StateMachine.add(
            'USER_IS_AWAY',
            ServiceState('/user/set_away_state',
                         SetAwayState,
                         request=SetAwayStateRequest(state=True),
                         response_cb=resp_cb),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'aborted': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'SET_FAILURE',
            SetFailure(),
            transitions={'succeeded': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'CLEAN_UP',
            CleanUp(),
            transitions={'succeeded': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='away/sleep'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='away/sleep'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORTED',
            log.DoLogAborted(scenario='away/sleep'),
            transitions={'succeeded': 'failure'}
        )
    asw = ActionServerWrapper(
        'away',
        GeneralHobbitAction,
        away_sm,
        ['succeeded'], ['failure'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command',
                        'previous_state': 'previous_state',
                        'parameters': 'parameters'}
    )

    #sis = IntrospectionServer('smach_server', away_sm, '/HOBBIT/away_sm_ROOT')
    #sis.start()
    asw.run_server()
    rospy.spin()
    #sis.stop()

if __name__ == '__main__':
    main()

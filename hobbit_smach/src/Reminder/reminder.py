#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'reminder'

import rospy
from uashh_smach.util import SleepState

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction,\
    ApproachUserAction, ApproachUserGoal
from smach_ros import ActionServerWrapper, \
    SimpleActionState, IntrospectionServer
from smach import StateMachine, State
from hobbit_user_interaction import HobbitMMUI
import hobbit_smach.locate_user_simple_import as locate_user
import hobbit_smach.logging_import as log


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


def main():
    rospy.init_node(NAME)

    rem_sm = StateMachine(
        outcomes=['succeeded', 'failure', 'preempted'],
        input_keys=['command', 'previous_state', 'parameters'],
        output_keys=['result'])

    rem_sm.userdata.result = String('started')
    rem_sm.userdata.detection = False
    rem_sm.userdata.categories = ['meeting']
    rem_sm.userdata.timeframe = '23:59'

    with rem_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'CALL_CHECK',
                         'canceled': 'CLEAN_UP'})
        StateMachine.add(
            'CALL_CHECK',
            CallCheck(),
            transitions={'preempted': 'CLEAN_UP',
                         'failure': 'SET_FAILURE',
                         'command': 'LOCATE_USER',
                         'event': 'LOCATE_USER'}
        )
        StateMachine.add(
            'MMUI_CONFIRM',
            HobbitMMUI.ConfirmInfo(info='T_RE_MedicationReminder'),
            #HobbitMMUI.AskYesNo(question='T_RE_MedicationReminder'),
            transitions={'succeeded': 'MMUI_SHOW_CAL',
                         'failed': 'SET_FAILURE'}

        )
        StateMachine.add(
            'MMUI_SHOW_CAL',
            HobbitMMUI.ShowCalendar(
                timeframe = rem_sm.userdata.timeframe,
                categories = rem_sm.userdata.categories
            ),
            transitions={'preempted': 'CLEAN_UP',
                         'succeeded': 'SET_SUCCESS',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'LOCATE_USER',
            locate_user.get_detect_user(),
            {'succeeded': 'MMUI_CONFIRM',
             'aborted': 'CLEAN_UP',
             'preempted': 'CLEAN_UP'})
        StateMachine.add(
            'SET_SUCCESS',
            SetSuccess(),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'preempted': 'CLEAN_UP'}
        )
        StateMachine.add(
            'SET_FAILURE',
            SetFailure(),
            transitions={'succeeded': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'CLEAN_UP',
            CleanUp(),
            transitions={'succeeded': 'LOG_PREEMPT'})
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Reminder'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Reminder'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORT',
            log.DoLogAborted(scenario='Reminder'),
            transitions={'succeeded': 'failure'}
        )

    asw = ActionServerWrapper(
        'reminder',
        GeneralHobbitAction,
        rem_sm,
        ['succeeded'], ['failure'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command',
                        'previous_state': 'previous_state',
                        'parameters': 'parameters'}
    )

    #sis = IntrospectionServer('smach_server', rem_sm, '/HOBBIT/REM_SM_ROOT')
    #sis.start()
    asw.run_server()
    rospy.spin()
    #sis.stop()

if __name__ == '__main__':
    main()

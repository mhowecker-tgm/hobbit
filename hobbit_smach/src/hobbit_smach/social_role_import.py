#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'SocialRole'

import rospy
import random

from std_msgs.msg import String
from smach import State, StateMachine
from hobbit_msgs import MMUIInterface as MMUI
from hobbit_msgs.srv import GetCoordinates, GetCoordinatesRequest
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
import hobbit_smach.sos_call_import as sos_call
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.locate_user_import as locate_user
import hobbit_smach.logging_import as log


class GetSocialRole(State):
    """
    Load social role parameter from ROS parameter server and
    return it as outcome
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['butler', 'tool', 'companion', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if rospy.has_param('social_role'):
            social_role = rospy.get_param('social_role')
        else:
            social_role = 2
        if social_role == 0:
            return 'tool'
        elif 1 <= social_role <= 3:
            return 'butler'
        else:
            return 'companion'


class SetSocialRole(State):
    """
    Load social role parameter from ROS parameter server and
    change its value +/- 1
    """

    def __init__(self, change=0):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )
        self.change = change

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if rospy.has_param('social_role'):
            social_role = rospy.get_param('social_role')
            social_role += self.change
            rospy.set_param('social_role', social_role)
            return 'succeeded'
        else:
            return 'aborted'


class GetSocialPose(State):
    """
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['room_name'],
            output_keys=['x', 'y', 'yaw']
        )

    def _getGoal(self, room, place):
        rospy.wait_for_service('get_coordinates',
                               timeout=5)
        serv = rospy.ServiceProxy(
            'get_coordinates',
            GetCoordinates,
            persistent=False
        )
        req = GetCoordinatesRequest(String(room), String(place))
        try:
            resp = serv(req)
        # print(resp.pose.x, resp.pose.y, resp.pose.theta)
            return (resp.pose.x, resp.pose.y, resp.pose.theta)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return (0.0, 0.0, 0.0)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.x, ud.y, ud.yaw = self._getGoal(ud.room_name, 'center')
        return 'succeeded'


class Games(State):
    """
    Switch to the games menu on the MMUI. Wrapped in a SMACH state.
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.GoToMenu('M_GAMES')
        print(resp)
        return 'succeeded'
        return 'failed'


class CheckHelpAccepted(State):
    """
    Check the userdata key
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['help_accepted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if ud.help_accepted:
            return 'succeeded'
        else:
            return 'aborted'


class RandomMenu(State):
    """
    Switch to a randomly chosen menu on the MMUI.
    Wrapped in a SMACH state.
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )
        self.menues = [
            'M_Audio, M_Entertain', 'M_Game',
            'M_Memory', 'M_Solitaire', 'M_Chess',
            'M_Simon', 'M_Sudoku', 'M_Muehle',
            'M_eBook', 'M_Picture', 'M_Internet',
            'M_Social']

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        menu = random.choice(self.menues)
        mmui = MMUI.MMUIInterface()
        resp = mmui.GoToMenu(menu)
        if resp:
            return 'succeeded'
        else:
            return 'aborted'


def get_surprise():
    """
    Return a SMACH Statemachine for surprise behaviour.
    """

    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted']
    )

    def msg_cb(msg, ud):
        rospy.loginfo('Waiting for back')
        rospy.loginfo(str(msg.event))
        if msg.event.lower() == 'P_E_BACK'.lower():
            return True
        else:
            return False

    with sm:
        StateMachine.add(
            'RND_MENU',
            RandomMenu(),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'aborted': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Surprise'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Surprise'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORT',
            log.DoLogAborted(scenario='Surprise'),
            transitions={'succeeded': 'aborted'}
        )
    return sm


def get_social_role_change():
    """
    Return a SMACH Statemachine that asks the user if she/he wants to change
    the robots behaviour and social role.
    """
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted']
    )

    with sm:
        StateMachine.add(
            'DETECT_USER',
            locate_user.get_detect_user(),
            transitions={'succeeded': 'SAY_OBTRUSIVE',
                         'aborted': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'SAY_OBTRUSIVE',
            HobbitMMUI.AskYesNo(
                question='T_SR_Obtrusive'),
            transitions={'yes': 'SAD',
                         'no': 'HAPPY',
                         'timeout': 'SAY_OBTRUSIVE',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'SAD'}
        )
        StateMachine.add_auto(
            'HAPPY',
            HobbitEmotions.ShowEmotions(
                emotion='HAPPY',
                emo_time=4
            ),
            connector_outcomes={'succeeded', 'failed', 'preempted'}
        )
        StateMachine.add(
            'SR_UP',
            SetSocialRole(change=1),
            transitions={'succeeded': 'SAY_SAFE_SECURE',
                         'aborted': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'SAD',
            HobbitEmotions.ShowEmotions(
                emotion='SAD',
                emo_time=4
            ),
            connector_outcomes={'succeeded', 'failed', 'preempted'}
        )
        StateMachine.add(
            'SR_DOWN',
            SetSocialRole(change=-1),
            transitions={'succeeded': 'SAY_SAFE_SECURE',
                         'aborted': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'SAY_SAFE_SECURE',
            HobbitMMUI.AskYesNo(
                question='T_SR_Safeandsupported'),
            transitions={'yes': 'VERY_HAPPY_THANKS',
                         'no': 'SAD_THANKS',
                         'timeout': 'SAY_SAFE_SECURE',
                         '3times': 'USER_NOT_RESPONDING',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'SAD_THANKS'}
        )
        # social_role =- 1
        StateMachine.add(
            'SAD_THANKS',
            speech_output.emo_say_something(
                emo='SAD',
                time=4,
                text='T_SR_Thankyou'
            ),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'aborted': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        # social_role =- 1
        StateMachine.add(
            'VERY_HAPPY_THANKS',
            speech_output.emo_say_something(
                emo='VERY_HAPPY',
                time=4,
                text='T_SR_Thankyou'
            ),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'aborted': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'USER_NOT_RESPONDING',
            sos_call.get_call_sos_simple(),
            transitions={'succeeded': 'LOG_ABORTED',
                         'failed': 'LOG_ABORTED',
                         'aborted': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogPreempt(scenario='Social Role'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Social Role'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORTED',
            log.DoLogPreempt(scenario='Social Role'),
            transitions={'succeeded': 'aborted'}
        )
    return sm


def get_reward_muc():
    """
    Return a SMACH Statemachine Says something nice. With MUC enabled
    """
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted']
    )

    with sm:
        StateMachine.add(
            'REWARD',
            speech_output.emo_say_something(
                emo='HAPPY',
                time=4,
                text='T_RW_YouAreWelcome'
            ),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'aborted': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Reward MUC'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Reward MUC'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORTED',
            log.DoLogAborted(scenario='Reward MUC'),
            transitions={'succeeded': 'aborted'}
        )
    return sm


def get_reward():
    """
    Return a SMACH Statemachine Says something nice. Without MUC enabled
    """
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted']
    )

    with sm:
        StateMachine.add(
            'REWARD',
            speech_output.emo_say_something(
                emo='NEUTRAL',
                time=4,
                text='T_AnythingElse'
            ),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'aborted': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Reward'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Reward'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORTED',
            log.DoLogAborted(scenario='Reward'),
            transitions={'succeeded': 'aborted'}
        )
    return sm

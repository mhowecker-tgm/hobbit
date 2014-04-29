#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'

import roslib
roslib.load_manifest(PKG)
import rospy

from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
import hobbit_smach.hobbit_move_import as hobbit_move

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction, Event
from smach_ros import ActionServerWrapper, \
    IntrospectionServer
from smach import StateMachine, State, Sequence


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

    """
    Class to initialize certain parameters
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            output_keys=['social_role'])

    def execute(self, ud):
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        else:
            pass
        return 'succeeded'


class CleanUp(State):

    """
    Class for setting the result message and clean up persistent variables
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['command', 'visited_places'],
            output_keys=['result', 'command', 'visited_places'])
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
        ud.visited_places = []
        ud.result = String('sos call finished')
        return 'succeeded'


class SetSuccess(State):

    """
    Class for setting the success message in the actionlib result and clean
    up of persistent variables
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            output_keys=['result', 'visited_places'])
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        ud.visited_places = []
        self.pub_face.publish('EMO_HAPPY')
        self.pub.publish('Stop')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('sos call finished')
        return 'succeeded'


class Dummy(State):

    """
    Class for setting the result message and clean up persistent variables
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'],
            input_keys=['command'],
            output_keys=['result', 'command']
        )

    def execute(self, ud):
        return 'succeeded'


class CheckCaller(State):

    """
    What emergency called SOS?
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted', 'move_to_dock'],
            input_keys=['parameters']
        )

    def execute(self, ud):
        if ud.parameters[0].data == 'user_not_detected':
            return 'move_to_dock'
        else:
            return 'succeeded'


class FailCounter(State):

    """
    Class for setting the result message and clean up persistent variables
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['fail1', 'fail2', 'fail3', 'loop'],
            input_keys=['parameters'],
            output_keys=['result', 'command']
        )
        self.loop = False
        self.fail = 1

    def execute(self, ud):
        if self.fail == 3:
            self.fail = 1
            if ud.parameters[0].data == 'user_initiated':
                print(bcolors.OKGREEN +
                      'user_initiated' +
                      bcolors.ENDC)
                if not self.loop:
                    self.loop = True
                    return 'loop'
                else:
                    self.loop = False
                    return 'fail3'
            else:
                return 'fail3'
        elif self.fail == 2:
            self.fail += 1
            return 'fail2'
        elif self.fail == 1:
            self.fail += 1
            return 'fail1'
        return 'fail3'


def callstate_cb(msg, ud):
    ud.call_state = msg.command
    print(msg.command)
    if msg.event == 'E_CALLCONFIRMED':
        return True
    else:
        return False


def get_call_sos():
    sos_sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command', 'parameters'],
        output_keys=['result'])

    sos_sm.userdata.result = String('started')

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'WAIT_FOR_MMUI',
            HobbitMMUI.WaitforSoundEnd('/Event', Event),
            transitions={'aborted': 'WAIT_FOR_MMUI'})
        Sequence.add('START_CALL', HobbitMMUI.CallEmergency())
        Sequence.add(
            'CHECK_CALL_STATE',
            HobbitMMUI.WaitforConfirmedCall(
                '/Event', Event, output_keys=['call_state']),
            transitions={'aborted': 'CHECK_CALL_STATE'}
        )
        Sequence.add(
            'CALL_DECISION',
            HobbitMMUI.CallDecison(),
            transitions={'stop': 'END_CALL',
                         'ended': 'failed'}
        )
        Sequence.add(
            'CHECK_CALL_ENDED',
            HobbitMMUI.WaitforEndedCall(
                '/Event', Event, output_keys=['call_state']),
            transitions={'aborted': 'CHECK_CALL_ENDED'}
        )
        Sequence.add('END_CALL', HobbitMMUI.CallEnded())

    with sos_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'SAY_T_HM_IWillCallFirstPersonInList',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'CHECK_CALLER',
            CheckCaller(),
            transitions={'succeeded': 'MMUI_MAIN_MENU',
                         'failed': 'aborted',
                         'preempted': 'preempted',
                         'move_to_dock': 'MOVE_TO_DOCK'}
        )
        StateMachine.add(
            'SEQ',
            seq,
            transitions={'succeeded': 'SAY_IAmWithYou',
                         'failed': 'FAIL_COUNTER'}
        )
        StateMachine.add(
            'MMUI_MAIN_MENU',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'succeeded': 'succeeded',
                         'failed': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'FAIL_COUNTER',
            FailCounter(),
            transitions={'loop': 'SAY_T_HM_NoAnswersIWillCallFirstPersonAgain',
                         'fail1': 'SAY_T_HM_IWillCallSecondPersonInList',
                         'fail2': 'SAY_T_HM_IWillCallThirdPersonInList',
                         'fail3': 'SAY_NoAnswersSayOrPressHelpAgainToTryAgain'}
        )
        StateMachine.add(
            'SAY_T_HM_IWillCallFirstPersonInList',
            HobbitMMUI.ShowInfo(info='IWillCallFirstPersonInList'),
            transitions={'succeeded': 'SEQ',
                         'failed': 'SEQ',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SAY_T_HM_IWillCallSecondPersonInList',
            HobbitMMUI.ShowInfo(info='IWillCallSecondPersonInList'),
            transitions={'succeeded': 'SEQ',
                         'failed': 'SEQ',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SAY_T_HM_IWillCallThirdPersonInList',
            HobbitMMUI.ShowInfo(info='IWillCallThirdPersonInList'),
            transitions={'succeeded': 'SEQ',
                         'failed': 'SEQ',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SAY_T_HM_NoAnswersIWillCallFirstPersonAgain',
            HobbitMMUI.ShowInfo(info='NoAnswersIWillCallFirstPersonAgain'),
            transitions={'succeeded': 'SEQ',
                         'failed': 'SEQ',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SAY_NoAnswersSayOrPressHelpAgainToTryAgain',
            HobbitMMUI.ShowInfo(info='NoAnswersSayOrPressHelpAgainToTryAgain'),
            transitions={'succeeded': 'WAIT_FOR_MMUI',
                         'failed': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'WAIT_FOR_MMUI',
            HobbitMMUI.WaitforSoundEnd('/Event', Event),
            transitions={'aborted': 'WAIT_FOR_MMUI',
                         'succeeded': 'SAY_IAmWithYou'}
        )
        StateMachine.add(
            'EMO_NEUTRAL',
            HobbitEmotions.ShowEmotions(emotion='NEUTRAL', emo_time=4),
            transitions={'succeeded': 'END_CALL',
                         'failed': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'END_CALL',
            HobbitMMUI.CallEnded(),
            transitions={'succeeded': 'CHECK_CALLER',
                         'failed': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'MOVE_TO_DOCK',
            hobbit_move.goToPosition(room=None, place='dock'),
            transitions={'succeeded': 'MMUI_MAIN_MENU',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SAY_IAmWithYou',
            HobbitMMUI.ShowInfo(info='IAmWithYou'),
            transitions={'succeeded': 'EMO_NEUTRAL',
                         'preempted': 'preempted',
                         'failed': 'EMO_NEUTRAL'}
        )
    return sos_sm

if __name__ == '__main__':
    print('You should import this file, not execute it.')

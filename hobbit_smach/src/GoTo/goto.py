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
#import uashh_smach.platform.move_base as move_base
import hobbit_smach.hobbit_move as hobbit_move

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction, Event
from hobbit_msgs.srv import GetCoordinates
from smach_ros import ActionServerWrapper, IntrospectionServer
from smach import StateMachine, State, Sequence
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
            outcomes=['succeeded', 'failure'],
            output_keys=['social_role', 'block_counter'])

    def execute(self, ud):
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        ud.block_counter = 0
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
        ud.result = String('Unable to go to location')
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
        ud.result = String('Location reached')
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


class DummyNo(State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['no', 'yes'],
            input_keys=['command'],
            output_keys=['result', 'command']
        )

    def execute(self, ud):
        return 'no'


class BlockedFirst(State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['first', 'second'],
            input_keys=['block_counter'],
            output_keys=['block_counter']
        )

    def execute(self, ud):
        if ud.block_counter < 2:
            ud.block_counter += 1
            return 'first'
        else:
            return 'second'


class DummyYes(State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['no', 'yes'],
            input_keys=['command'],
            output_keys=['result', 'command']
        )

    def execute(self, ud):
        return 'yes'


class Dummy(State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
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

    seq = Sequence(
        outcomes=['succeeded', 'failed', 'preempted'],
        connector_outcome='succeeded',
        input_keys=['emotion', 'emo_time']
    )

    goto_sm.userdata.result = String('started')
    goto_sm.userdata.emotion = 'WONDERING'
    goto_sm.userdata.emo_time = 4

    with goto_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'CALL_CHECK'}
            # TODO: Do we get touch info from MMUI?
            # If yes, do EMO_WONDERING + T_GT_SelectPlace
        )
        StateMachine.add(
            'CALL_CHECK',
            CallCheck(),
            transitions={'touch': 'EMO_HAPPY',
                         'voice': 'EMO_WONDERING',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'EMO_WONDERING',
            HobbitEmotions.ShowEmotions(emotion='EMO_WONDERING', emo_time=4),
            transitions={'preempted': 'preempted',
                         'succeeded': 'MMUI_ConfirmPlace',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'MMUI_ConfirmPlace',
            HobbitMMUI.AskYesNo(question='T_GT_ConfirmGoToPlace'),
            transitions={'yes': 'EMO_HAPPY',
                         'no': 'MMUI_REPEAT_CMD',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'MMUI_REPEAT_CMD',
            HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=4),
            transitions={'preempted': 'preempted',
                         'succeeded': 'failure',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'EMO_HAPPY',
            HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=4),
            transitions={'preempted': 'preempted',
                         'succeeded': 'VIEW_BLOCKED',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'VIEW_BLOCKED',
            DummyNo(),
            transitions={'yes': 'EMO_SAD',
                         'no': 'SEQ'}
        )
        StateMachine.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='EMO_SAD', emo_time=4),
            transitions={'preempted': 'preempted',
                         'succeeded': 'COUNT_CHECK',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'COUNT_CHECK',
            BlockedFirst(),
            transitions={'first': 'MMUI_SAY_CAM_BLOCKED',
                         'second': 'MMUI_SAY_CAM_STILL_BLOCKED'}
        )
        StateMachine.add(
            'MMUI_SAY_CAM_BLOCKED',
            HobbitMMUI.ShowInfo(info='T_GT_EyesBlockedMoveObject'),
            transitions={'succeeded': 'VIEW_BLOCKED',
                         'preempted': 'preempted',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'MMUI_SAY_CAM_STILL_BLOCKED',
            HobbitMMUI.ShowInfo(info='T_GT_EyesBlockedRemoveObject'),
            transitions={'succeeded': 'SET_FAILURE',
                         'preempted': 'preempted',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'SEQ',
            seq,
            transitions={'succeeded': 'SET_SUCCESS',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        with seq:
            Sequence.add(
                'EMO_HAPPY',
                HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=4))
            Sequence.add(
                'MMUI_SAY_GoingToPlace',
                HobbitMMUI.ShowInfo(info='T_GT_GoingToPlace'))
            Sequence.add(
                'WAIT_FOR_MMUI',
                HobbitMMUI.WaitforSoundEnd('/Event', Event),
                transitions={'aborted': 'WAIT_FOR_MMUI'})
            if not DEBUG:
                Sequence.add(
                    'MOVE_TO_DOCK',
                    hobbit_move.goToPosition()
                )
            else:
                Sequence.add('SET_NAV_GOAL', Dummy())
                Sequence.add(
                    'MOVE_BASE_GO',
                    Dummy(),
                    transitions={'failed': 'EMO_SAD',
                                 'succeeded': 'EMO_HAPPY_1'})
            Sequence.add(
                'EMO_SAD',
                HobbitEmotions.ShowEmotions(emotion='EMO_SAD', emo_time=4))
            Sequence.add(
                'MMUI_SAY_WayBlocked',
                HobbitMMUI.ShowInfo(info='T_GT_WayBlocked'))
            Sequence.add(
                'WAIT_FOR_MMUI_1',
                HobbitMMUI.WaitforSoundEnd('/Event', Event),
                transitions={'aborted': 'WAIT_FOR_MMUI_1'})
            Sequence.add(
                'SHOW_MENU_MAIN',
                HobbitMMUI.ShowMenu(menu='MAIN'),
                transitions={'succeeded': 'failed'})
            Sequence.add(
                'EMO_HAPPY_1',
                HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=4))
            Sequence.add(
                'MMUI_SAY_ReachedPlace',
                HobbitMMUI.ShowInfo(info='T_GT_ReachedMyDestionation'))
            Sequence.add(
                'WAIT_FOR_MMUI_2',
                HobbitMMUI.WaitforSoundEnd('/Event', Event),
                transitions={'aborted': 'WAIT_FOR_MMUI_2'})
            Sequence.add(
                'SHOW_MENU_MAIN_1',
                HobbitMMUI.ShowMenu(menu='MAIN'))
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

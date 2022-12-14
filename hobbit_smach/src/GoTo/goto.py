#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'goto'
DEBUGGOTO = False
MMUI_IS_DOING_IT = True

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.speech_output_import as speech_output

from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction, Event
from hobbit_msgs.srv import GetCoordinates
from smach_ros import ActionServerWrapper, IntrospectionServer
from smach import StateMachine, State, Sequence
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
from uashh_smach.util import SleepState
import hobbit_smach.head_move_import as head_move
import hobbit_smach.logging_import as log


class TestData(State):
    """
    """

    def __init__(self, ):
        State.__init__(
            self,
            input_keys=['room_name', 'location_name'],
            outcomes=['succeeded', 'failed', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('TestData')
        print(ud.room_name, ud.location_name)
        return 'succeeded'


class Init(State):
    """Class to initialize certain parameters"""
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failure'],
            input_keys=['command', 'parameters'],
            output_keys=['social_role', 'block_counter', 'room_name', 'location_name'])

    def execute(self, ud):
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        ud.block_counter = 0
        print(ud.parameters)
        ud.room_name = ud.parameters[0].data
        ud.location_name = ud.parameters[1].data
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
        output_keys=['result', 'room_name', 'location_name'])

    seq = Sequence(
        outcomes=['succeeded', 'failed', 'preempted'],
        connector_outcome='succeeded',
        input_keys=['room_name', 'location_name']
    )

    goto_sm.userdata.result = String('started')
    goto_sm.userdata.emotion = 'WONDERING'
    goto_sm.userdata.emo_time = 1

    with goto_sm:
        # Entry point of state machine
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'CALL_CHECK'}
            # TODO: Do we get touch info from MMUI?
            # If yes, do EMO_WONDERING + T_GT_SelectPlace
        )
        # Check if the state machine was called from the user or not. hardcoded to touch input
        StateMachine.add(
            'CALL_CHECK',
            CallCheck(),
            transitions={'touch': 'EMO_HAPPY',
                         'voice': 'EMO_WONDERING',
                         'preempted': 'LOG_PREEMPT'}
        )
        # Show the emotion "wondering" for one second
        StateMachine.add(
            'EMO_WONDERING',
            HobbitEmotions.ShowEmotions(emotion='EMO_WONDERING', emo_time=1),
            transitions={'preempted': 'LOG_PREEMPT',
                         'succeeded': 'MMUI_ConfirmPlace',
                         'failed': 'SET_FAILURE'}
        )
        # Ask the user to confirm that the robot shall go to the defined location
        StateMachine.add(
            'MMUI_ConfirmPlace',
            HobbitMMUI.AskYesNo(question='T_GT_ConfirmGoToPlace'),
            transitions={'yes': 'EMO_HAPPY',
                         'no': 'MMUI_REPEAT_CMD',
                         'failed': 'SET_FAILURE',
                         'preempted': 'LOG_PREEMPT',
                         'timeout': 'MMUI_ConfirmPlace',
                         '3times': 'SET_FAILURE'}
        )
        # For now we only show the emotion "happy" for one second
        StateMachine.add(
            'MMUI_REPEAT_CMD',
            HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=1),
            transitions={'preempted': 'LOG_PREEMPT',
                         'succeeded': 'failure',
                         'failed': 'SET_FAILURE'}
        )
        # Show the emotion "happy" for one second
        StateMachine.add(
            'EMO_HAPPY',
            HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=1),
            transitions={'preempted': 'LOG_PREEMPT',
                         'succeeded': 'VIEW_BLOCKED',
                         'failed': 'SET_FAILURE'}
        )

        StateMachine.add(
            'VIEW_BLOCKED',
            DummyNo(),
            transitions={'yes': 'EMO_SAD',
                         'no': 'SEQ'}
        )
        # Show the emotion "sad" for one second
        StateMachine.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='EMO_SAD', emo_time=1),
            transitions={'preempted': 'LOG_PREEMPT',
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
            speech_output.sayText(info='T_GT_EyesBlockedMoveObject'),
            transitions={'succeeded': 'VIEW_BLOCKED',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'MMUI_SAY_CAM_STILL_BLOCKED',
            speech_output.sayText(info='T_GT_EyesBlockedRemoveObject'),
            transitions={'succeeded': 'SET_FAILURE',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'SEQ',
            seq,
            transitions={'succeeded': 'SET_SUCCESS',
                         'failed': 'SET_FAILURE',
                         'preempted': 'LOG_PREEMPT'}
        )
        with seq:
            # Show the emotion "happy" for one second
            Sequence.add(
                'EMO_HAPPY',
                HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=1))
            Sequence.add(
                'MMUI_SAY_GoingToPlace',
                speech_output.sayTextRoom(info='T_GT_GoingToPlace'))
        # Sequence.add(
        #    'TESTDATA',
        #    TestData(),
        # )
            Sequence.add(
                'SET_NAV_GOAL',
                hobbit_move.get_set_nav_goal_state(),
                transitions={'aborted': 'failed'}
            )
        # Sequence.add(
        #     'TESTDATA1',
        #     TestData(),
        # )
            Sequence.add(
                'MOVE_TO_GOAL',
                hobbit_move.goToPose(),
                transitions={'aborted': 'WAIT',
                             'succeeded': 'EMO_HAPPY_1'}
            )
            # Sequence.add(
            #     'EMO_SAD',
            #     #HobbitEmotions.ShowEmotions(emotion='EMO_SAD', emo_time=4))
            #     HobbitEmotions.ShowEmotions(emotion='EMO_SAD', emo_time=1))
            # Sequence.add(
            #     'MMUI_SAY_WayBlocked',
            #     HobbitMMUI.ShowInfo(info='T_GT_WayBlocked'))
                #speech_output.sayText(info='Way blocked'))
            Sequence.add(
                'WAIT',
                SleepState(duration=5)
            )
            Sequence.add(
                'SHOW_MENU_MAIN',
                HobbitMMUI.ShowMenu(menu='MAIN'),
                transitions={'succeeded': 'failed'})
            # Show the emotion "happy" for one second
            Sequence.add(
                'EMO_HAPPY_1',
                #HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=4))
                HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=1))
            #Sequence.add('HEAD_DOWN_BEFORE_MOVEMENT',
            #         head_move.MoveTo(pose='center_center'),
            #             transitions={'aborted': 'SHOW_MENU_MAIN_1'})
            Sequence.add(
                'SHOW_MENU_MAIN_1',
                HobbitMMUI.ShowMenu(menu='MAIN'))
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
        # cleanup if we need to
        StateMachine.add(
            'CLEAN_UP',
            CleanUp(),
            transitions={'succeeded': 'LOG_PREEMPT'})
        # send success to the logging facility
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Goto'),
            transitions={'succeeded': 'succeeded'}
        )
        # send preempt to the logging facility
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Goto'),
            transitions={'succeeded': 'preempted'}
        )
        # send abort to the logging facility
        StateMachine.add(
            'LOG_ABORT',
            log.DoLogAborted(scenario='Goto'),
            transitions={'succeeded': 'failure'}
        )

    # wrap the whole scenario in a ROS Actionserver
    # This makes it easy to start it

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

    #sis = IntrospectionServer('smach_server', goto_sm, '/HOBBIT/goto_sm_ROOT')
    #sis.start()
    asw.run_server()
    rospy.spin()
    #sis.stop()

if __name__ == '__main__':
    main()

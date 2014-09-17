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
import hobbit_smach.head_move_import as head_move
import hobbit_smach.logging_import as log


class Init(State):
    """Class to initialize certain parameters"""
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failure'],
            input_keys=['command', 'parameters'],
            output_keys=['social_role', 'block_counter', 'room_name', 'location_name', 'parameters'])

    def execute(self, userdata):
        if rospy.has_param('/hobbit/social_role'):
            userdata.social_role = rospy.get_param('/hobbit/social_role')
        userdata.block_counter = 0
        print(userdata.parameters)
        userdata.room_name = userdata.parameters[0].data
        userdata.location_name = userdata.parameters[1].data
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
                       output_keys=['question', 'timeframe', 'parameters']
                       )

    def execute(self, userdata):
        if self.preempt_requested():
            userdata.result = String('preempted')
            return 'preempted'
        print(userdata.parameters)
        print(userdata.command)
        return 'touch'
        if userdata.parameters[0].data.lower() == 'command':
            # TODO: Change hardcoded question to one from the translation pack
            pass
        else:
            rospy.loginfo(
                'Unknown type in GeneralHobbitAction: %s' % userdata.command.data)
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

    def execute(self, userdata):
        self.pub_face.publish('EMO_SAD')
        userdata.result = String('Unable to go to location')
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

    def execute(self, userdata):
        self.pub_face.publish('EMO_HAPPY')
        self.pub.publish('Stop')
        if self.preempt_requested():
            userdata.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        userdata.result = String('Location reached')
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

    def execute(self, userdata):
        self.pub_face.publish('EMO_SAD')
        self.pub.publish('Stop')
        if self.preempt_requested():
            userdata.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        userdata.result = String('failure')
        return 'succeeded'


class DummyNo(State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['no', 'yes'],
            input_keys=['command', 'parameters'],
            output_keys=['result', 'command', 'room_name', 'location_name', 'parameters']
        )

    def execute(self, userdata):
        rospy.loginfo('CHECK_VIEW: room_name: %s' % userdata.parameters[0].data)
        rospy.loginfo('CHECK_VIEW: place_name: %s' % userdata.parameters[1].data)
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

    def execute(self, userdata):
        if userdata.block_counter < 2:
            userdata.block_counter += 1
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

    def execute(self, userdata):
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

    def execute(self, userdata):
        #userdata.result = String('')
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
    goto_sm.userdata.room = 'room_name_dummy'

    with goto_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'CALL_CHECK'},
            remapping={'room_name': 'room'}
        )
        StateMachine.add(
            'CALL_CHECK',
            CallCheck(),
            transitions={'touch': 'EMO_HAPPY',
                         'voice': 'EMO_WONDERING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'EMO_WONDERING',
            #HobbitEmotions.ShowEmotions(emotion='EMO_WONDERING', emo_time=4),
            HobbitEmotions.ShowEmotions(emotion='EMO_WONDERING', emo_time=1),
            transitions={'preempted': 'LOG_PREEMPT',
                         'succeeded': 'MMUI_ConfirmPlace',
                         'failed': 'SET_FAILURE'}
        )
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
        StateMachine.add(
            'MMUI_REPEAT_CMD',
            #HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=4),
            HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=1),
            transitions={'preempted': 'LOG_PREEMPT',
                         'succeeded': 'failure',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'EMO_HAPPY',
            #HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=4),
            HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=1),
            transitions={'preempted': 'LOG_PREEMPT',
                         'succeeded': 'VIEW_BLOCKED',
                         'failed': 'SET_FAILURE'}
        )
        StateMachine.add(
            'VIEW_BLOCKED',
            DummyNo(),
            transitions={'yes': 'EMO_SAD',
                         'no': 'MMUI_SAY_GoingToPlace'}
        )
        StateMachine.add(
            'EMO_SAD',
            #HobbitEmotions.ShowEmotions(emotion='EMO_SAD', emo_time=4),
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
            'MMUI_SAY_GoingToPlace',
            speech_output.sayTextRoom(info='T_GT_GoingToPlace',
                                      room=goto_sm.userdata.room),
            transitions={'succeeded': 'SEQ',
                         'failed': 'SET_FAILURE'})
        StateMachine.add(
            'SEQ',
            seq,
            transitions={'succeeded': 'SET_SUCCESS',
                         'failed': 'SET_FAILURE',
                         'preempted': 'LOG_PREEMPT'}
        )
        with seq:
            Sequence.add(
                'EMO_HAPPY',
                HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=1))
            if not DEBUGGOTO:
                Sequence.add(
                    'SET_NAV_GOAL',
	                hobbit_move.get_set_nav_goal_state(),
                    transitions={'aborted': 'failed'}
                )
                Sequence.add(
                    'MOVE_TO_GOAL',
                    hobbit_move.goToPose(),
                    transitions={'aborted': 'EMO_SAD',
                                 'succeeded': 'EMO_HAPPY_1'}
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
                HobbitEmotions.ShowEmotions(emotion='EMO_SAD', emo_time=1))
            Sequence.add(
                'MMUI_SAY_WayBlocked',
                HobbitMMUI.ShowInfo(info='T_GT_WayBlocked'))
                #speech_output.sayText(info='Way blocked'))
            Sequence.add(
                'SHOW_MENU_MAIN',
                HobbitMMUI.ShowMenu(menu='MAIN'),
                transitions={'succeeded': 'failed'})
            Sequence.add(
                'EMO_HAPPY_1',
                HobbitEmotions.ShowEmotions(emotion='EMO_HAPPY', emo_time=1))
            Sequence.add('HEAD_DOWN_BEFORE_MOVEMENT',
                     head_move.MoveTo(pose='center_center'),
                         transitions={'aborted': 'SHOW_MENU_MAIN_1'})
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
        StateMachine.add(
            'CLEAN_UP',
            CleanUp(),
            transitions={'succeeded': 'LOG_PREEMPT'})
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='GoTo'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORT',
            log.DoLogPreempt(scenario='GoTo'),
            transitions={'succeeded': 'failure'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogPreempt(scenario='GoTo'),
            transitions={'succeeded': 'succeeded'}
        )

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

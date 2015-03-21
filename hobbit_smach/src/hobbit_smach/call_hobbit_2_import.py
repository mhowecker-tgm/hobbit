###!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'call_hobbit_2'
DEBUG = False
PREEMPT_TIMEOUT = 5
SERVER_TIMEOUT = 5

import rospy
import smach
import threading
from std_msgs.msg import String
from smach_ros import SimpleActionState, ServiceState
from smach import Iterator, Sequence, State, StateMachine, Concurrence
from uashh_smach.util import SleepState, WaitForMsgState, WaitAndCheckMsgState
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.logging_import as log
from hobbit_msgs.msg import GeneralHobbitAction, GeneralHobbitGoal
from rgbd_acquisition.msg import Person
from hobbit_user_interaction import HobbitMMUI
import head_move_import as head_move
from uashh_smach.util import WaitForMsgState, SleepState
from hobbit_msgs.msg import Event
from hobbit_msgs.srv import SetCloserStateRequest
from hobbit_msgs.srv import SwitchVision, SwitchVisionRequest, SetCloserState
from hobbit_msgs import MMUIInterface as MMUI

def closer_cb(ud, goal):
    params=[]
    params.append(String(str(ud.person_z/1000)))
    params.append(String(str(-ud.person_x/1000)))
    goal = GeneralHobbitGoal(
        command=String('start'),
        parameters=params
    )
    return goal

def switch_vision_cb(ud, response):
    if response.result:
        return 'succeeded'
    else:
        return 'aborted'

class ExtractGoal(State):
    """
    """
    def __init__(self):
        State.__init__(
            self,
            input_keys=['params', 'room_name', 'location_name'],
            output_keys=['room_name', 'location_name'],
            outcomes=['succeeded', 'aborted', 'preempted']
        )

    def execute(self, ud):
        rospy.sleep(1.0)
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('NAME: ', ud.params[0].name)
        print('VALUE: ', ud.params[0].value)
        ud.room_name  = ud.params[0].name.lower()
        ud.location_name = ud.params[0].value.lower()
        print(ud.room_name)
        print(ud.location_name)
        return 'succeeded'

class Count(State):
    """
    Just count the number of times the come closer should be done
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded',
                      'aborted',
                      'preempted']
        )
        self.counter = 1

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('Counter'+str(self.counter))
        # FIXME: use of magic number
        if self.counter < 3:
            self.counter += 1
            return 'succeeded'
        self.counter = 1
        return 'aborted'

class Count2(State):
    """
    Just count the number of times the come closer should be done
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded',
                      'aborted',
                      'preempted']
        )
        self.counter = 1

    def execute(self, ud):
        if self.preempt_requested():
        # self.counter = 1
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('Counter: '+str(self.counter))
        # FIXME: use of magic number
        if self.counter < 2:
            self.counter += 1
            return 'succeeded'
        self.counter = 1
        return 'aborted'

def msg_timer_sm():
    sm = StateMachine(outcomes = ['succeeded','aborted','preempted'],
                      output_keys=['person_x', 'person_z'])

    def person_cb(msg, ud):
        print(msg)
        if (rospy.Time.now() - msg.stamp) > rospy.Duration(1):
            rospy.loginfo('Person data is too old. Ignore.')
            return False
        if msg.source == 6:
            #print('Do not use this data')
            return False
        ud.person_x = msg.x
        ud.person_z = msg.z
        #print("OK. Use it.")
        return True

    with sm:
        StateMachine.add(
            'GET_PERSON',
            WaitAndCheckMsgState(
                '/persons',
                Person,
                msg_cb=person_cb,
                timeout=5,
                output_keys=['user_pose', 'person_z', 'person_x']
            ),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'aborted'}
        )
    return sm

def gesture_sm():

    
    sm = StateMachine(outcomes = ['succeeded','aborted','preempted'])

    with sm:
        counter_it = Iterator(outcomes = ['succeeded', 'preempted', 'aborted'],
                              input_keys = [],
                              output_keys = [],
                              it = lambda: range(0, 2),
                              it_label = 'index',
                              exhausted_outcome = 'succeeded')

        with counter_it:
            def child_term_cb(outcome_map):
                rospy.loginfo('cc1: child_term_cb: ')
                rospy.loginfo(str(outcome_map))
                return True
            def out_cb(outcome_map):
                rospy.loginfo(str(outcome_map))
                if outcome_map['YES_NO'] == 'yes':
                    return 'succeeded'
                elif outcome_map['TOPIC'] == 'succeeded':
                    return 'succeeded'
                elif outcome_map['TOPIC'] == 'aborted':
                    return 'aborted'
                elif outcome_map['YES_NO'] in ['no', 'timeout', '3times', 'failed']:
                    return 'aborted'
                else:
                    return 'preempted'


            container_sm = StateMachine(outcomes = ['succeeded','aborted','preempted'])
        
            cc1 = Concurrence(outcomes=['aborted', 'succeeded', 'preempted'],
                             default_outcome='aborted',
                             child_termination_cb=child_term_cb,
                             outcome_cb=out_cb
            )
            with container_sm:
                def g_come_cb(event_message, ud):
                    if event_message.event.upper() in ['G_COME', 'A_CLOSER']:
                         rospy.loginfo('Was G_COME or A_CLOSER! Close MMUI Prompt!')
                         mmui = MMUI.MMUIInterface()
                         mmui.remove_last_prompt()
                         return 'succeeded'

                StateMachine.add(
                    'WAIT_FOR_CLOSER',
                    cc1,
                    transitions={'succeeded': 'MOVE',
                                 'preempted': 'preempted',
                                 'aborted': 'aborted'}
                )
                StateMachine.add(
                    'MOVE',
                    hobbit_move.move_discrete(in_motion='Move', in_value=0.1),
                    transitions={'succeeded': 'MOVED_BACK',
                                 'preempted': 'preempted',
                                 'aborted': 'aborted'}
                )
                StateMachine.add(
                    'MOVED_BACK',
                    ServiceState(
                        '/came_closer/set_closer_state',
                        SetCloserState,
                        request=SetCloserStateRequest(state=True),
                    ),
                    transitions={'succeeded': 'succeeded',
                                 'preempted': 'preempted',}
                )

            with cc1:
                Concurrence.add(
                    'YES_NO',
                    HobbitMMUI.AskYesNo(question='T_CLOSER_QUESTION'),
                )
                Concurrence.add(
                    'TOPIC',
                    WaitAndCheckMsgState(
                        '/Event',
                        Event,
                        msg_cb=g_come_cb,
                        timeout=45
                    )
                )

            Iterator.set_contained_state('CONTAINER_STATE',
                                         container_sm,
                                         loop_outcomes = ['succeeded']
                                        )

        StateMachine.add('COUNT_GESTURES', counter_it,
                        {'succeeded':'succeeded',
                         'preempted':'preempted',
                         'aborted':'aborted'})
    return sm


def call_hobbit():
    """
    First check if Hobbit is charging and in the docking station,
    if so move out. Then go to the user.
    """

    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command', 'params', 'parameters'],
        output_keys=['command', 'params', 'parameters']
    )

    with sm:
        StateMachine.add_auto(
            'EXTRACT_GOAL',
            ExtractGoal(),
            connector_outcomes=['succeeded']
        )
        StateMachine.add_auto(
            'SET_NAV_GOAL',
            hobbit_move.SetNavigationGoal(),
            connector_outcomes=['succeeded']
        )
        StateMachine.add(
            'MOVE_TO_GOAL',
            hobbit_move.goToPoseSilent(),
            transitions={'succeeded': 'SWITCH_VISION',
                         'aborted': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'SWITCH_VISION',
            ServiceState(
                '/vision_system/comeCloser',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb
            ),
            transitions={'succeeded': 'HEAD_UP',
                         'aborted': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'HEAD_UP',
            head_move.MoveTo(pose='littledown_center'),
            transitions={'succeeded': 'GET_PERSON',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORT'}
        )
        StateMachine.add(
            'GET_PERSON',
            msg_timer_sm(),
            transitions={'succeeded': 'CLOSER',
                         'preempted': 'preempted',
                         'aborted': 'SAY_NOT_DETECTED'}
        )
        StateMachine.add(
            'CLOSER',
            SimpleActionState(
                'come_closer',
                GeneralHobbitAction,
                goal_cb=closer_cb,
                input_keys=['person_z', 'person_x'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'MOVED',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'GESTURE_HANDLING'}
        )
        StateMachine.add(
            'MOVED',
            ServiceState(
                '/came_closer/set_closer_state',
                SetCloserState,
                request=SetCloserStateRequest(state=True),
            ),
            transitions={'succeeded': 'GESTURE_HANDLING',
                         'preempted': 'LOG_PREEMPT',}
        )
        StateMachine.add(
            'SAY_NOT_DETECTED',
            speech_output.sayText(
                info="T_NO_USER"),
            transitions={'succeeded': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'LOG_ABORT'}
        )
        StateMachine.add(
            'GESTURE_HANDLING',
            gesture_sm(),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORT'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Call Hobbit'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORT',
            log.DoLogAborted(scenario='Call Hobbit'),
            transitions={'succeeded': 'aborted'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Call Hobbit'),
            transitions={'succeeded': 'succeeded'}
        )

        return sm


def come_closer_from_everywhere():
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with sm:
        StateMachine.add(
            'SWITCH_VISION',
            ServiceState(
                '/vision_system/comeCloser',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb
            ),
            transitions={'succeeded': 'HEAD_UP',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'HEAD_UP',
            head_move.MoveTo(pose='littledown_center'),
            transitions={'succeeded': 'GET_PERSON',
                         'preempted': 'preempted',
                         'aborted': 'aborted'}
        )
        StateMachine.add(
            'GET_PERSON',
            msg_timer_sm(),
            transitions={'succeeded': 'CLOSER',
                         'preempted': 'preempted',
                         'aborted': 'SAY_NOT_DETECTED'}
        )
        StateMachine.add(
            'CLOSER',
            SimpleActionState(
                'come_closer',
                GeneralHobbitAction,
                goal_cb=closer_cb,
                input_keys=['person_z', 'person_x'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'MOVED',
                         'preempted': 'preempted',
                         'aborted': 'GESTURE_HANDLING'}
        )
        StateMachine.add(
            'MOVED',
            ServiceState(
                '/came_closer/set_closer_state',
                SetCloserState,
                request=SetCloserStateRequest(state=True),
            ),
            transitions={'succeeded': 'GESTURE_HANDLING',
                         'preempted': 'preempted',}
        )
        StateMachine.add(
            'SAY_NOT_DETECTED',
            speech_output.sayText(
                info="T_NO_USER"),
            transitions={'succeeded': 'aborted',
                         'preempted': 'preempted',
                         'failed': 'aborted'}
        )
        StateMachine.add(
            'GESTURE_HANDLING',
            gesture_sm(),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'preempted',
                         'aborted': 'aborted'}
        )
    return sm
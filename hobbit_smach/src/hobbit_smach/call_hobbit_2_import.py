#!/usr/bin/python
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
from smach import Sequence, State, StateMachine, Concurrence
from uashh_smach.util import SleepState, WaitForMsgState
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

def event_cb(msg, ud):
    if msg.event.upper() in ['G_COME', 'A_YES', 'G_YES']:
        rospy.loginfo("Move 0.1 meters")
        return True
    else:
        rospy.loginfo("Do not move")
        return False

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

    def child_term_cb(outcome_map):
        print(outcome_map)
        return True

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

    cc = Concurrence(outcomes=['aborted', 'succeeded'],
                     default_outcome='aborted',
                     child_termination_cb=child_term_cb,
                     output_keys=['person_x', 'person_z'],
                     outcome_map={'succeeded': {'LISTENER': 'succeeded'},
                                  'aborted': {'TIMER': 'succeeded'}})
    with sm:
        StateMachine.add(
            'GET_PERSON',
            WaitForMsgState(
                '/persons',
                Person,
                msg_cb=person_cb,
                timeout=3,
                output_keys=['user_pose', 'person_z', 'person_x']
            ),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'GET_PERSON'}
        )
    with cc:
        Concurrence.add('LISTENER', sm)
        Concurrence.add('TIMER', SleepState(duration=3))
    return cc

def gesture_sm():
    sm = StateMachine(outcomes = ['succeeded','aborted','preempted'])

    def child_term_cb(outcome_map):
        return True
    def out_cb(outcome_map):
        rospy.loginfo(str(outcome_map))
        if outcome_map['TOPIC'] == 'succeeded':
            return 'succeeded'
        elif outcome_map['YES_NO'] == 'yes':
            return 'succeeded'
        elif outcome_map['YES_NO'] in ['no', 'timout', '3times', 'failed']:
            return 'aborted'
        else:
            return 'preempted'

    cc1 = Concurrence(outcomes=['aborted', 'succeeded', 'preempted'],
                     default_outcome='aborted',
                     child_termination_cb=child_term_cb,
                     outcome_cb=out_cb
    )
    cc = Concurrence(outcomes=['aborted', 'succeeded', 'preempted'],
                      default_outcome='aborted',
                      child_termination_cb=child_term_cb,
                      outcome_map={'succeeded': {'LISTENER': 'succeeded'},
                                   'aborted': {'TIMER': 'succeeded'},
                                   'preempted': {'LISTENER': 'preempted',
                                                 'TIMER': 'preempted'}}
                      )
    with sm:
        StateMachine.add(
            'WAIT_FOR_CLOSER',
            cc1,
            transitions={'succeeded': 'MOVE',
                         'preempted': 'preempted',
                         'aborted': 'WAIT_FOR_CLOSER'}
        )
        StateMachine.add(
            'MOVE',
            hobbit_move.MoveDiscrete(motion='Move', value=0.1),
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
            transitions={'succeeded': 'WAIT_FOR_CLOSER',
                         'preempted': 'preempted',}
        )

    with cc:
        Concurrence.add('LISTENER', sm)
        Concurrence.add('TIMER', SleepState(duration=30))

    with cc1:
        Concurrence.add(
            'YES_NO',
            HobbitMMUI.AskYesNo(question='Shall I come even closer?'),
        )
        Concurrence.add(
            'TOPIC',
            WaitForMsgState(
                '/Event',
                Event,
                msg_cb=event_cb,
                timeout=10
            )

        )

    return cc


class CheckMsgState(WaitForMsgState):
    def __init__(self, topic, msg_type, msg_cb=None, output_keys=None, latch=False, timeout=None):
        if output_keys is None:
            output_keys = []
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted', 'timeout'], output_keys=output_keys)
        self.latch = latch
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.msg = None
        self.msg_cb = msg_cb
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)
    def execute(self, ud):
        """Tiny changes to default execute(), see class description."""
        msg = self.waitForMsg()
        if msg is not None:
            if msg == 'preempted':
                return 'preempted'
            # call callback if there is one
            if self.msg_cb is not None:
                cb_result = self.msg_cb(msg, ud)
                # check if callback wants to dictate output
                if cb_result is not None:
                    if cb_result:
                        return 'succeeded'
                    else:
                        return 'aborted'
            return 'succeeded'
        else:
            return 'timeout'


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
            transitions={'succeeded': 'HEAD_UP',
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
            )
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
                         'aborted': 'COUNT'}
        )
        StateMachine.add(
            'COUNT',
            Count(),
            transitions={'succeeded': 'GET_PERSON',
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
                         'aborted': 'HEAD_UP_AFTER_MOVEMENT'}
        )
        StateMachine.add(
            'MOVED',
            ServiceState(
                '/came_closer/set_closer_state',
                SetCloserState,
                request=SetCloserStateRequest(state=True),
            ),
            transitions={'succeeded': 'HEAD_UP_AFTER_MOVEMENT',
                         'preempted': 'LOG_PREEMPT',}
        )
        StateMachine.add(
            'HEAD_UP_AFTER_MOVEMENT',
            head_move.MoveTo(pose='littledown_center'),
            transitions={'succeeded': 'GESTURE_HANDLING',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORT'}
        )
        StateMachine.add(
            'SAY_NOT_DETECTED',
            speech_output.sayText(
                info="Sorry, I can't see you. Press the call button again if I should try again."),
            transitions={'succeeded': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'LOG_ABORT'}
        )
        StateMachine.add(
            'WAIT',
            SleepState(duration=3),
            transitions={'succeeded': 'GESTURE_HANDLING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'GESTURE_HANDLING',
            gesture_sm(),
            transitions={'succeeded': 'COUNT_2',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORT'}
        )
        StateMachine.add(
            'COUNT_2',
            Count2(),
            transitions={'succeeded': 'GESTURE_HANDLING',
                         'aborted': 'LOG_SUCCESS',
                         'preempted': 'LOG_PREEMPT'}
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

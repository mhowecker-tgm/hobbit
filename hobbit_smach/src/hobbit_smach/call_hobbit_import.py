#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'call_hobbit'
DEBUG = False
PREEMPT_TIMEOUT = 5
SERVER_TIMEOUT = 5

import rospy
from std_msgs.msg import String
from smach_ros import SimpleActionState
from smach import Sequence, State, StateMachine, Concurrence
from uashh_smach.util import SleepState, WaitForMsgState
# from uashh_smach.platform.move_base import HasMovedState
from mira_msgs.msg import BatteryState
import hobbit_smach.hobbit_move_import as hobbit_move
from hobbit_user_interaction import HobbitMMUI
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.logging_import as log
from hobbit_msgs.msg import GeneralHobbitAction, GeneralHobbitGoal
from rgbd_acquisition.msg import Person
import head_move_import as head_move
from uashh_smach.util import WaitForMsgState, SleepState
from hobbit_msgs.msg import Event


def closer_cb(ud, goal):
    params=[]
    params.append(String(str(ud.person_z/1000)))
    params.append(String(str(-ud.person_x/1000)))
    goal = GeneralHobbitGoal(
        command=String('start'),
        parameters=params
    )
    return goal

def battery_cb(msg, ud):
    print('Received battery_state message')
    print(msg.charging)
    #print(ud.params)
    rospy.sleep(2.0)
    if msg.charging:
        print('I am charging')
        return True
    else:
        print('I am NOT charging')
        return False

def event_cb(msg, ud):
    if msg.event.upper() in ['G_CLOSER', 'A_YES', 'G_YES']:
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


class PreemptChecker(State):
    """
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )

    def execute(self, ud):
        rospy.sleep(1.0)
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'aborted'

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

def construct_sm():
    sm = StateMachine(outcomes = ['succeeded','aborted','preempted'])
    def child_term_cb(outcome_map):
        return True

    cc = Concurrence(outcomes=['aborted', 'succeeded', 'preempted'],
                     default_outcome='aborted',
                     child_termination_cb=child_term_cb,
                     outcome_map={'succeeded': {'LISTENER': 'succeeded'},
                                  'aborted': {'TIMER': 'succeeded'}})
    with sm:
        StateMachine.add(
            'WAIT_FOR_CLOSER',
            WaitForMsgState(
                '/Event',
                Event,
                msg_cb=event_cb,
                timeout=10
            ),
            transitions={'succeeded': 'MOVE',
                         'preempted': 'preempted',
                         'aborted': 'WAIT_FOR_CLOSER'}
        )
        StateMachine.add(
            'MOVE',
            hobbit_move.MoveDiscrete(motion='Move', value=0.1),
            transitions={'succeeded': 'COUNT',
                         'preempted': 'preempted',
                         'aborted': 'aborted'}
        )
        StateMachine.add(
            'COUNT',
            Count(),
            transitions={'succeeded': 'WAIT_FOR_CLOSER',
                         'preempted': 'preempted',
                         'aborted': 'succeeded'}
        )

    with cc:
        Concurrence.add('LISTENER', sm)
        Concurrence.add('TIMER', SleepState(duration=60))
    return cc


def getRecharge():
    """This function handles the autonomous charging sequence.
    It is without the user interaction and is mainly used during
    the night or as part of the recharging scenario.
    """

    seq = Sequence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        connector_outcome='succeeded',
    )
    seq.userdata.room_name = 'dock'
    seq.userdata.location_name = 'dock'

    with seq:
        if not DEBUG:
            Sequence.add(
                'SET_NAV_GOAL',
                hobbit_move.SetNavGoal(room='dock', place='dock')
            )
            Sequence.add(
                'MOVE_TO_DOCK',
                hobbit_move.goToPose())
            Sequence.add(
                'DOCKING',
                startDockProcedure())
        else:
            pass
        Sequence.add('MMUI_MAIN_MENU', HobbitMMUI.ShowMenu(menu='MAIN'),
                     transitions={'failed': 'aborted'})
    return seq


# def get_end_recharge():
#     """This function handles the autonomous charging sequence.
#     It is without the user interaction and is mainly used during
#     the night or as part of the recharging scenario.
#     """
#
#     seq = Sequence(
#         outcomes=['succeeded', 'aborted', 'preempted'],
#         input_keys=['room_name', 'location_name'],
#         connector_outcome='succeeded'
#     )
#
#     with seq:
#         Sequence.add(
#             'UNDOCK',
#             hobbit_move.Undock()
#         )
#         #  Sequence.add(
#         #      'SOUND',
#         #      speech_output.playMoveOut()
#         #  )
#         Sequence.add(
#             'WAIT_FOR_MIRA',
#             SleepState(duration=5)
#         )
#         Sequence.add(
#             'SET_NAV_GOAL_DOCK',
#             hobbit_move.SetNavGoal(room='dock', place='dock')
#         )
#         Sequence.add(
#             'MOVE_AWAY_FROM_DOCK',
#             hobbit_move.goToPose())
#     return seq
#
#

def person_cb(msg, ud):
    print(msg)
    if msg.source == 6:
        return None
    ud.person_x = msg.x
    ud.person_z = msg.z
    return True


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
        #StateMachine.add(
        #    'CHARGE_CHECK',
        #    WaitForMsgState('/battery_state', BatteryState, msg_cb=battery_cb, input_keys=['params']),
        #    transitions={'succeeded': 'UNDOCK',
        #                 'aborted': 'EXTRACT_GOAL',
        #                 'preempted': 'preempted'}
        #)
        #StateMachine.add(
        #    'UNDOCK',
        #    hobbit_move.get_undock(),
        #    transitions={'succeeded': 'EXTRACT_GOAL',
        #                 'aborted': 'aborted',
        #                 'preempted': 'preempted'}
        #)
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
            hobbit_move.goToPose(),
            transitions={'succeeded': 'GET_PERSON',
                         'aborted': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'GET_PERSON',
            CheckMsgState(
                '/persons',
                Person,
                msg_cb=person_cb,
                timeout=5,
                output_keys=['user_pose', 'person_z', 'person_x']
            ),
            transitions={'succeeded': 'CLOSER',
                         'aborted': 'COUNT',
                         'preempted': 'LOG_PREEMPT',
                         'timeout': 'MMUI_BLIND_CLOSER'}
        )
        StateMachine.add(
            'COUNT',
            Count(),
            transitions={'succeeded': 'GET_PERSON',
                         'preempted': 'preempted',
                         'aborted': 'CLOSER'}
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
            transitions={'succeeded': 'MMUI_SAY_ReachedPlace',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'MMUI_BLIND_CLOSER'}
        )
        StateMachine.add(
            'HEAD_UP_AFTER_MOVEMENT',
            head_move.MoveTo(pose='center_center'),
            transitions={'succeeded': 'MMUI_SAY_ReachedPlace',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORT'}
        )
        StateMachine.add(
            'MMUI_SAY_ReachedPlace',
            speech_output.sayText(info='T_GT_ReachedMyDestination2'),
            transitions={'succeeded': 'MMUI_Say_come_closer',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'LOG_ABORT'}
        )
        StateMachine.add(
            'MMUI_Say_come_closer',
            speech_output.sayText(
                info='If you want me to move even closer just say or gesture yes or use the come closer gesture.'),
            transitions={'succeeded': 'GESTURE_HANDLING',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'LOG_ABORT'}
        )
        StateMachine.add(
            'MMUI_BLIND_CLOSER',
            speech_output.sayText(
                info='If you want me to move forward say or gesture yes or use the come closer gesture.'),
            transitions={'succeeded': 'GESTURE_HANDLING',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'LOG_ABORT'}
        )
        StateMachine.add(
            'GESTURE_HANDLING',
            construct_sm(),
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

#!/usr/bin/env python

import rospy
import threading
from smach import State, Sequence
from smach_ros import ServiceState
from datetime import datetime, time
from hobbit_user_interaction import HobbitMMUI
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.logging_import as log
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.arm_move_import as arm_move
from uashh_smach.util import SleepState
from hobbit_msgs.srv import GetMoveState, GetMoveStateRequest
from hobbit_msgs.srv import SetMoveState, SetMoveStateRequest
from hobbit_msgs.srv import GetArmState, GetArmStateRequest
from hobbit_msgs.srv import SetArmState, SetArmStateRequest

movement = False

class ResetStorage(State):
    """
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )

    def execute(self, ud):
        global movement
        rospy.loginfo(movement)
        movement = False
        rospy.loginfo(movement)
        return 'succeeded'


class CheckStorage(State):
    """
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )

    def execute(self, ud):
        global movement
        rospy.loginfo(movement)
        if movement:
            return 'succeeded'
        else:
            return 'aborted'


class TimeCheck(State):
    """
    Class to check if we were activated during the night.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['day', 'night', 'canceled'],
            input_keys=['night'],
            output_keys=['night'])
        if rospy.has_param('/sleep_time'):
            self.sleep_time = rospy.get_param('/sleep_time')
        else:
            self.sleep_time = '22:00'
        if rospy.has_param('/wakeup_time'):
            self.wakeup_time = rospy.get_param('/wakeup_time')
        else:
            self.wakeup_time = '06:30'

    def execute(self, ud):
        wake = self.wakeup_time.split(':')
        sleep = self.sleep_time.split(':')
        now = datetime.now()
        if time(int(wake[0]), int(wake[1]))\
                <= now.time()\
                <= time(int(sleep[0]), int(sleep[1])):
            print('yes, within the interval')
            return 'day'
        else:
            return 'night'


def IsItNight():
    if rospy.has_param('/sleep_time'):
        sleep_time = rospy.get_param('/sleep_time')
    else:
        sleep_time = '22:00'
    if rospy.has_param('/wakeup_time'):
        wakeup_time = rospy.get_param('/wakeup_time')
    else:
        wakeup_time = '06:30'

    wake = wakeup_time.split(':')
    sleep = sleep_time.split(':')
    now = datetime.now()
    if time(int(wake[0]), int(wake[1]))\
            <= now.time()\
            <= time(int(sleep[0]), int(sleep[1])):
        return False
    else:
        return True


def get_hobbit_full_stop():
    """
    Return a SMACH sequence that will stop all movement of Hobbit and
    returns to the main menu of the MMUI.
    """
    seq = Sequence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        connector_outcome='succeeded'
    )
    def resp_cb(userdata, response):
        global movement
        if response.result:
            movement = True
            rospy.loginfo('setting movement to TRUE')
            rospy.loginfo(response.result)
            rospy.loginfo(movement)
            return 'succeeded'
        else:
            return 'aborted'

    def resp_empty_cb(userdata, response):
        rospy.loginfo('result was: '+str(response))
        return 'succeeded'

    with seq:
        Sequence.add(
            'CHECK_MOVE',
            ServiceState('/moving/get_move_state',
                         GetMoveState,
                         request=GetMoveStateRequest(state=True),
                         response_cb=resp_cb),
            transitions={'aborted': 'CHECK_ARM'}
        )
        Sequence.add(
            'STOP_MOVEMENT',
            hobbit_move.get_full_stop()
        )
        Sequence.add(
            'CHECK_ARM',
            ServiceState('/arm/get_move_state',
                         GetArmState,
                         request=GetArmStateRequest(state=True),
                         response_cb=resp_cb),
            transitions={'aborted': 'STOP_ARM'}
        )
        Sequence.add(
            'STOP_ARM',
            arm_move.DoArmStop(),
            transitions={'aborted': 'CHECK_MOVEMENT_STORAGE'}
        )
        Sequence.add(
            'CHECK_MOVEMENT_STORAGE',
            CheckStorage(),
            transitions={'succeeded': 'RESET_MOVEMENT_STORAGE',
                         'aborted': 'succeeded'}
        )
        Sequence.add(
            'RESET_MOVEMENT_STORAGE',
            ResetStorage(),
            transitions={'succeeded': 'SAY_STOP',
                         'aborted': 'SET_ARM_STATE'}
        )
        Sequence.add(
            'SAY_STOP',
            speech_output.sayText(
                info='T_STOP_DETECTED'),
            transitions={'preempted': 'LOG_PREEMPT',
                         'failed': 'LOG_ABORTED'}
        )
        Sequence.add(
            'SET_ARM_STATE',
            ServiceState('/moving/set_move_state',
                         SetArmState,
                         request=SetArmStateRequest(state=False),
                         response_cb=resp_empty_cb),
            transitions={'succeeded':'SET_NOT_MOVING',
                         'aborted': 'SET_NOT_MOVING',
                         'preempted': 'preempted'}
        )
        Sequence.add(
            'SET_NOT_MOVING',
            ServiceState('/moving/set_move_state',
                         SetMoveState,
                         request=SetMoveStateRequest(state=False),
                         response_cb=resp_empty_cb),
            transitions={'succeeded':'succeeded',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        Sequence.add(
            'WAIT',
            SleepState(duration=3),
            transitions={'succeeded': 'MAIN_MENU',
                         'preempted': 'LOG_PREEMPT'}
        )
        Sequence.add(
            'MAIN_MENU',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'failed': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        Sequence.add(
            'LOG_SUCCESS',
            log.DoLogPreempt(scenario='Full Stop'),
            transitions={'succeeded': 'succeeded'}
        )
        Sequence.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Full Stop'),
            transitions={'succeeded': 'preempted'}
        )
        Sequence.add(
            'LOG_ABORTED',
            log.DoLogPreempt(scenario='Full Stop'),
            transitions={'succeeded': 'aborted'}
        )
    return seq


class WaitForMsgState(State):
    """This class acts as a generic message listener with blocking, timeout, latch and flexible usage.

    It is meant to be extended with a case specific class that initializes this one appropriately
    and contains the msg_cb (or overrides execute if really needed).

    Its waitForMsg method implements the core functionality: waiting for the message, returning
    the message itself or None on timeout.

    Its execute method wraps the waitForMsg and returns succeeded or aborted, depending on the returned
    message beeing existent or None. Additionally, in the successfull case, the msg_cb, if given, will
    be called with the message and the userdata, so that a self defined method can convert message data to
    smach userdata.
    Those userdata fields have to be passed via 'output_keys'.

    If the state outcome should depend on the message content, the msg_cb can dictate the outcome:
    If msg_cb returns True, execute() will return "succeeded".
    If msg_cb returns False, execute() will return "aborted".
    If msg_cb has no return statement, execute() will act as described above.

    If thats still not enough, execute() might be overridden.

    latch: If True waitForMsg will return the last received message, so one message might be returned indefinite times.
    timeout: Seconds to wait for a message, defaults to None, disabling timeout
    output_keys: Userdata keys that the message callback needs to write to.
    """

    def __init__(self, topic, msg_type, msg_cb=None, input_keys=['dummy'], output_keys=None, latch=False, timeout=None):
        if output_keys is None:
            output_keys = []
        if input_keys is None:
            input_keys = []
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=input_keys, output_keys=output_keys)
        self.latch = latch
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.msg = None
        self.msg_cb = msg_cb
        self.topic = topic
        self.msg_type = msg_type
        # self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)

    def _callback(self, msg):
        self.mutex.acquire()
        self.msg = msg
        self.mutex.release()

    def waitForMsg(self):
        """Await and return the message or None on timeout."""
        self.subscriber = rospy.Subscriber(self.topic, self.msg_type, self._callback, queue_size=1)
        rospy.sleep(1.0)
        rospy.loginfo('Waiting for message...')
        if self.timeout is not None:
            timeout_time = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        while self.timeout is None or rospy.Time.now() < timeout_time:
            self.mutex.acquire()
            if self.msg is not None:
                rospy.loginfo('Got message.')
                message = self.msg

                if not self.latch:
                    self.msg = None

                self.mutex.release()
                return message
            self.mutex.release()

            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('waitForMsg is preempted!')
                return 'preempted'

            rospy.sleep(.1)

        rospy.loginfo('Timeout on waiting for message!')
        return None

    def execute(self, ud):
        """Default simplest execute(), see class description."""
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
            return 'aborted'

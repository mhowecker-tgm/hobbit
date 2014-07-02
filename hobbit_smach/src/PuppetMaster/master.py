#!/usr/bin/env python

PKG = 'hobbit_smach'
NAME = 'HOBBIT_MASTER'
task = 'nothing'
import roslib
roslib.load_manifest(PKG)
import rospy
from smach_ros import SimpleActionState, IntrospectionServer
from smach import StateMachine, Concurrence, State
from std_msgs.msg import String
from hobbit_msgs.msg import Command, Event, GeneralHobbitAction,\
    GeneralHobbitGoal
import uashh_smach.util as util

# sos = ['G_FALL', 'E_SOSBUTTON', 'C_HELP']
# recharge = ['E_RECHARGE']
# stop = ['C_STOP']
# call_hobbit = ['E_CALLHOBBIT']
# break_away = ['C_BREAK']
# reminder = ['E_REMINDER']
# in_call = ['E_CALLRING', 'E_CALLESTABLISHED', 'E_CALLENDED']
# start_call = ['C_MAKECALL']
# clear_floor = ['E_CLEARFLOOR']
# equal = ['C_PICKUP', 'C_FOLLOW', 'C_LEARN', 'C_BRING', 'C_GOTO']
# patrol = ['E_PATROL']
# surprise = ['C_SURPRISE']
# reward = ['C_REWARD']

actions = [['G_FALL', 'E_SOSBUTTON', 'C_HELP', 'E_HELP'],
           ['E_RECHARGE'],
           ['C_STOP'],
           ['E_CALLHOBBIT'],
           ['C_BREAK'],
           ['E_REMINDER'],
           ['E_CALLRING', 'E_CALLESTABLISHED', 'E_CALLENDED'],
           ['C_MAKECALL'],
           ['E_CLEARFLOOR'],
           ['C_PICKUP', 'C_FOLLOW', 'C_LEARN', 'C_BRING', 'C_GOTO'],
           ['E_PATROL'],
           ['C_SURPRISE'],
           ['C_REWARD']]
commands = [['G_FALL', 'E_SOSBUTTON', 'C_HELP', 'E_HELP', 'C_HELP', 'F_CALLSOS', 'G_EMERGENCY'],
            ['E_RECHARGE', 'C_RECHARGE'],
            ['E_REMINDER'],
            ['C_STOP', 'G_STOP'],
            ['C_CALLHOBBIT'],
            ['E_CALLRING', 'E_CALLESTABLISHED', 'E_CALLENDED', 'C_MAKECALL'],
            ['E_CLEARFLOOR'],
            ['C_PICKUP', 'C_FOLLOW', 'C_LEARN', 'C_BRING', 'C_GOTO', 'G_POINTING'],
            ['E_PATROL'],
            ['C_SURPRISE'],
            ['C_REWARD', 'G_REWARD']
            ]

class SimpleActionStateName(SimpleActionState):
    def __init__(self,
            # Action info
            action_name,
            action_spec,
            # Default goal
            goal = None,
            goal_key = None,
            goal_slots = [],
            goal_cb = None,
            goal_cb_args = [],
            goal_cb_kwargs = {},
            # Result modes
            result_key = None,
            result_slots = [],
            result_cb = None,
            result_cb_args = [],
            result_cb_kwargs = {},
            # Keys
            input_keys = [],
            output_keys = [],
            outcomes = [],
            # Timeouts
            exec_timeout = None,
            preempt_timeout = rospy.Duration(60.0),
            server_wait_timeout = rospy.Duration(60.0)
            ):
        print('Inside subclass')
        print(action_name)
        print(action_spec)
        print(server_wait_timeout)
        global task
        action_name = task
        print(action_name)
        print('subclass: action_name = %s' % action_name)
        SimpleActionState.__init__(self,
                                   action_name,
                                   action_spec)


def event_cb(msg, ud):
    rospy.loginfo('/Event data received:')
    print(msg.event)
    if rospy.has_param('active_task'):
        active_task = rospy.get_param('active_task')
    else:
        active_task = 100
    for index, item in enumerate(commands):
        if msg.event in item:
            if index + 1 >= active_task:
                rospy.loginfo('New task has lower priority. Do nothing')
                ud.command = msg.event
                ud.params = msg.params
                return False
            else:
                rospy.loginfo('New task has higher priority. Start it.')
                rospy.set_param('active_task', index)
                return True
    rospy.loginfo('Unknown event received %s' % msg.event)
    return False


def command_cb(msg, ud):
    rospy.loginfo('/Command data received:')
    rospy.loginfo(str(msg))
    print(msg)
    if rospy.has_param('active_task'):
        active_task = rospy.get_param('active_task')
    else:
        active_task = 100
    for index, item in enumerate(commands):
        if msg.command in item:
            if index + 1 >= active_task:
                rospy.loginfo('New task has lower priority. Do nothing')
                ud.command = msg.event
                ud.params = msg.params
                return False
            else:
                rospy.loginfo('New task has higher priority. Start it.')
                rospy.set_param('active_task', index)
                return True
    rospy.loginfo('Unknown command received %s' % msg.command)
    return False


def child_cb(outcome_map):
    print('child_cb')
    return True


def child_cb1(outcome_map):
    print('child_cb1')
    print(outcome_map)
    if outcome_map['Commands'] == 'succeeded':
        return True
    else:
        return False


def task_goal_cb(ud, goal):
    global task
    task = 'learn_object'
    goal = GeneralHobbitGoal(
        command=String('learn_object'),
        previous_state=String('IDLE'),
        parameters=[]
    )
    print('Trying to set goal')
    return goal


class Init(State):
    """Initialize a few data structures
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('Init')
        return 'succeeded'


class SelectTask(State):
    """Select the task for execution
    """
    def __init__(self):
        State.__init__(
            self,
            input_keys=['name'],
            output_keys=['name'],
            outcomes=['succeeded', 'preempted'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('Task Selection')
        global task
        print(task)
        task = 'learn_object'
        print(task)
        return 'succeeded'


class TestASW(State):
    """class to test the ASW functionality
    """
    def __init__(self):
        State.__init__(
            self,
            input_keys=['command'],
            outcomes=['succeeded', 'preempted', 'aborted'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('TestASW')
        print(ud.command)
        rospy.sleep(10)
        return 'succeeded'


def main():
    rospy.init_node(NAME)
    sm = StateMachine(
        outcomes=['succeeded',
                  'preempted',
                  'failed'],
        input_keys=['command', 'active_task', 'params'],
        output_keys=['command', 'active_task', 'params']
    )

    sm1 = StateMachine(
        outcomes=['succeeded',
                  'preempted',
                  'failed']
    )
    sm1.userdata.command = 'IDLE'
    sm1.userdata.active_task = 'IDLE'
    sm1.userdata.params = []

    sm2 = StateMachine(
        outcomes=['succeeded',
                  'preempted',
                  'failed'],
        input_keys=['command', 'params'],
        output_keys=['command', 'active_task']
    )

    cc = Concurrence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        default_outcome='aborted',
        input_keys=['command', 'active_task', 'params'],
        output_keys=['command', 'params'],
        child_termination_cb=child_cb,
        outcome_map={'succeeded': {'Event_Listener': 'succeeded'},
                     'aborted': {'Event_Listener': 'aborted'}}
    )

    cc1 = Concurrence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        default_outcome='aborted',
        input_keys=['command'],
        output_keys=['command'],
        child_termination_cb=child_cb1,
        outcome_map={'succeeded': {'ASW': 'succeeded'},
                     'aborted': {'Commands': 'failed'}}
    )

    with cc:
        Concurrence.add(
            'Event_Listener',
            util.WaitForMsgState(
                '/Event',
                Event,
                msg_cb=event_cb,
                output_keys=['command', 'params']
            )
        )
        Concurrence.add(
            'Command_Listener',
            util.WaitForMsgState(
                '/Command',
                Command,
                msg_cb=command_cb,
                output_keys=['command', 'params']
            )
        )

    with sm:
        StateMachine.add(
            'WAIT_FOR_E_C',
            cc,
            transitions={'succeeded': 'succeeded',
                         'aborted': 'WAIT_FOR_E_C',
                         'preempted': 'preempted'}
        )

    with cc1:
        Concurrence.add(
            'ASW',
            #TestASW()
            sm2
        )
        Concurrence.add(
            'Commands',
            sm
        )

    with sm1:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'MAIN',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'MAIN',
            cc1,
            transitions={'succeeded': 'MAIN',
                         'aborted': 'MAIN',
                         'preempted': 'preempted'}
        )

    with sm2:
        global task
        StateMachine.add(
            'SELECT_TASK',
            SelectTask(),
            transitions={'succeeded': 'TASK'}
        )
        StateMachine.add(
            'TASK',
            SimpleActionStateName(
                'idle',
                GeneralHobbitAction,
                goal_cb=task_goal_cb,
                preempt_timeout=rospy.Duration(5),
                server_wait_timeout=rospy.Duration(10)
            ),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )

    sis = IntrospectionServer('master', sm1, '/MASTER')
    sis.start()

    outcome = sm1.execute()
    rospy.loginfo(NAME + ' returned outcome ' + str(outcome))
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.set_param('active_task', 100)
    main()

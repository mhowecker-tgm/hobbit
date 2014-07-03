#!/usr/bin/env python

PKG = 'hobbit_smach'
NAME = 'HOBBIT_MASTER'
task = 'nothing'
import roslib
roslib.load_manifest(PKG)
import rospy
<<<<<<< HEAD
<<<<<<< HEAD
from smach_ros import SimpleActionState, IntrospectionServer
from smach import StateMachine, Concurrence, State
from std_msgs.msg import String
from hobbit_msgs.msg import Command, Event, GeneralHobbitAction,\
    GeneralHobbitGoal
import hobbit_smach.helper_import as helper
import hobbit_smach.recharge_import as recharge
import uashh_smach.util as util
from hobbit_user_interaction import HobbitEmotions

commands = [['emergency', 'G_FALL', 'E_SOSBUTTON', 'C_HELP', 'E_HELP', 'C_HELP', 'F_CALLSOS', 'G_EMERGENCY'],
            ['recharge', 'E_RECHARGE', 'C_RECHARGE'],
            ['reminder', 'E_REMINDER'],
            ['stop', 'C_STOP', 'G_STOP'],
            ['call_hobbit', 'C_CALLHOBBIT', 'E_CALLHOBBIT'],
            ['call', 'E_CALLRING', 'E_CALLESTABLISHED', 'E_CALLENDED', 'C_MAKECALL'],
            ['clear_floor', 'E_CLEARFLOOR'],
            ['pickup', 'follow', 'learn_object', 'bring_object', 'goto', 'pickup'
             'C_PICKUP', 'C_FOLLOW', 'C_LEARN', 'C_BRING', 'C_GOTO', 'G_POINTING'],
            ['patrol', 'E_PATROL'],
            ['surprise', 'C_SURPRISE'],
            ['reward', 'C_REWARD', 'G_REWARD']
            ]
=======
import smach_ros
=======
from smach_ros import SimpleActionState, IntrospectionServer
>>>>>>> tried to reuse SimpleActionState based on task. Does not work this way.
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
<<<<<<< HEAD
            ['C_REWARD']]
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
=======
            ['C_REWARD', 'G_REWARD']
            ]
>>>>>>> master handles event,command data.

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
<<<<<<< HEAD
<<<<<<< HEAD
    print(msg.event)
    night = helper.IsItNight()
=======
    rospy.loginfo(str(msg))
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
=======
    print(msg.event)
>>>>>>> master handles event,command data.
    if rospy.has_param('active_task'):
        active_task = rospy.get_param('active_task')
    else:
        active_task = 100
<<<<<<< HEAD
    print(active_task)
    for index, item in enumerate(commands):
        if msg.event in item:
            if index == 4:
                ud.command = item[0]
                ud.params = msg.params
                rospy.set_param('active_task', index)
                return True
            elif index == 1 and not night:
                ud.command = 'silent_recharge'
                rospy.set_param('active_task', index)
                return True
            elif index + 1 >= active_task and not night:
=======
    for index, item in enumerate(commands):
        if msg.event in item:
            if index + 1 >= active_task:
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
                rospy.loginfo('New task has lower priority. Do nothing')
                ud.command = msg.event
                ud.params = msg.params
                return False
            else:
                rospy.loginfo('New task has higher priority. Start it.')
<<<<<<< HEAD
                if index == 7:
                    i = item.index(msg.event)
                    ud.command = item[i-6]
                    pass
                else:
                    ud.command = item[0]
                ud.params = msg.params
                if item[0] == 'stop':
                    print('Reset active_task value')
                    rospy.set_param('active_task', 100)
                else:
                    rospy.set_param('active_task', index)
                return True
    rospy.loginfo('Unknown event received %s' % msg.event)
=======
                rospy.set_param('active_task', index)
                return True
<<<<<<< HEAD
        else:
            rospy.loginfo('Unknown event received %s' % msg.event)
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
=======
    rospy.loginfo('Unknown event received %s' % msg.event)
>>>>>>> master handles event,command data.
    return False


def command_cb(msg, ud):
    rospy.loginfo('/Command data received:')
<<<<<<< HEAD
    print(msg.command)
=======
    rospy.loginfo(str(msg))
<<<<<<< HEAD
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
=======
    print(msg)
>>>>>>> master handles event,command data.
    if rospy.has_param('active_task'):
        active_task = rospy.get_param('active_task')
    else:
        active_task = 100
<<<<<<< HEAD
    print(active_task)
=======
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
    for index, item in enumerate(commands):
        if msg.command in item:
            if index + 1 >= active_task:
                rospy.loginfo('New task has lower priority. Do nothing')
                ud.command = msg.event
                ud.params = msg.params
                return False
            else:
                rospy.loginfo('New task has higher priority. Start it.')
<<<<<<< HEAD
                if index == 7:
                    i = item.index(msg.command)
                    ud.command = item[i-6]
                    pass
                else:
                    ud.command = item[0]
                ud.params = msg.params
                if item[0] == 'stop':
                    print('Reset active_task value')
                    rospy.set_param('active_task', 100)
                else:
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


def task_reminder_cb(ud, goal):
    goal = GeneralHobbitGoal(
        command=String('reminder'),
        previous_state=String('IDLE'),
        parameters=[String('command')]
    )
    return goal


def task_lo_cb(ud, goal):
    goal = GeneralHobbitGoal(
        command=String('learn_object'),
        previous_state=String('IDLE'),
        parameters=[]
    )
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
            input_keys=['command', 'params', 'active_task'],
            outcomes=['emergency',
                      'recharge',
                      'reminder',
                      'stop',
                      'call_hobbit',
                      'call',
                      'clear_floor',
                      'pickup',
                      'follow',
                      'learn_object',
                      'bring_object',
                      'goto',
                      'patrol',
                      'surprise',
                      'reward',
                      'silent_recharge',
                      'social_role',
                      'preempted',
                      'none'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('Task Selection')
        print(ud.command)
        if ud.command == 'IDLE':
            return 'none'
        return ud.command


class FakeForAllWithoutRunningActionSever(State):
    """Initialize a few data structures
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('FakeForAllWithoutRunningActionSever')
        return 'succeeded'


=======
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


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
=======
=======
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

>>>>>>> tried to reuse SimpleActionState based on task. Does not work this way.

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


>>>>>>> master handles event,command data.
def main():
    rospy.init_node(NAME)
    sm = StateMachine(
        outcomes=['succeeded',
                  'preempted',
<<<<<<< HEAD
<<<<<<< HEAD
                  'failed'],
        input_keys=['command', 'active_task', 'params'],
        output_keys=['command', 'active_task', 'params']
=======
                  'failed'],
<<<<<<< HEAD
        input_keys=['command'],
        output_keys=['command'],
>>>>>>> master handles event,command data.
=======
        input_keys=['command', 'active_task', 'params'],
        output_keys=['command', 'active_task', 'params']
>>>>>>> tried to reuse SimpleActionState based on task. Does not work this way.
    )

    sm1 = StateMachine(
        outcomes=['succeeded',
                  'preempted',
<<<<<<< HEAD
<<<<<<< HEAD
                  'failed']
    )
    sm1.userdata.command = 'IDLE'
    sm1.userdata.active_task = 'IDLE'
    sm1.userdata.params = []

    sm2 = StateMachine(
        outcomes=['succeeded',
                  'preempted',
                  'failed'],
        input_keys=['command', 'params', 'active_task'],
        output_keys=['command', 'active_task']
    )
=======
                  'failed']
    )
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
=======
                  'failed'],
        input_keys=['command'],
        output_keys=['command'],
    )
    sm1.userdata.command = 'IDLE'
>>>>>>> master handles event,command data.
=======
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
>>>>>>> tried to reuse SimpleActionState based on task. Does not work this way.

    cc = Concurrence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        default_outcome='aborted',
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        input_keys=['command', 'active_task', 'params'],
        output_keys=['command', 'params'],
        child_termination_cb=child_cb,
=======
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
=======
        input_keys=['command'],
        output_keys=['command'],
=======
        input_keys=['command', 'active_task', 'params'],
        output_keys=['command', 'params'],
>>>>>>> tried to reuse SimpleActionState based on task. Does not work this way.
        child_termination_cb=child_cb,
>>>>>>> master handles event,command data.
        outcome_map={'succeeded': {'Event_Listener': 'succeeded'},
                     'aborted': {'Event_Listener': 'aborted'}}
    )

<<<<<<< HEAD
<<<<<<< HEAD
    cc1 = Concurrence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        default_outcome='aborted',
        input_keys=['command', 'params', 'active_task'],
=======
    cc1 = Concurrence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        default_outcome='aborted',
        input_keys=['command'],
>>>>>>> master handles event,command data.
        output_keys=['command'],
        child_termination_cb=child_cb1,
        outcome_map={'succeeded': {'ASW': 'succeeded'},
                     'aborted': {'Commands': 'failed'}}
    )

<<<<<<< HEAD
=======
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
=======
>>>>>>> master handles event,command data.
    with cc:
        Concurrence.add(
            'Event_Listener',
            util.WaitForMsgState(
                '/Event',
                Event,
<<<<<<< HEAD
<<<<<<< HEAD
                msg_cb=event_cb,
                output_keys=['command', 'params']
=======
                msg_cb=event_cb
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
=======
                msg_cb=event_cb,
                output_keys=['command', 'params']
>>>>>>> master handles event,command data.
            )
        )
        Concurrence.add(
            'Command_Listener',
            util.WaitForMsgState(
                '/Command',
                Command,
<<<<<<< HEAD
<<<<<<< HEAD
                msg_cb=command_cb,
                output_keys=['command', 'params']
            )
        )

    with sm:
        StateMachine.add(
            'WAIT_FOR_E_C',
            cc,
            transitions={'succeeded': 'succeeded',
=======
                msg_cb=command_cb
=======
                msg_cb=command_cb,
                output_keys=['command', 'params']
>>>>>>> master handles event,command data.
            )
        )

    with sm:
        StateMachine.add(
            'WAIT_FOR_E_C',
            cc,
<<<<<<< HEAD
            transitions={'succeeded': 'WAIT_FOR_E_C',
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
=======
            transitions={'succeeded': 'succeeded',
>>>>>>> master handles event,command data.
                         'aborted': 'WAIT_FOR_E_C',
                         'preempted': 'preempted'}
        )

<<<<<<< HEAD
<<<<<<< HEAD
    with cc1:
        Concurrence.add(
            'ASW',
            sm2
=======
    with cc1:
        Concurrence.add(
            'ASW',
<<<<<<< HEAD
            TestASW()
>>>>>>> master handles event,command data.
=======
            #TestASW()
            sm2
>>>>>>> tried to reuse SimpleActionState based on task. Does not work this way.
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

<<<<<<< HEAD
<<<<<<< HEAD
    with sm2:
        StateMachine.add(
            'SELECT_TASK',
            SelectTask(),
            transitions={'emergency': 'EMERGENCY',
                         'recharge': 'RECHARGE',
                         'reminder': 'REMINDER',
                         'stop': 'STOP',
                         # 'call_hobbit': 'CALL_HOBBIT',
                         'call_hobbit': 'UNDOCK',
                         'call': 'CALL',
                         'clear_floor': 'CLEAR_FLOOR',
                         'pickup': 'PICKUP',
                         'follow': 'FOLLOW',
                         'learn_object': 'LEARN_OBJECT',
                         'bring_object': 'BRING_OBJECT',
                         'goto': 'GOTO',
                         'patrol': 'PATROL',
                         'surprise': 'SURPRISE',
                         'reward': 'REWARD',
                         'silent_recharge': 'SILENT_RECHARGE',
                         'social_role': 'SOCIAL_ROLE',
                         'preempted': 'preempted',
                         'none': 'succeeded'}
        )
        StateMachine.add(
            'LEARN_OBJECT',
            SimpleActionState(
                'learn_object',
                GeneralHobbitAction,
                goal_cb=task_lo_cb,
=======
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
>>>>>>> tried to reuse SimpleActionState based on task. Does not work this way.
                preempt_timeout=rospy.Duration(5),
                server_wait_timeout=rospy.Duration(10)
            ),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
<<<<<<< HEAD
        StateMachine.add(
            'REMINDER',
            SimpleActionState(
                'reminder',
                GeneralHobbitAction,
                goal_cb=task_reminder_cb,
                preempt_timeout=rospy.Duration(5),
                server_wait_timeout=rospy.Duration(10)
            ),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'EMERGENCY',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'CLEAR_FLOOR',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'PATROL',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'GOTO',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'STOP',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'CALL_HOBBIT',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'PICKUP',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'CALL',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'SURPRISE',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'FOLLOW',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'REWARD',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'BRING_OBJECT',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'RECHARGE',
            HobbitEmotions.ShowEmotions(emotion='VERY_HAPPY',
                                                 emo_time=4),
            transitions={'succeeded': 'succeeded',
                         'failed': 'failed',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SILENT_RECHARGE',
            # FakeForAllWithoutRunningActionSever(),
            recharge.getRecharge(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'SOCIAL_ROLE',
            FakeForAllWithoutRunningActionSever(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )
        StateMachine.add(
            'UNDOCK',
            recharge.getEndRecharge(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'failed'}
        )

    """
    Now we actually start the IntrospectionServer to visualize the StateMachine
    as a dot graph, and execute the main StateMachine.
    """
    sis = IntrospectionServer('master', sm1, '/MASTER')
    sis.start()
    sm1.execute()
=======
    sis = smach_ros.IntrospectionServer('master', sm, '/MASTER')
    sis.start()

    outcome = sm.execute()
    rospy.loginfo(NAME + ' returned outcome' + str(outcome))
>>>>>>> Start work on the main statemachine. All hail the puppet master. ;-)
=======
    sis = smach_ros.IntrospectionServer('master', sm1, '/MASTER')
=======

    sis = IntrospectionServer('master', sm1, '/MASTER')
>>>>>>> tried to reuse SimpleActionState based on task. Does not work this way.
    sis.start()

    outcome = sm1.execute()
    rospy.loginfo(NAME + ' returned outcome ' + str(outcome))
>>>>>>> master handles event,command data.
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.set_param('active_task', 100)
    main()

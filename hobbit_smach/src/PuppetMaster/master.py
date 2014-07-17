#!/usr/bin/env python

PKG = 'hobbit_smach'
NAME = 'HOBBIT_MASTER'

import roslib
roslib.load_manifest(PKG)
import rospy
from datetime import datetime, time
from smach_ros import SimpleActionState, IntrospectionServer
from smach import StateMachine, Concurrence, State
from std_msgs.msg import String
from hobbit_msgs.msg import Command, Event, GeneralHobbitAction,\
    GeneralHobbitGoal
import hobbit_smach.helper_import as helper
import hobbit_smach.recharge_import as recharge
import hobbit_smach.call_hobbit_import as call_hobbit
import uashh_smach.util as util
from hobbit_user_interaction import HobbitEmotions


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

commands = [['emergency', 'G_FALL', 'E_SOSBUTTON', 'C_HELP', 'E_HELP', 'G_HELP', 'A_HELP','C_HELP', 'F_CALLSOS', 'G_EMERGENCY'],
            ['recharge', 'E_RECHARGE', 'C_RECHARGE'],
            ['reminder', 'E_REMINDER'],
            ['stop', 'C_STOP', 'G_STOP', 'E_STOP'],
            ['call_hobbit', 'C_CALLHOBBIT', 'E_CALLHOBBIT'],
            ['call', 'E_CALLRING', 'E_CALLESTABLISHED', 'E_CALLENDED', 'C_MAKECALL'],
            ['away', 'C_AWAY1', 'C_AWAY2', 'C_AWAY3', 'C_AWAY4', 'C_AWAY5', 'C_AWAY6',
            'C_SLEEP1','C_SLEEP2','C_SLEEP3','C_SLEEP4','C_SLEEP5','C_SLEEP6',],
            ['clear_floor', 'E_CLEARFLOOR'],
            ['pickup', 'follow', 'learn_object', 'bring_object', 'goto', 'pickup'
             'C_PICKUP', 'C_FOLLOW', 'C_LEARN', 'C_BRING', 'C_GOTOPOINT', 'G_POINTING'],
            ['patrol', 'E_PATROL'],
            ['surprise', 'C_SURPRISE'],
            ['reward', 'C_REWARD', 'G_REWARD']
            ]


def IsItNight(ud):
    sleep_time = ud.parameters['sleep_time']
    wakeup_time = ud.parameters['wakeup_time']

    wake = wakeup_time.split(':')
    sleep = sleep_time.split(':')
    now = datetime.now()
    if time(int(wake[0]), int(wake[1]))\
            <= now.time()\
            <= time(int(sleep[0]), int(sleep[1])):
        return False
    else:
        return True


def command_cb(msg, ud):
    try:
        print msg.command
        input_ce = msg.command
        rospy.loginfo('/Command data received:')
    except AttributeError, e:
        print(e)
        pass
    try:
        print msg.event
        input_ce = msg.event
        rospy.loginfo('/Event data received:')
    except AttributeError, e:
        print(e)
        pass

    night = IsItNight(ud)
    rospy.sleep(2.0)
    print('active_task and night')
    active_task = ud.parameters['active_task']
    print(active_task)
    print(night)
    for index, item in enumerate(commands):
        print(input_ce)
        if input_ce in item:
            # if index == 4:
            if item[0] == 'call_hobbit':
                ud.command = item[0]
                ud.params = msg.params
                print(msg.params)
                ud.parameters['active_task'] = index
                return True
            # elif index == 1 and night and index + 1 <= active_task:
            elif item[0] == 'recharge' and not night and index  <= active_task:
                print('RECHARGING')
                ud.command = 'recharge'
                ud.parameters['active_task'] = index
                return True
            elif index + 1 >= active_task and not night:
                rospy.loginfo('New task has lower priority. Do nothing')
                print(bcolors.OKGREEN +
                      'New task has lower priority. Do nothing' +
                      bcolors.ENDC)
                return False
            else:
                rospy.loginfo('New task has higher priority. Start it.')
                print(bcolors.OKGREEN +
                      'New task has higher priority. Start it.' +
                      bcolors.ENDC)
                # if index == 7:
                if item[0] == 'pickup':
                    i = item.index(input_ce)
                    ud.command = item[i - 5]
                    print('COMMAND')
                    print(item[i-5])
                    pass
                else:
                    ud.command = item[0]
                ud.params = msg.params
                if item[0] == 'stop':
                    print('Reset active_task value')
                    ud.parameters['active_task'] = 100
                else:
                    ud.parameters['active_task'] = index
                return True
    rospy.loginfo('Unknown event/command received %s' % input_ce)
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


class ResetActiveTask(State):
    """Initialize a few data structures
    """
    def __init__(self):
        State.__init__(
            self,
            input_keys=['parameters'],
            outcomes=['succeeded', 'preempted', 'aborted'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('ResetActiveTask')
        rospy.set_param('active_task', 100)
        ud.parameters['active_task']  = 100
        return 'succeeded'


class Init(State):
    """Initialize a few data structures
    """
    def __init__(self):
        State.__init__(
            self,
            input_keys=['parameters', 'params', 'command', 'active_task'],
            output_keys=['parameters', 'params'],
            outcomes=['succeeded', 'preempted'])
        # load a few needed parameters from server
        self.sleep_time = rospy.get_param('/sleep_time', '22:00')
        self.wakeup_time = rospy.get_param('/wakeup_time', '06:30')
        self.active_task = 100

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.parameters = {}
        ud.parameters['sleep_time'] = self.sleep_time
        ud.parameters['wakeup_time'] = self.wakeup_time
        ud.parameters['active_task'] = self.active_task
        print('Init')
        return 'succeeded'


class SelectTask(State):
    """Select the task for execution
    """
    def __init__(self):
        State.__init__(
            self,
            input_keys=['command', 'params', 'active_task', 'parameters'],
            outcomes=['emergency',
                      'recharge',
                      'reminder',
                      'stop',
                      'call_hobbit',
                      'call',
                      'away',
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
    def __init__(self, name=None):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted', 'failed'])
        if name == None:
            self.name = 'HOW_DID_WE_END_HERE?'
        else:
            self.name = name

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('FakeForAllWithoutRunningActionSever')
        print(self.name)
        return 'succeeded'


def away_cb(ud, goal):
    par = []
    par.append(String('2'))
    goal = GeneralHobbitGoal(command=String('away'),
                             previous_state=String('call_hobbit'),
                             parameters=par)
    print goal
    return goal


def goto_cb(ud, goal):
    room, place = ud.params[0].value.lower().split(' ')
    par = []
    par.append(String(room))
    par.append(String(place))
    goal = GeneralHobbitGoal(command=String('goto'),
                             previous_state=String('previous_task'),
                                 parameters=par)
    print goal
    return goal


def sos_cb(ud, goal):
    par = []
    par.append(String('user_initiated'))
    goal = GeneralHobbitGoal(command=String('Emergency'),
                             parameters=par)
    return goal


def main():
    rospy.init_node(NAME)
    sm = StateMachine(
        outcomes=['succeeded',
                  'preempted',
                  'failed'],
        input_keys=['command', 'active_task', 'params', 'parameters'],
        output_keys=['command', 'active_task', 'params', 'parameters']
    )

    sm1 = StateMachine(
        input_keys=['command', 'params', 'parameters', 'active_task'],
        outcomes=['succeeded',
                  'preempted',
                  'failed']
    )
    sm1.userdata.command = 'IDLE'
    sm1.userdata.active_task = 'CALL'
    # sm1.userdata.params = []

    sm2 = StateMachine(
        outcomes=['succeeded',
                  'preempted',
                  'failed'],
        input_keys=['command', 'params', 'active_task', 'parameters'],
        output_keys=['command', 'active_task', 'parameters', 'params']
    )
    def cc_out_cb(outcome_map):
        if outcome_map['Event_Listener'] == 'succeeded':
            return 'succeeded'
        elif outcome_map['Command_Listener'] == 'succeeded':
            return 'succeeded'
        else: 
            return 'aborted'

    cc = Concurrence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        default_outcome='aborted',
        input_keys=['command', 'active_task', 'params', 'parameters'],
        output_keys=['command', 'params', 'parameters'],
        child_termination_cb=child_cb,
        outcome_cb=cc_out_cb,
        # outcome_map={'succeeded': {'Event_Listener': 'succeeded'},
        #             'aborted': {'Event_Listener': 'aborted'}}
    )

    cc1 = Concurrence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        default_outcome='aborted',
        input_keys=['command', 'params', 'active_task', 'parameters'],
        output_keys=['command', 'params', 'parameters'],
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
                msg_cb=command_cb,
                input_keys=['parameters', 'params', 'command', 'active_task'],
                output_keys=['parameters', 'params', 'command', 'active_task']
            )
        )
        Concurrence.add(
            'Command_Listener',
            util.WaitForMsgState(
                '/Command',
                Command,
                msg_cb=command_cb,
                input_keys=['parameters', 'params', 'command', 'active_task'],
                output_keys=['parameters', 'params', 'command', 'active_task']
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
        # sm2.userdata.room_name = 'dock'
        # sm2.userdata.location_name = 'dock'
        StateMachine.add(
            'SELECT_TASK',
            SelectTask(),
            transitions={'emergency': 'EMERGENCY',
                         'recharge': 'RECHARGE',
                         'reminder': 'REMINDER',
                         'stop': 'STOP',
                         'call_hobbit': 'CALL_HOBBIT',
                         'call': 'CALL',
                         'clear_floor': 'CLEAR_FLOOR',
                         'away': 'AWAY',
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
            # FakeForAllWithoutRunningActionSever(name='LEARN_OBJECT'),
            SimpleActionState(
                'learn_object',
                GeneralHobbitAction,
                goal_cb=task_lo_cb,
                preempt_timeout=rospy.Duration(5),
                server_wait_timeout=rospy.Duration(10)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'REMINDER',
            # FakeForAllWithoutRunningActionSever(name='REMINDER'),
            SimpleActionState(
                'reminder',
                GeneralHobbitAction,
                goal_cb=task_reminder_cb,
                preempt_timeout=rospy.Duration(5),
                server_wait_timeout=rospy.Duration(10)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'EMERGENCY',
            # FakeForAllWithoutRunningActionSever(name='EMERGENCY'),
            SimpleActionState('emergency_user',
                              GeneralHobbitAction,
                              goal_cb=sos_cb,
                              input_keys=['parameters', 'params']),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'CLEAR_FLOOR',
            FakeForAllWithoutRunningActionSever(name='CLEAR_FLOOR'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'PATROL',
            FakeForAllWithoutRunningActionSever(name='PATROL'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'GOTO',
            # FakeForAllWithoutRunningActionSever(name='GOTO'),
            SimpleActionState('goto',
                              GeneralHobbitAction,
                              goal_cb=goto_cb,
                              input_keys=['parameters', 'params']),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'STOP',
            helper.get_hobbit_full_stop(),
            # FakeForAllWithoutRunningActionSever(name='STOP'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         # 'aborted': 'failed',
                         'failed': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'CALL_HOBBIT',
            call_hobbit.call_hobbit(),
            # FakeForAllWithoutRunningActionSever(name='CALL_HOBBIT'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'PICKUP',
            FakeForAllWithoutRunningActionSever(name='PICKUP'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'CALL',
            FakeForAllWithoutRunningActionSever(name='CALL'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'SURPRISE',
            FakeForAllWithoutRunningActionSever(name='SURPRISE'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'FOLLOW',
            FakeForAllWithoutRunningActionSever(name='FOLLOW'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'REWARD',
            HobbitEmotions.ShowEmotions(emotion='HAPPY',
                                                 emo_time=4),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'failed': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'BRING_OBJECT',
            FakeForAllWithoutRunningActionSever(name='BRING_OBJECT'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK',
                         'failed': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'RECHARGE',
            # FakeForAllWithoutRunningActionSever(name='RECHARGE'),
            recharge.getRecharge(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         #'failed': 'failed',
                         'aborted': 'RESET_ACTIVE_TASK',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SILENT_RECHARGE',
            # FakeForAllWithoutRunningActionSever(name='SILENT_RECHARGE'),
            recharge.getRecharge(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'AWAY',
            # FakeForAllWithoutRunningActionSever(name='GOTO'),
            SimpleActionState('away',
                              GeneralHobbitAction,
                              goal_cb=away_cb,
                              input_keys=['parameters', 'params']),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'SOCIAL_ROLE',
            FakeForAllWithoutRunningActionSever(name='SOCIAL_ROLE'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'RESET_ACTIVE_TASK',
            ResetActiveTask(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )

    """
    Now we actually start the IntrospectionServer to visualize the StateMachine
    as a dot graph, and execute the main StateMachine.
    """
    sis = IntrospectionServer('master', sm1, '/MASTER')
    sis.start()
    sm1.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.set_param('active_task', 100)
    main()

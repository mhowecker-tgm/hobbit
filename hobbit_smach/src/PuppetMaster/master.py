#!/usr/bin/env python

PKG = 'hobbit_smach'
NAME = 'hobbit_master'

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
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.social_role_import as social_role
import uashh_smach.util as util
from hobbit_user_interaction import HobbitMMUI
from hobbit_smach.bcolors import bcolors


commands = [['emergency', 'G_FALL', 'E_SOSBUTTON', 'C_HELP', 'E_HELP',
             'G_HELP', 'A_HELP', 'C_HELP', 'F_CALLSOS', 'G_EMERGENCY'],
            ['recharge', 'E_RECHARGE', 'C_RECHARGE'],
            ['reminder', 'E_REMINDER'],
            ['stop', 'C_STOP', 'G_STOP', 'E_STOP', 'E_CANCEL'],
            ['call_hobbit', 'C_CALLHOBBIT', 'E_CALLHOBBIT'],
            ['call', 'E_CALLRING', 'E_CALLESTABLISHED', 'E_CALLENDED', 'C_MAKECALL'],
            ['away', 'C_AWAY1', 'C_AWAY2', 'C_AWAY3', 'C_AWAY4', 'C_AWAY5', 'C_AWAY6',
            'C_SLEEP1', 'C_SLEEP2', 'C_SLEEP3', 'C_SLEEP4', 'C_SLEEP5', 'C_SLEEP6'],
            ['clear_floor', 'E_CLEARFLOOR'],
            ['pickup', 'follow', 'learn_object', 'bring_object', 'goto', 'pickup',
             'C_PICKUP', 'C_FOLLOW', 'C_LEARN', 'C_BRING', 'C_GOTOPOINT', 'G_POINTING'],
            ['patrol', 'E_PATROL'],
            ['surprise', 'C_SURPRISE'],
            ['reward', 'C_REWARD', 'G_REWARD'],
            ['social_role', 'E_SOCIALROLE'],
            ['master_reset', 'C_MASTER_RESET']
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
        rospy.loginfo('/Command data received:')
        # rospy.loginfo(str(msg.command))
        input_ce = msg.command.upper()
    except AttributeError, e:
        print(e)
        pass
    try:
        rospy.loginfo('/Event data received:')
        # rospy.loginfo(str(msg.event))
        input_ce = msg.event.upper()
    except AttributeError, e:
        print(e)
        pass

    night = IsItNight(ud)
    active_task = ud.parameters['active_task']
    for index, item in enumerate(commands):
        if input_ce in item:
            if item[0] == 'master_reset':
                    rospy.loginfo('Master RESET activated')
                    ud.parameters['active_task'] = 100
                    ud.command = 'master_reset'
                    return True
            elif item[0] == 'call_hobbit':
                ud.command = item[0]
                for i, v in enumerate(msg.params):
                    # print(v.name)
                    # print(v.value)
                    if v.name == 'bathroom' and v.value == 'true':
                        ud.command = 'emergency_bathroom'
                        ud.parameters['active_task'] = 'emergency_bathroom'
                        ud.params = msg.params
                        return True
                ud.params = msg.params
                ud.parameters['active_task'] = index
                return True
            elif item[0] == 'emergency':
                ud.parameters['active_task'] = index
                ud.command = item[0]
                return True
            elif item[0] == 'away' or item[0] == 'sleep':
                times = [1, 2, 4, 6, 12, 24]
                # print(input_ce)
                index = int(input_ce[-1:]) - 1
                print(times[index])
                if input_ce[:-1] == 'SLEEP':
                    ud.command = 'sleep'
                else:
                    ud.command = 'away'
                ud.parameters['sleep_time'] = times[index]
                return True
            # elif index == 1 and night and index + 1 <= active_task:
            elif item[0] == 'recharge' and not night and index < active_task:
                rospy.loginfo('RECHARGING')
                ud.command = 'recharge'
                ud.parameters['active_task'] = index
                return True
            elif index + 1 >= active_task and not night:
                rospy.loginfo('New task has lower priority. DO NOTHING')
                print(bcolors.FAIL + \
                    'New task has lower priority. Do nothing' \
                    + bcolors.ENDC)
                return False
            else:
                rospy.loginfo('New task has higher priority. START IT.')
                print(bcolors.OKGREEN +\
                    'New task has higher priority. Start it.'\
                    + bcolors.ENDC)
                # if index == 7:
                if item[0] == 'pickup':
                    i = item.index(input_ce)
                    ud.command = item[i - 6]
                else:
                    ud.command = item[0]
                ud.params = msg.params
                if item[0] == 'stop':
                    rospy.loginfo('Reset active_task value')
                    ud.parameters['active_task'] = 100
                else:
                    ud.parameters['active_task'] = index
                return True
    rospy.loginfo('Unknown event/command received %s' % input_ce)
    return False


def child_cb(outcome_map):
    rospy.loginfo('Event Command cc child termination callback')
    return True


def child_cb1(outcome_map):
    rospy.loginfo('cc1 child termination callback')
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
            input_keys=['parameters', 'command'],
            output_keys=['parameters', 'command'],
            outcomes=['succeeded', 'preempted', 'aborted'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('ResetActiveTask')
        rospy.set_param('active_task', 100)
        ud.parameters['active_task'] = 100
        ud.command = ''
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
        ud.params = []
        rospy.loginfo('Init')
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
                      'emergency_bathroom',
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
                      'master_reset',
                      'preempted',
                      'sleep',
                      'none'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('Task Selection')
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
        if name is None:
            self.name = 'HOW_DID_WE_END_HERE?'
        else:
            self.name = name

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('FakeForAllWithoutRunningActionSever')
        rospy.loginfo(self.name)
        return 'succeeded'


def sleep_cb(ud, goal):
    par = []
    par.append(String(ud.parameters['sleep_time']))
    goal = GeneralHobbitGoal(command=String('sleep'),
                             previous_state=String('call_hobbit'),
                             parameters=par)
    return goal


def bring_cb(ud, goal):
    par = []
    par.append({'object_name': ud.params[0].value})
    # par.append(String(ud.params[0].value))
    rospy.loginfo('bring_cb: %s' % ud.params[0].value)
    goal = GeneralHobbitGoal(command=String('bring_object'),
                             previous_state=String('call_hobbit'),
                             parameters=par)
    return goal


def away_cb(ud, goal):
    par = []
    par.append(String(ud.parameters['sleep_time']))
    goal = GeneralHobbitGoal(command=String('away'),
                             previous_state=String('call_hobbit'),
                             parameters=par)
    return goal


def patrol_cb(ud, goal):
    goal = GeneralHobbitGoal(command=String('locateUser'),
                             previous_state=String('call_hobbit'),
                             parameters=[])
    return goal


def follow_cb(ud, goal):
    room, place = ud.params[0].value.lower().split(' ')
    if not room or not place:
        room = 'corridor'
        place = 'default'
    par = []
    par.append(String(room))
    par.append(String(place))
    goal = GeneralHobbitGoal(command=String('follow'),
                             previous_state=String('previous_task'),
                             parameters=par)
    return goal


def goto_cb(ud, goal):
    room, place = ud.params[0].value.lower().split(' ')
    par = []
    par.append(String(room))
    par.append(String(place))
    goal = GeneralHobbitGoal(command=String('goto'),
                             previous_state=String('previous_task'),
                             parameters=par)
    ud.goal_room = room
    rospy.loginfo('Going to room: '+str(room))
    return goal


def sos_cb(ud, goal):
    par = []
    par.append(String('user_initiated'))
    goal = GeneralHobbitGoal(
        command=String('Emergency'),
        parameters=par)
    return goal


def pickup_cb(ud, goal):
    goal = GeneralHobbitGoal(command=String('pickup'))
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
        # input_keys=['command', 'params', 'parameters', 'active_task'],
        outcomes=['succeeded',
                  'preempted',
                  'failed']
    )
    sm1.userdata.command = 'IDLE'
    sm1.userdata.active_task = 'CALL'
    sm1.userdata.params = []

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
                         'emergency_bathroom': 'EMERGENCY_BATHROOM',
                         'call': 'CALL',
                         'clear_floor': 'CLEAR_FLOOR',
                         'away': 'AWAY',
                         'sleep': 'SLEEP',
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
                         'master_reset': 'RESET_ACTIVE_TASK',
                         'preempted': 'preempted',
                         'none': 'succeeded'}
        )
        StateMachine.add(
            'LEARN_OBJECT',
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
        StateMachine.add_auto(
            'EMERGENCY',
            SimpleActionState('emergency_user',
                              GeneralHobbitAction,
                              goal_cb=sos_cb,
                              input_keys=['parameters', 'params']),
            connector_outcomes=['succeeded', 'aborted', 'preempted']
        )
        StateMachine.add(
            'CLEAR_FLOOR',
            FakeForAllWithoutRunningActionSever(name='CLEAR_FLOOR'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'PATROL',
            SimpleActionState('locate_user',
                              GeneralHobbitAction,
                              goal_cb=patrol_cb,
                              input_keys=['parameters', 'params']),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'GOTO',
            SimpleActionState('goto',
                              GeneralHobbitAction,
                              goal_cb=goto_cb,
                              input_keys=['parameters', 'params'],
                              output_keys=['goal_room']),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'STOP',
            # FakeForAllWithoutRunningActionSever(name='STOP'),
            helper.get_hobbit_full_stop(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'CALL_HOBBIT',
            call_hobbit.call_hobbit(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'EMERGENCY_BATHROOM',
            SimpleActionState('emergency_bathroom',
                              GeneralHobbitAction,
                              goal_cb=away_cb,
                              input_keys=['parameters', 'params']),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'PICKUP',
            SimpleActionState('pickup',
                              GeneralHobbitAction,
                              goal_cb=pickup_cb,
                              input_keys=['parameters', 'params']),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'CALL',
            # There is nothing to do during a phone call,
            # so we just wait and do nothing.
            FakeForAllWithoutRunningActionSever(name='CALL'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'SURPRISE',
            social_role.get_surprise(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'FOLLOW',
            SimpleActionState('follow',
                              GeneralHobbitAction,
                              goal_cb=follow_cb,
                              input_keys=['parameters', 'params']),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add_auto(
            'REWARD',
            speech_output.emo_say_something(
                emo='VHAPPY',
                time=4,
                text='T_RW_YouAreWelcome'
            ),
            connector_outcomes=['succeeded', 'aborted'])
        StateMachine.add(
            'MAIN_MENU',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'failed': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'BRING_OBJECT',
            FakeForAllWithoutRunningActionSever(name='BRING_OBJECT'),
            # SimpleActionState('bring_object',
            #                  GeneralHobbitAction,
            #                  goal_cb=bring_cb,
            #                  input_keys=['parameters', 'params']
            #                  ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK',
                         'failed': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'RECHARGE',
            recharge.getRecharge(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         # 'failed': 'failed',
                         'aborted': 'RESET_ACTIVE_TASK',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SILENT_RECHARGE',
            recharge.getRecharge(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'AWAY',
            SimpleActionState('away',
                              GeneralHobbitAction,
                              goal_cb=away_cb,
                              input_keys=['parameters', 'params']),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'SLEEP',
            SimpleActionState('sleep',
                              GeneralHobbitAction,
                              goal_cb=sleep_cb,
                              input_keys=['parameters', 'params']),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'SOCIAL_ROLE',
            # FakeForAllWithoutRunningActionSever(name='SOCIAL_ROLE'),
            social_role.get_social_role_change(),
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

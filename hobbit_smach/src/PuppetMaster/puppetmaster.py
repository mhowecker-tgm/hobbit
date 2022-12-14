#!/usr/bin/env python

PKG = 'hobbit_smach'
NAME = 'puppetmaster'
PREEMPT_TIMEOUT = 5
SERVER_TIMEOUT = 5
global MUC_ENABLED

import rospy
from datetime import datetime, time
from smach_ros import SimpleActionState, IntrospectionServer, ServiceState, MonitorState
from smach import StateMachine, Concurrence, State
from std_msgs.msg import String
from hobbit_msgs.msg import Command, Event, GeneralHobbitAction,\
    GeneralHobbitGoal
from hobbit_msgs.srv import GetAwayState, SetAwayState, GetAwayStateRequest, SetAwayStateRequest, \
    GetSafetycheckStateRequest, GetSafetycheckState
import hobbit_smach.helper_import as helper
import hobbit_smach.recharge_import as recharge
import hobbit_smach.call_hobbit_2_import as call_hobbit
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.social_role_import as social_role
import hobbit_smach.end_interaction_import as end_interaction
import hobbit_smach.logging_import as log
import uashh_smach.util as util
from hobbit_msgs import MMUIInterface as MMUI
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
from hobbit_smach.bcolors import bcolors
import hobbit_smach.arm_move_import as arm_move
import hobbit_smach.phone_call_import as phone_call
import hobbit_smach.safety_check_import as safety_check

new_command = None
new_params = None

commands = [
    ['emergency', 'G_FALL', 'E_SOSBUTTON', 'C_HELP', 'E_HELP',
     'G_HELP', 'A_HELP', 'C_HELP', 'F_CALLSOS', 'G_EMERGENCY'],
    ['recharge', 'E_RECHARGE', 'A_Recharge'],
    ['silent_recharge', 'C_RECHARGE'],
    ['reminder', 'E_REMINDER'],
    ['stop', 'C_STOP', 'G_STOP', 'E_STOP', 'B_CANCEL', 'E_CANCEL', 'P_E_CANCEL', 'A_stop', 'A_cancel'], #'E_FITNESS_CLOSED'],
    ['call_hobbit', 'C_CALLHOBBIT', 'E_CALLHOBBIT'],
    ['call', 'E_CALLRING', 'E_CALLESTABLISHED', 'C_MAKECALL'],
    ['away', 'C_AWAY1', 'C_AWAY2', 'C_AWAY3', 'C_AWAY4', 'C_AWAY5', 'C_AWAY6',
     'C_SLEEP1', 'C_SLEEP2', 'C_SLEEP3', 'C_SLEEP4', 'C_SLEEP5', 'C_SLEEP6'],
    ['clear_floor', 'E_CLEARFLOOR'],
    ['pickup', 'follow', 'learn_object', 'bring_object', 'goto', 'pickup',
     'C_PICKUP', 'C_FOLLOW', 'C_LEARN', 'C_BRING', 'C_GOTOPOINT', 'G_POINTING'],
    ['patrol', 'E_PATROL'],
    ['surprise', 'C_SURPRISE', 'A_surprise'],
    ['reward', 'C_REWARD', 'G_REWARD', 'A_welldone'],
    ['social_role', 'E_SOCIALROLE'],
    ['master_reset', 'C_MASTER_RESET'],
    ['fitness', 'B_FITNESS']
    ]


def IsItNight(ud):
    global wakeup_time
    global sleep_time
    try:
        sleep = sleep_time.split(':')
        wakeup = wakeup_time.split(':')
    except Exception:
        rospy.loginfo("Exception thrown while trying to use sleep/wakeup time in puppetmaster. Using default values.")
        wakeup = [9, 00]
        sleep = [21, 00]

    wake = wakeup_time.split(':')
    sleep = sleep_time.split(':')
    now = datetime.now()

    if time(int(wake[0]), int(wake[1]))\
            <= now.time()\
            <= time(int(sleep[0]), int(sleep[1])):
        return False
    else:
        return True

def start_command(ud):
    mmui = MMUI.MMUIInterface()
    mmui.remove_last_prompt()
    rospy.loginfo('New task has higher priority. START IT.')
    rospy.loginfo('start_command: '+str(ud.params))
    # print(bcolors.OKGREEN +
    #       'New task has higher priority. Start it.'
    #       + bcolors.ENDC)
    return True

def is_the_user_away():
    get_user_state = rospy.ServiceProxy(
            '/user/get_away_state',
            GetAwayState,
            persistent=False)
    try:
        request=GetAwayStateRequest(state=True)
        resp = get_user_state(request)
        #rospy.loginfo("/user/get_away_status returned: "+str(type(resp.result))+", "+str(resp.result))
        return resp.result
    except rospy.ServiceException:
        return False

def the_user_is_back():
    set_user_state = rospy.ServiceProxy(
            '/user/set_away_state',
            SetAwayState,
            persistent=False)
    try:
        request=SetAwayStateRequest(state=False)
        resp = set_user_state(request)
        return resp.result
    except rospy.ServiceException:
        return False

def command_cb(ud, msg):
    res = command_cb2(msg, ud)
    return not res

def command_cb2(msg, ud):
    global new_command
    global new_params
    try:
        rospy.loginfo(str(msg.command))
        input_ce = msg.command.upper()
        #rospy.loginfo('/Command data received:')
    except AttributeError, e:
        #rospy.loginfo("command_cb: Command: "+str(e))
        pass
    try:
        rospy.loginfo(str(msg.event))
        input_ce = msg.event.upper()
        #rospy.loginfo('/Event data received:')
        if msg.event == 'E_WATCHDOG':
            pass
            #rospy.loginfo(str(msg.header))
    except AttributeError, e:
        #rospy.loginfo("command_cb: Event: "+str(e))
        pass

    night = IsItNight(ud)
    away = is_the_user_away()
    active_task = ud.parameters['active_task']
    first = True
    if away and input_ce not in ['E_HELP', 'C_CALLHOBBIT', 'E_CALLHOBBIT', 'E_WAKEUP', 'C_GOTOPOINT']:
        rospy.loginfo('The user is away and the command is not there to wake up Hobbit.')
        return False
    the_user_is_back()
    for index, item in enumerate(commands):
        if first and active_task < 100:
            rospy.loginfo('GOT COMMAND: '+str(input_ce))
            rospy.loginfo('CURRENTLY RUNNING TASK LEVEL: '+ str(active_task)+' '+commands[active_task][0]+' or similar')
            first = False
        if input_ce in item:
            if item[0] == 'master_reset':
                    rospy.loginfo('Master RESET activated')
                    ud.parameters['active_task'] = 100
                    ud.command = 'master_reset'
                    new_command = ud.command
                    return start_command(ud)
            elif item[0] == 'stop':
                rospy.loginfo('Reset active_task value')
                ud.parameters['active_task'] = 100
                ud.command = 'stop'
                arm_move.do_stop()
                new_command = ud.command
                return start_command(ud)
            elif item[0] == 'call_hobbit':
                rospy.loginfo('CALL_BUTTON msg: '+str(msg))
                rospy.loginfo('CALL_BUTTON msg params: '+str(msg.params))
                ud.command = item[0]
                for i, v in enumerate(msg.params):
                    if v.name == 'bathroom' and v.value == 'true':
                        ud.command = 'emergency_bathroom'
                        ud.parameters['active_task'] = 'emergency_bathroom'
                        ud.params = msg.params
                        new_params = ud.params
                        new_command = ud.command
                        return start_command(ud)
                rospy.loginfo('before setting ud.params from msg.params')
                rospy.loginfo('ud.params: '+str(ud.params)+' msg.params: '+str(msg.params))
                ud.params = msg.params
                new_params = ud.params
                rospy.loginfo('setting ud.params from msg.params')
                rospy.loginfo('ud.params: '+str(ud.params)+' msg.params: '+str(msg.params))
                rospy.loginfo('new_params: '+str(new_params))
                ud.parameters['active_task'] = index
                new_command = ud.command
                return start_command(ud)
            elif item[0] == 'emergency' and active_task > 0:
                rospy.loginfo('emergency. active_task = ' + str(active_task))
                ud.parameters['active_task'] = index
                ud.command = item[0]
                ud.emergency = True
                new_command = ud.command
                return start_command(ud)
            elif item[0] == 'away' or item[0] == 'sleep':
                times = [1, 2, 4, 6, 12, 24]
                index = int(input_ce[-1:]) - 1
                print(times[index])
                if input_ce[:-1] == 'SLEEP':
                    ud.command = 'sleep'
                else:
                    ud.command = 'away'
                new_params = str(times[index])
                new_command = ud.command
                return start_command(ud)
            elif item[0] == 'recharge' and not night and index < active_task:
                rospy.loginfo('RECHARGING')
                ud.command = 'recharge'
                ud.parameters['active_task'] = index
                new_command = ud.command
                return start_command(ud)
            elif index + 1 >= active_task and not night:
                rospy.loginfo('New task has lower priority. DO NOTHING')
                print(bcolors.FAIL +
                      'New task has lower priority. Do nothing'
                      + bcolors.ENDC)
                return False
            else:
                if item[0] == 'pickup':
                    i = item.index(input_ce)
                    ud.command = item[i - 6]
                    rospy.loginfo('should be pickup now')
                else:
                    ud.command = item[0]
                ud.params = msg.params
                new_params = ud.params
                new_command = ud.command
                return start_command(ud)
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


class ResetEmergency(State):
    """Initialize a few data structures
    """
    def __init__(self):
        State.__init__(
            self,
            input_keys=['parameters', 'command', 'emergency'],
            output_keys=['parameters', 'command', 'emergency'],
            outcomes=['succeeded', 'preempted', 'aborted'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('ResetActiveTask')
        rospy.set_param('active_task', 100)
        ud.parameters['active_task'] = 100
        ud.command = ''
        ud.emergency = False
        return 'succeeded'


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
        rospy.loginfo('ResetActiveTask')
        rospy.set_param('active_task', 100)
        ud.parameters['active_task'] = 100
        ud.command = ''
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
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
    """
    Select the task for execution
    """
    def __init__(self):
        State.__init__(
            self,
            input_keys=['command', 'params', 'active_task', 'parameters', 'emergency'],
            output_keys=['command', 'params'],
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
                      'fitness',
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
        #rospy.loginfo('Task Selection' +str(ud.params))
        global new_command
        global new_params
        #rospy.loginfo('Task Selection: '+str(new_command))
        #rospy.loginfo('Task Selection: '+str(new_params))
        ud.params = new_params
        #rospy.loginfo('Task Selection'+str(ud.params))
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if not new_command:
            return 'none'
        rospy.loginfo("SelectTask: "+str(new_command))
        if new_command == 'IDLE':
            return 'none'
        ret = new_command
        return ret


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
        rospy.loginfo('FakeForAllWithoutRunningActionSever')
        for i in xrange(1,10):
            rospy.sleep(1)
            rospy.loginfo(self.name+str(i))
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
        rospy.loginfo(self.name)
        return 'succeeded'


class Emergency(State):
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
        rospy.loginfo('FakeForAllWithoutRunningActionSever')
        rospy.sleep(10)
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
    par.append(String(new_params[0].value))
    rospy.loginfo('bring_cb: %s' % new_params[0].value)
    goal = GeneralHobbitGoal(command=String('bring_object'),
                             previous_state=String(''),
                             parameters=par)
    return goal

def emergency_bathroom_cb(ud, goal):
    rospy.loginfo('emergency_bathroom_cb')
    par = []
    par.append(String('bathroom sos button'))
    goal = GeneralHobbitGoal(
        command=String('Emergency'),
        parameters=par)


def away_cb(ud, goal):
    par = []
    rospy.loginfo("away_cb")
    rospy.loginfo(str(type(new_params)))
    rospy.loginfo((new_params))
    par.append(String(new_params))
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
    goal = GeneralHobbitGoal(command=String('follow'),
                             previous_state=String('previous_task'),
                             parameters=[])
    return goal


def goto_cb(ud, goal):
    room, place = new_params[0].value.lower().split(' ')
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

def sf_resp_cb(ud, resp):
    if resp.result == True:
        return 'succeeded' 
    else:
        return 'aborted'

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
        outcomes=['succeeded',
                  'preempted',
                  'failed']
    )
    sm1.userdata.command = 'IDLE'
    sm1.userdata.active_task = 'CALL'
    #sm1.userdata.params = []

    sm2 = StateMachine(
        outcomes=['succeeded',
                  'preempted',
                  'failed'],
        input_keys=['command', 'params', 'active_task', 'parameters'],
        output_keys=['command', 'params', 'active_task', 'parameters']
    )

    def cc_out_cb(outcome_map):
        if outcome_map['Event_Listener'] == 'invalid':
            return 'succeeded'
        elif outcome_map['Command_Listener'] == 'invalid':
            return 'succeeded'
        else:
            return 'aborted'

    cc = Concurrence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        default_outcome='aborted',
        input_keys=['command', 'active_task', 'params', 'parameters'],
        output_keys=['command', 'params', 'parameters'],
        child_termination_cb=child_cb,
        outcome_cb=cc_out_cb
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
            MonitorState(
            #util.WaitForMsgState(
                '/Event',
                Event,
                command_cb,
                #msg_cb=command_cb,
                input_keys=['parameters', 'params', 'command', 'active_task'],
                output_keys=['parameters', 'params', 'command', 'active_task', 'emergency']
            )
        )
        Concurrence.add(
            'Command_Listener',
            MonitorState(
            #util.WaitForMsgState(
                '/Command',
                Command,
                command_cb,
                #msg_cb=command_cb,
                input_keys=['parameters', 'params', 'command', 'active_task'],
                output_keys=['parameters', 'params', 'command', 'active_task', 'emergency']
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
        sm2.userdata.scenario = 'IDLE'
        sm2.userdata.emergency = False
        StateMachine.add_auto(
            'LOG_TASK_STARTED',
            log.DoLogStart(),
            connector_outcomes=['succeeded', 'aborted']
        )
        StateMachine.add_auto(
            'EMO',
            HobbitEmotions.ShowEmotions(
                emotion='NEUTRAL',
                emo_time=0),
            connector_outcomes=['succeeded', 'failed']
        )
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
                         'fitness': 'FITNESS',
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
            'REMINDER',
            SimpleActionState(
                'reminder',
                GeneralHobbitAction,
                goal_cb=task_reminder_cb,
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'EMERGENCY',
            SimpleActionState(
                'emergency_user',
                GeneralHobbitAction,
                goal_cb=sos_cb,
                input_keys=['parameters', 'params'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'CLEAR_FLOOR',
            FakeForAllWithoutRunningActionSever(name='CLEAR_FLOOR'),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'FITNESS',
            SimpleActionState(
                'fitness',
                GeneralHobbitAction,
                goal_cb=patrol_cb,
                input_keys=['parameters', 'params'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'PATROL',
            SimpleActionState(
                'locate_user_simple',
                GeneralHobbitAction,
                goal_cb=patrol_cb,
                input_keys=['parameters', 'params'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'GOTO',
            SimpleActionState(
                'goto',
                GeneralHobbitAction,
                goal_cb=goto_cb,
                input_keys=['parameters', 'params'],
                output_keys=['goal_room'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'STOP',
            helper.get_hobbit_full_stop(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'CALL_HOBBIT',
            call_hobbit.call_hobbit(),
            transitions={'succeeded': 'CHECK_SAFETY_CHECK',
                         'preempted': 'preempted',
                         'aborted': 'CHECK_SAFETY_CHECK'}
        )
        StateMachine.add(
            'CHECK_SAFETY_CHECK',
            ServiceState('/user/get_safetycheck_state',
                         GetSafetycheckState,
                         request=GetSafetycheckStateRequest(state=True),
                         response_cb=sf_resp_cb),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'SAFETY_CHECK'}
        )
        sf_goal = GeneralHobbitGoal(command=String('safety_check'),
                             previous_state=String('call_hobbit'),
                             parameters=[])
        StateMachine.add(
            'SAFETY_CHECK',
            SimpleActionState(
                'safety_check',
                GeneralHobbitAction,
                goal=sf_goal,
                input_keys=['parameters', 'params'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'EMERGENCY_BATHROOM',
            SimpleActionState(
                'emergency_bathroom',
                GeneralHobbitAction,
                goal_cb=emergency_bathroom_cb,
                input_keys=['parameters', 'params'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'CALL',
            phone_call.wait_for_end_of_call(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'SURPRISE',
            social_role.get_surprise(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'FOLLOW',
            SimpleActionState(
                'follow_me_simple',
                GeneralHobbitAction,
                goal_cb=follow_cb,
                input_keys=['parameters', 'params'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'MAIN_MENU',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'succeeded': 'EMOTION',
                         'preempted': 'preempted',
                         'failed': 'EMOTION'}
        )
        StateMachine.add(
            'BRING_OBJECT',
            SimpleActionState(
                'bring_object',
                GeneralHobbitAction,
                goal_cb=bring_cb,
                input_keys=['parameters', 'params'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'RECHARGE',
            recharge.getRecharge(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SILENT_RECHARGE',
            recharge.getSilentRecharge(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'AWAY',
            SimpleActionState(
                'away',
                GeneralHobbitAction,
                goal_cb=away_cb,
                input_keys=['parameters', 'params'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'SLEEP',
            SimpleActionState(
                'sleep',
                GeneralHobbitAction,
                goal_cb=sleep_cb,
                input_keys=['parameters', 'params'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'SOCIAL_ROLE',
            social_role.get_social_role_change(),
            transitions={'succeeded': 'RESET_ACTIVE_TASK',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'RESET_ACTIVE_TASK_SILENT',
            ResetActiveTask(),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'RESET_ACTIVE_TASK',
            ResetActiveTask(),
            transitions={'succeeded': 'END_USER_INTERACTION',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'EMOTION',
            HobbitEmotions.ShowEmotions(
                emotion='NEUTRAL',
                emo_time=0),
            transitions={'succeeded': 'MUC_RESET_ACTIVE_TASK',
                         'preempted': 'MUC_RESET_ACTIVE_TASK',
                         'failed': 'MUC_RESET_ACTIVE_TASK'}
        )
        if MUC_ENABLED:
            StateMachine.add(
            'LEARN_OBJECT',
            SimpleActionState(
                'learn_object',
                GeneralHobbitAction,
                goal_cb=task_lo_cb,
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'END_USER_INTERACTION_LEARN',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
            StateMachine.add(
            'PICKUP',
            SimpleActionState(
                'pickup',
                GeneralHobbitAction,
                goal_cb=pickup_cb,
                input_keys=['parameters', 'params'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'END_USER_INTERACTION_PICKUP',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
            StateMachine.add(
                'END_USER_INTERACTION_REWARD',
                end_interaction.end_interaction_muc_reward(),
                transitions={'succeeded': 'succeeded',
                             'aborted': 'MUC_RESET_ACTIVE_TASK',
                             'preempted': 'preempted'}
            )
            StateMachine.add(
                'REWARD',
                social_role.get_reward_muc(),
                transitions={'succeeded': 'END_USER_INTERACTION_REWARD',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
            )
            StateMachine.add(
                'END_USER_INTERACTION_LEARN',
                end_interaction.end_interaction_muc_learn(),
                transitions={'succeeded': 'succeeded',
                             'aborted': 'MUC_RESET_ACTIVE_TASK',
                             'preempted': 'preempted'}
            )
            StateMachine.add(
                'END_USER_INTERACTION_PICKUP',
                end_interaction.end_interaction_muc_pickup(),
                transitions={'succeeded': 'succeeded',
                             'aborted': 'MUC_RESET_ACTIVE_TASK',
                             'preempted': 'preempted'}
            )
        else:
            StateMachine.add(
            'LEARN_OBJECT',
            SimpleActionState(
                'learn_object',
                GeneralHobbitAction,
                goal_cb=task_lo_cb,
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'END_USER_INTERACTION',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK'}
        )
            StateMachine.add(
                'REWARD',
                social_role.get_reward(),
                transitions={'succeeded': 'END_USER_INTERACTION',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
            )
            StateMachine.add(
            'PICKUP',
            SimpleActionState(
                'pickup',
                GeneralHobbitAction,
                goal_cb=pickup_cb,
                input_keys=['parameters', 'params'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'END_USER_INTERACTION',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )
        StateMachine.add(
            'MUC_RESET_ACTIVE_TASK',
            ResetActiveTask(),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'preempted',
                         'aborted': 'MUC_RESET_ACTIVE_TASK'}
        )
        StateMachine.add(
            'PREEMPT_RESET_ACTIVE_TASK',
            ResetActiveTask(),
            transitions={'succeeded': 'preempted',
                         'preempted': 'preempted',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT'}
        )

        StateMachine.add(
            'END_USER_INTERACTION',
            end_interaction.move_away(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'RESET_ACTIVE_TASK_SILENT',
                         'preempted': 'preempted'}
        )
    """
    Now we actually start the IntrospectionServer to visualize the StateMachine
    as a dot graph, and execute the main StateMachine.
    """
    #sis = IntrospectionServer('master', sm1, '/MASTER')
    #sis.start()
    sm1.execute()
    rospy.spin()
    #sis.stop()

if __name__ == '__main__':
    global wakeup_time
    global sleep_time
    global MUC_ENABLED
    wakeup_time = "09:00"
    sleep_time = "22:00"

    rospy.set_param('active_task', 100)
    if rospy.has_param('/Hobbit/sleep_time'):
        sleep_tmp = rospy.get_param('/Hobbit/sleep_time')
        if type(sleep_tmp) is not int:
            sleep_time = sleep_tmp
    if rospy.has_param('/Hobbit/wakeup_time'):
        wakeup_tmp = rospy.get_param('/Hobbit/wakeup_time')
        if type(wakeup_tmp) is not int:
            wakeup_time = wakeup_tmp

    if rospy.has_param('/Hobbit/enable_muc'):
        MUC_ENABLED = rospy.get_param('/Hobbit/enable_muc')
    else:
        MUC_ENABLED = False
    main()

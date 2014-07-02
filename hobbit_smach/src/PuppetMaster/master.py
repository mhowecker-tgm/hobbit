#!/usr/bin/env python

PKG = 'hobbit_smach'
NAME = 'HOBBIT_MASTER'
import roslib
roslib.load_manifest(PKG)
import rospy
import smach_ros
from smach import StateMachine, Concurrence, State
from hobbit_msgs.msg import Command, Event
import uashh_smach.util as util
from std_msgs.msg import String

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
    return False



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
        return 'succeeded'


def main():
    rospy.init_node(NAME)
    sm = StateMachine(
        outcomes=['succeeded',
                  'preempted',
                  'failed'],
        input_keys=['command'],
        output_keys=['command'],
    )

    sm1 = StateMachine(
        outcomes=['succeeded',
                  'preempted',
                  'failed'],
        input_keys=['command'],
        output_keys=['command'],
    )
    sm1.userdata.command = 'IDLE'

    cc = Concurrence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        default_outcome='aborted',
        input_keys=['command'],
        output_keys=['command'],
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
            TestASW()
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

    sis = smach_ros.IntrospectionServer('master', sm1, '/MASTER')
    sis.start()

    outcome = sm1.execute()
    rospy.loginfo(NAME + ' returned outcome ' + str(outcome))
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.set_param('active_task', 100)
    main()

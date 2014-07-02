#!/usr/bin/env python

PKG = 'hobbit_smach'
NAME = 'HOBBIT_MASTER'
import roslib
roslib.load_manifest(PKG)
import rospy
import smach_ros
from smach import StateMachine, Concurrence, Sequence
from hobbit_msgs.msg import Command, Status, Event
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

commands = [['G_FALL', 'E_SOSBUTTON', 'C_HELP', 'E_HELP'],
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


def event_cb(msg, ud):
    rospy.loginfo('/Event data received:')
    rospy.loginfo(str(msg))
    if rospy.has_param('active_task'):
        active_task = rospy.get_param('active_task')
    else:
        active_task = 100
    for index, item in enumerate(commands):
        if msg.event in item:
            if index + 1 >= active_task:
                rospy.loginfo('New task has lower priority. Do nothing')
                return False
            else:
                rospy.loginfo('New task has higher priority. Start it.')
                rospy.set_param('active_task', index)
                return True
        else:
            rospy.loginfo('Unknown event received %s' % msg.event)
    return False


def command_cb(msg, ud):
    rospy.loginfo('/Command data received:')
    rospy.loginfo(str(msg))
    if rospy.has_param('active_task'):
        active_task = rospy.get_param('active_task')
    else:
        active_task = 100
    for index, item in enumerate(commands):
        if msg.command in item:
            if index + 1 >= active_task:
                rospy.loginfo('New task has lower priority. Do nothing')
                return False
            else:
                rospy.loginfo('New task has higher priority. Start it.')
                rospy.set_param('active_task', index)
                return True
        else:
            rospy.loginfo('Unknown command received %s' % msg.command)
    return False


def main():
    rospy.init_node(NAME)
    sm = StateMachine(
        outcomes=['succeeded',
                  'preempted',
                  'failed']
    )

    cc = Concurrence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        default_outcome='aborted',
        outcome_map={'succeeded': {'Event_Listener': 'succeeded'},
                     'aborted': {'Event_Listener': 'aborted'}}
    )

    with cc:
        Concurrence.add(
            'Event_Listener',
            util.WaitForMsgState(
                '/Event',
                Event,
                msg_cb=event_cb
            )
        )
        Concurrence.add(
            'Command_Listener',
            util.WaitForMsgState(
                '/Command',
                Command,
                msg_cb=command_cb
            )
        )
        # Concurrence.add(
        #     'Status_Listener',
        #     util.WaitForMsgState(
        #         '/Status',
        #         Status,
        #         msg_cb=status_cb
        #     )
        # )

    with sm:
        # StateMachine.add(
        #     'START',
        #     Start(),
        #     transitions={'succeeded': 'WAIT_FOR_E_C',
        #                  'aborted': 'SET_FAILURE'}
        # )
        StateMachine.add(
            'WAIT_FOR_E_C',
            cc,
            transitions={'succeeded': 'WAIT_FOR_E_C',
                         'aborted': 'WAIT_FOR_E_C',
                         'preempted': 'preempted'}
        )

    sis = smach_ros.IntrospectionServer('master', sm, '/MASTER')
    sis.start()

    outcome = sm.execute()
    rospy.loginfo(NAME + ' returned outcome' + str(outcome))
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.set_param('active_task', 100)
    main()

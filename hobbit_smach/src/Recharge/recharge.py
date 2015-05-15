#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'recharge'

DEBUG = False

import rospy

from hobbit_msgs.msg import GeneralHobbitAction
from std_msgs.msg import String
from smach_ros import ActionServerWrapper, \
    IntrospectionServer
from smach import StateMachine
import hobbit_smach.recharge_import as recharge


def main():
    rospy.init_node(NAME)

    rech_sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command', 'previous_state', 'parameters'],
        output_keys=['result'])

    rech_sm.userdata.result = String('started')

    with rech_sm:
        StateMachine.add(
            'RECHARGE',
            recharge.getRecharge(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )

    asw = ActionServerWrapper(
        'recharge',
        GeneralHobbitAction,
        rech_sm,
        ['succeeded'], ['failure'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command',
                        'previous_state': 'previous_state',
                        'parameters': 'parameters'}
    )

    #sis = IntrospectionServer('smach_server', rech_sm, '/HOBBIT/rech_sm_ROOT')
    #sis.start()
    asw.run_server()
    rospy.spin()
    #sis.stop()

if __name__ == '__main__':
    main()

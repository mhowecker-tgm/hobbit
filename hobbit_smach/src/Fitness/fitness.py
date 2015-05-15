#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'fitness'

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer, ActionServerWrapper

from hobbit_msgs.msg import GeneralHobbitAction
import hobbit_smach.fitness_import as fitness


def main():
    rospy.init_node(NAME)

    fit_sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command', 'parameters'],
        output_keys=['result'])

    with fit_sm:
        StateMachine.add(
            'START_FITNESS',
            fitness.get_do_fitness(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )

    asw = ActionServerWrapper(
        'fitness', GeneralHobbitAction, fit_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command',
                        'parameters': 'parameters'})

    #sis = IntrospectionServer(
    #    'smach_server',
    #    fit_sm,
    #    '/HOBBIT/FIT_SM_ROOT')

    #sis.start()
    asw.run_server()
    rospy.spin()
    #sis.stop()

if __name__ == '__main__':
    main()

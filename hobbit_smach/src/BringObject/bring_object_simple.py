#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'bring_object'

import roslib
roslib.load_manifest(PKG)
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer, ActionServerWrapper

from hobbit_msgs.msg import GeneralHobbitAction
from hobbit_user_interaction import HobbitMMUI
import hobbit_smach.bring_object_simple_import as bring_object
import hobbit_smach.logging_import as log

def main():
    rospy.init_node(NAME)

    bo_sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command', 'parameters'],
        output_keys=['result'])

    with bo_sm:
        StateMachine.add(
            'BRING_OBJECT',
            bring_object.get_bring_object(),
            transitions={'succeeded': 'SHOW_MAIN_SUCCESS',
                         'aborted': 'SHOW_MAIN_ABORT',
                         'preempted': 'SHOW_MAIN_PREEMPT'}
        )
        StateMachine.add_auto(
            'SHOW_MAIN_SUCCESS',
            HobbitMMUI.ShowMenu(
                menu='MAIN'
            ),
            connector_outcomes=['succeeded', 'failed', 'preempted']
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Bring object'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add_auto(
            'SHOW_MAIN_PREEMPT',
            HobbitMMUI.ShowMenu(
                menu='MAIN'
            ),
            connector_outcomes=['succeeded', 'failed', 'preempted']
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Bring object'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add_auto(
            'SHOW_MAIN_ABORT',
            HobbitMMUI.ShowMenu(
                menu='MAIN'
            ),
            connector_outcomes=['succeeded', 'failed', 'preempted']
        )
        StateMachine.add(
            'LOG_ABORTED',
            log.DoLogAborted(scenario='Bring object'),
            transitions={'succeeded': 'aborted'}
        )

    asw = ActionServerWrapper(
        'bring_object', GeneralHobbitAction, bo_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command',
                        'parameters': 'parameters'})

    sis = IntrospectionServer(
        'smach_server',
        bo_sm,
        '/HOBBIT/BO_SM_ROOT')

    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

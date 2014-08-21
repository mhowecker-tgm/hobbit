#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'safety_check'
DEBUG = True

import rospy
import roslib
roslib.load_manifest(PKG)

from hobbit_user_interaction import HobbitMMUI
from smach_ros import IntrospectionServer, ActionServerWrapper
from hobbit_msgs.msg import GeneralHobbitAction
from smach import StateMachine
import hobbit_smach.safety_check_import as safety_check


def main():
    rospy.init_node(NAME)

    sc_sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command'],
        output_keys=['result'])

    with sc_sm:
        StateMachine.add_auto(
            'SAFETY_CHECK',
            safety_check.get_safety_check(),
            connector_outcomes=['succeeded', 'aborted', 'preempted']
        )
        StateMachine.add(
            'MAIN_MENU',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'succeeded': 'succeeded',
                         'failed': 'aborted',
                         'preempted': 'preempted'}
        )

    asw = ActionServerWrapper(
        'safety_check', GeneralHobbitAction, sc_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        result_slots_map={'result': 'result'}
    )

    sis = IntrospectionServer(
        'smach_server',
        sc_sm,
        '/HOBBIT/safety_check_SM_ROOT'
    )

    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

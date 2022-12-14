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
import hobbit_smach.bring_object_import as bring_object


def main():
    rospy.init_node(NAME)

    bo_sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command'],
        output_keys=['result'])

    with bo_sm:
        StateMachine.add(
            'BRING_OBJECT',
            bring_object.get_bring_scenario(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )

    asw = ActionServerWrapper(
        'bring_object', GeneralHobbitAction, bo_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command'})

    sis = IntrospectionServer(
        'smach_server',
        bo_sm,
        '/HOBBIT/bo_sm_ROOT')

    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

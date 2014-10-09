#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'locate_user'

import roslib
roslib.load_manifest(PKG)
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer, ActionServerWrapper
from hobbit_msgs.msg import GeneralHobbitAction
import hobbit_smach.locate_user_simple_import as locate_user
import hobbit_smach.logging_import as log


def main():
    rospy.init_node(NAME)

    lu_sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command'],
        output_keys=['result'])

    with lu_sm:
        StateMachine.add(
            'LOCATE_USER',
            locate_user.get_detect_user(),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'aborted': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Locate user'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Locate user'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORTED',
            log.DoLogAborted(scenario='Locate user'),
            transitions={'succeeded': 'aborted'}
        )

    asw = ActionServerWrapper(
        'locate_user_simple', GeneralHobbitAction, lu_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command'})

    sis = IntrospectionServer(
        'smach_server',
        lu_sm,
        '/HOBBIT/LU_SM_ROOT')

    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

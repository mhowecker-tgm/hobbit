#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'safety_check'
DEBUG = True

import rospy
import roslib
roslib.load_manifest(PKG)

from std_msgs.msg import String
from hobbit_user_interaction import HobbitMMUI
from smach_ros import IntrospectionServer, ActionServerWrapper
from hobbit_msgs.msg import GeneralHobbitAction
from smach import StateMachine, State
import hobbit_smach.safety_check_import as safety_check
import hobbit_smach.logging_import as log


def main():
    rospy.init_node(NAME)

    sc_sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command'],
        output_keys=['result'])
    sc_sm.userdata.result = 'failed'

    with sc_sm:
        StateMachine.add_auto(
            'SAFETY_CHECK',
            safety_check.get_safety_check(),
            connector_outcomes=['succeeded'],
            transitions={'aborted': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add_auto(
            'SET_SUCCESS',
            SetSuccess(),
            connector_outcomes=['succeeded', 'preempted']
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogPreempt(scenario='Safety Check'),
            transitions={'succeeded': 'succeeded'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Safety Check'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORT',
            log.DoLogPreempt(scenario='Safety Check'),
            transitions={'succeeded': 'failure'}
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


class SetSuccess(State):
    """
    Class for setting the success message in the actionlib result \
        and clean up of persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            output_keys=['result']
        )

    def execute(self, ud):
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('succeeded')
        return 'succeeded'


if __name__ == '__main__':
    main()

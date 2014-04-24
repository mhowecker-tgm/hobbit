#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'Emergency'

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros

from std_msgs.msg import String
#from hobbit_msgs.srv import *
from hobbit_msgs.msg import EndUserInteractionGoal, EndUserInteractionAction,\
    LocateUserAction, LocateUserGoal, GeneralHobbitAction, Event
#from actionlib import SimpleActionServer
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
import uashh_smach.util as util


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

    def disable(self):
        self.HEADER = ''
        self.OKBLUE = ''
        self.OKGREEN = ''
        self.WARNING = ''
        self.FAIL = ''
        self.ENDC = ''


class Init(smach.State):
    """Class to initialize certain parameters"""
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded_normal',
                      'succeeded_bathroom',
                      'aborted',
                      'preempted'],
            input_keys=['command'],
            output_keys=['social_role'])
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)
        self.pub_face.publish('EMO_NEUTRAL')

    def execute(self, ud):
        print('HELLO WORLD')
        self.pub_face.publish('EMO_NEUTRAL')
        #self.pub_head.publish('down')
        #self.pub_obstacle.publish('active')
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        return 'succeeded_normal'


class CleanUp(smach.State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['command', 'visited_places'],
            output_keys=['result', 'command', 'visited_places'])
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
        ud.visited_places = []
        ud.result = String('user not detected')
        return 'succeeded'


class SetSuccess(smach.State):
    """
    Class for setting the success message in the actionlib result and clean
    up of persistent variables
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            output_keys=['result', 'visited_places'])
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        ud.visited_places = []
        self.pub_face.publish('EMO_HAPPY')
        self.pub.publish('Stop')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('user detected')
        return 'succeeded'


class FaceHappy(smach.State):
    """
    Show a happy face
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                'succeeded',
                'preempted'
            ]
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'succeeded'


def callstate_cb(msg, ud):
    ud.call_state = msg.command
    print(msg.command)
    return True


def main():
    rospy.init_node(NAME)

    em_sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command'],
        output_keys=['result'])

    em_sm.userdata.result = String('started')
    em_sm.userdata.detection = False
    em_sm.userdata.question = 'Do you need help?'

    with em_sm:
        smach.StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded_normal': 'NEED_HELP_Y_N',
                         #'succeeded_bathroom': 'RUN_MOVE_BASE',
                         'succeeded_bathroom': 'NEED_HELP_Y_N',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'NEED_HELP_Y_N',
            HobbitMMUI.AskYesNo(question='T_HM_DoYouNeedHelp'),
            transitions={'yes': 'EMO_HAPPY',
                         'no': 'END_USER_INTERACTION',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'EMO_HAPPY',
            HobbitEmotions.ShowEmotions(emotion='HAPPY', emo_time=4),
            transitions={'succeeded': 'START_CALL',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'START_CALL',
            HobbitMMUI.CallEmergency(),
            transitions={'succeeded': 'CHECK_CALL_STATE',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'CHECK_CALL_STATE',
            util.WaitForMsgState(
                '/Event',
                Event,
                callstate_cb,
                timeout=2,
                output_keys=['state']),
            transitions={'succeeded': 'END_CALL',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'END_CALL',
            HobbitMMUI.CallEnded(),
            transitions={'succeeded': 'END_USER_INTERACTION',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'END_USER_INTERACTION',
            smach_ros.SimpleActionState(
                'end_user_interaction',
                EndUserInteractionAction,
                goal=EndUserInteractionGoal(
                    command=String('end_user_interaction'))
            ),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'preempted',
                         'aborted': 'aborted'}
        )

    asw = smach_ros.ActionServerWrapper(
        'emergency', GeneralHobbitAction, em_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command'}
    )

    sis = smach_ros.IntrospectionServer(
        'smach_server',
        em_sm,
        '/HOBBIT/EM_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

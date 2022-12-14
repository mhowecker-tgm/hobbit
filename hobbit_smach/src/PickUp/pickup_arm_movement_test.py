#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
PROJECT = 'Hobbit'
NAME = 'pickup_object_grasp_only'

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import uashh_smach.util as util
from uashh_smach.util import SleepState  # df 30.7.2014
# import tf
# import math
from smach import StateMachine
from smach_ros import ActionServerWrapper, IntrospectionServer, ServiceState,\
    MonitorState
from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction
from hobbit_msgs.srv import SwitchVision, SwitchVisionRequest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from rgbd_acquisition.msg import PointEvents
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.head_move_import as head_move
import hobbit_smach.arm_move_import as arm_move
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.pickup_import as pickup
import hobbit_smach.logging_import as log
# from hobbit_smach.helper_import import WaitForMsgState


def switch_vision_cb(ud, response):
    if response.result:
        return 'succeeded'
    else:
        return 'aborted'


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


#def pointevents_cb(msg, ud):
def pointevents_cb(ud, msg):
    print('pointevents_cb')
    print(msg)
    ud.pointing_msg = msg
    return False

#def point_cloud_cb(msg, ud):
def point_cloud_cb(ud, msg):
    print('point cloud received')
    ud.cloud = msg
    return False





class Init(smach.State):
    """Class to initialize certain parameters"""
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'canceled'],
            output_keys=['social_role'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        return 'succeeded'




def main():
    rospy.init_node(NAME)

    pickup_sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        output_keys=['result'])

    pickup_sm.userdata.result = String('started')
    pickup_sm.userdata.detection = False

    with pickup_sm:
        StateMachine.add(
            'HEAD_TO_SEARCH',
            head_move.MoveTo(pose='to_grasp'),  # wait=True <=> df
            transitions={'succeeded': 'WAIT_FINISH_HEAD_TO_SEARCH', # df
                         'aborted': 'LOG_ABORT',
                         'preempted': 'LOG_ABORT'}
        )
        StateMachine.add(
            'WAIT_FINISH_HEAD_TO_SEARCH',
            SleepState(duration=2),
            transitions={'succeeded': 'SAY_MOVE_ARM_HOME',
                         'preempted': 'LOG_ABORT'}
        )
        
        StateMachine.add(
            'SAY_MOVE_ARM_TO_PRE_GRASP_POS',
            speech_output.sayText(info='move arm to pregrasp from floor position'),
            transitions={'succeeded': 'MOVE_ARM_TO_PRE_GRASP_POS',
                         'failed': 'LOG_ABORT'}
        )
        
        
        StateMachine.add(
            'MOVE_ARM_TO_PRE_GRASP_POS',
            arm_move.goToPreGraspPosition(),
            transitions={'succeeded': 'SAY_MOVE_ARM_HOME', 
                         'preempted': 'LOG_ABORT',
                         'failed': 'LOG_ABORT'}    # better failure handling appreciated
        )
        
            
        StateMachine.add(
            'SAY_MOVE_ARM_HOME',
            speech_output.sayText(info='move arm to home position'),
            transitions={'succeeded': 'MOVE_ARM_TO_HOME',
                         'failed': 'LOG_ABORT'}
        )

        
        StateMachine.add(
            'MOVE_ARM_TO_HOME',   #position where gripper is not blocking view to floor where object was lying
            arm_move.goToHomePosition(),
            #arm_move.goToTrayPosition(),
            transitions={'succeeded': 'succeeded', 
                         'preempted': 'LOG_ABORT',
                         'failed': 'LOG_ABORT'}    # better failure handling appreciated
        )
        
        

        StateMachine.add(
            'LOG_ABORT',
            log.DoLogAborted(scenario='Pickup'),
            transitions={'succeeded': 'aborted'}
        )

    asw = ActionServerWrapper(
        'pickup', GeneralHobbitAction, pickup_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        result_slots_map={'result': 'result'}
    )

    sis = IntrospectionServer(
        'smach_server',
        pickup_sm,
        '/HOBBIT/PICKUP_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

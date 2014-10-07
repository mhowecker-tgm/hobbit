#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'learn_object'
DEBUG = False
dir_param = '/hobbit/pcd_path'

import rospy
import os
import glob
import shutil
import random

from smach import Sequence, Concurrence, State
from smach_ros import MonitorState
from sensor_msgs.msg import PointCloud2
import uashh_smach.util as util
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
import hobbit_smach.arm_move_import as move_arm
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.head_move_import as head_move
import hobbit_smach.arm_move_import as arm_move


class MoveFiles(State):

    """
    Move the pcd files for the upper or lower part of the object
    """

    def __init__(self, side):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed']
        )
        self.side = side

    def execute(self, ud):
        if rospy.has_param('/hobbit/pcd_path'):
            directory = rospy.get_param('/hobbit/pcd_path') + '/'
        else:
            directory = '/tmp/pcd_data/'
        directory2 = directory + self.side
        if not os.path.exists(directory2):
            os.makedirs(directory2)
        for data in glob.glob(directory + '*.pcd'):
            shutil.move(data, directory2)
        return 'succeeded'


class SetName(State):

    """
    Set the name of the pcd files
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failure'],
            input_keys=['object_name']
        )
        random.seed()

    def execute(self, ud):
        rospy.set_param('/hobbit/object_to_learn', ud.object_name)
        rospy.loginfo(ud.object_name)
        if rospy.has_param('/hobbit/pcd_path'):
            directory = rospy.get_param('/hobbit/pcd_path') + '/'
        else:
            directory = '/localhome/demo/objects/pcd_data/'
        if not ud.object_name == '':
            directory2 = directory + ud.object_name
        else:
            directory2 = directory + 'object-' +\
                random.choice("abcdefghijklmnopqrstuvwxyz")
        if not os.path.exists(directory2):
            os.makedirs(directory2)
        shutil.move(directory + 'up', directory2)
        shutil.move(directory + 'down', directory2)
        return 'succeeded'


def returnTurntable():
    """
    Return a SMACH Sequence to return the turntable from
    its current position to the home position

    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'SAY_REMOVE_OBJECT',
            HobbitMMUI.ConfirmInfo(
                info='T_LO_PLEASE_REMOVE_OBJECT_FROM_TURNTABLE'))
        Sequence.add(
            'SAY_RETURN_TURNTABLE',
            speech_output.sayText(info='T_LO_RETURNING_TURNTABLE'))
        Sequence.add(
            'MOVE_ARM_TT_POSE',
            move_arm.returnTurnTable())
        Sequence.add(
            'MOVE_HEAD_FRONT',
            head_move.MoveTo(pose='center_center'),
            transitions={'aborted': 'failed'})
        Sequence.add('MMUI_MAIN_MENU', HobbitMMUI.ShowMenu(menu='MAIN'))
    return seq


def returnTurntableProblem():
    """
    Return a SMACH Sequence to return the turntable from
    its current position to the home position

    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'RETURN_TURNTABLE',
            returnTurntable())
        Sequence.add(
            'MMUI_SAY_STOPPED_LEARNING',
            speech_output.sayText(info='T_LO_I_STOPPED_LEARNING'))
        Sequence.add('MMUI_MAIN_MENU', HobbitMMUI.ShowMenu(menu='MAIN'))
    return seq


def unloadReturnTurntable():
    """
    Returns a SMACH Sequence that will first unload the turntable,
    returns it to the storage position and returns the arm to its
    home position.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'EMO_SAD',
            HobbitEmotions.ShowEmotions(emotion='SAD', emo_time=4)
        )
        Sequence.add(
            'SAY_I_WILL_PUT_THE_OBJECT_INTO_TRAY',
            speech_output.sayText(info='T_LO_I_WILL_PUT_OBJECT_INTO_TRAY')
        )
        # Sequence.add(
        #    'MOVE_ARM_TRAY_POSE',
        #    move_arm.goToTrayPosition()
        # )
        # Sequence.add(
        #    'RETURN_TURNTABLE',
        #    returnTurntable()
        # )
    return seq


def child_term_cb(outcome_map):
    if outcome_map['ROTATE'] == 'succeeded':
        return True


def msg_cb(msg, ud):
    pub = rospy.Publisher('/hobbit/object/points', PointCloud2)
    pub.publish(msg)
    return False


def out_cb(outcome_map):
    if outcome_map['ROTATE'] == 'succeeded':
        return 'succeeded'


def getDataCW():
    """
    Return a SMACH concurrence container which rotates the turntable and
    publishes the received point cloud data to a topic for saving as a
    series of pcd files.

    in_topic: /headcam/depth_registered/points
    out_topic: /hobbit/object/points
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    seq1 = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'GET_DATA',
            MonitorState(
                '/headcam/depth_registered/points',
                PointCloud2,
                cond_cb=msg_cb
            ),
            # util.WaitForMsgState('/headcam/depth_registered/points',
            #                      # util.WaitForMsgState('/camera/depth_registered/points',
            #                      PointCloud2,
            #                      msg_cb=msg_cb
            #                      ),
            # transitions={'aborted': 'GET_DATA'})
            transitions={'invalid': 'GET_DATA',
                         'valid': 'succeeded'})

    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_cb=out_cb
    )

    with cc:
        Concurrence.add(
            'ROTATE',
            arm_move.rotateToCW())
        Concurrence.add(
            'GET_DATA',
            seq)

    with seq1:
        Sequence.add(
            'DATA',
            cc
        )
        Sequence.add(
            'MOVE_FILES',
            MoveFiles('up')
        )
    return seq1


def getDataCCW():
    """
    Return a SMACH concurrence container which rotates the turntable and
    publishes the received point cloud data to a topic for saving as a
    series of pcd files.

    in_topic: /headcam/depth_registered/points
    out_topic: /hobbit/object/points
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add(
            'GET_DATA',
            MonitorState(
                '/headcam/depth_registered/points',
                PointCloud2,
                cond_cb=msg_cb
            ),
            transitions={'invalid': 'GET_DATA',
                         'valid': 'succeeded'})
            # util.WaitForMsgState('/headcam/depth_registered/points',
            #                      # util.WaitForMsgState('/camera/depth_registered/points',
            #                      PointCloud2,
            #                      msg_cb=msg_cb
            #                      ),
            # transitions={'aborted': 'GET_DATA'})

    seq1 = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    cc = Concurrence(
        outcomes=['succeeded', 'preempted', 'failed'],
        default_outcome='failed',
        child_termination_cb=child_term_cb,
        outcome_cb=out_cb
    )

    with cc:
        Concurrence.add(
            'ROTATE',
            arm_move.rotateToCCW())
        Concurrence.add(
            'GET_DATA',
            seq)

    with seq1:
        Sequence.add(
            'DATA',
            cc
        )
        Sequence.add(
            'MOVE_FILES',
            MoveFiles('down')
        )
    return seq1

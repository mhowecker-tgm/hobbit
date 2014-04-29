#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'learn_object'
DEBUG = True

import roslib
roslib.load_manifest(PKG)
#import rospy

from smach import Sequence
#from std_msgs.msg import String
from hobbit_msgs.msg import Event
#import uashh_smach.util as util
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.arm_move_import as move_arm


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
            'MMUI_SAY_RETURN_TURNTABLE',
            HobbitMMUI.ShowInfo(info='RETURNING_TURNTABLE'))
        Sequence.add(
            'MOVE_ARM_TT_POSE',
            move_arm.goToPosition(pose='storage'))
        Sequence.add(
            'OPEN_GRIPPER',
            move_arm.OpenGripper())
        if not DEBUG:
            Sequence.add(
                'MOVE_BACK',
                hobbit_move.Move(goal='back', distance=0.25))
        Sequence.add(
            'MOVE_ARM_STORAGE',
            move_arm.goToPosition(pose='home'))
        if not DEBUG:
            Sequence.add(
                'MOVE_FRONT',
                hobbit_move.Move(goal='front', distance=0.25))
        Sequence.add(
            'MMUI_SAY_STOPPED_LEARNING',
            HobbitMMUI.ShowInfo(info='I_STOPPED_LEARNING'))
        Sequence.add(
            'WAIT_FOR_MMUI',
            HobbitMMUI.WaitforSoundEnd('/Event', Event),
            transitions={'aborted': 'WAIT_FOR_MMUI'})
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
            HobbitMMUI.ShowInfo(info='SAY_I_WILL_PUT_THE_OBJECT_INTO_TRAY')
        )
        Sequence.add(
            'MOVE_ARM_TRAY_POSE',
            move_arm.goToPosition(pose='tray')
        )
        Sequence.add(
            'TILT_TRAY',
            move_arm.goToPosition(pose='tray_tilt')
        )
        Sequence.add(
            'RETURN_TURNTABLE',
            returnTurntable()
        )
    return seq

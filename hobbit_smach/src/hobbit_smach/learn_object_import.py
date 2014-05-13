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
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.head_move_import as head_move


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
            'SAY_RETURN_TURNTABLE',
            speech_output.sayText(info='RETURNING_TURNTABLE'))
        Sequence.add(
            'MOVE_ARM_TT_POSE',
        #    move_arm.goToPosition(pose='storage'))
        #    move_arm.StoreTurntable())
             move_arm.returnTurnTable())
        #Sequence.add(
        #    'OPEN_GRIPPER',
        #    move_arm.OpenGripper())
        if not DEBUG:
            Sequence.add(
                'MOVE_BACK',
                hobbit_move.Move(goal='back', distance=0.25))
        #Sequence.add(
        #    'MOVE_ARM_STORAGE',
        #    move_arm.goToHomePosition())
        if not DEBUG:
            Sequence.add(
                'MOVE_FRONT',
                hobbit_move.Move(goal='front', distance=0.25))
        Sequence.add(
            'MOVE_HEAD_FRONT',
            head_move.MoveTo(pose='center_center'))
        Sequence.add(
            'MMUI_SAY_STOPPED_LEARNING',
            speech_output.sayText(info='I_STOPPED_LEARNING'))
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
            speech_output.sayText(info='SAY_I_WILL_PUT_THE_OBJECT_INTO_TRAY')
        )
        #Sequence.add(
        #    'MOVE_ARM_TRAY_POSE',
        #    move_arm.goToTrayPosition()
        #)
        #Sequence.add(
        #    'RETURN_TURNTABLE',
        #    returnTurntable()
        #)
    return seq

#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'ArmMove'
DEBUG = False

import roslib
roslib.load_manifest(PKG)
import rospy

from smach import Sequence, State
from uashh_smach.util import SleepState
from ArmControllerClientFunctions import ArmClientFunctions
from ArmClientFunctionsPublisher import ArmClientFunctionsPublisher
from ArmActionClient import ArmActionClient

import actionlib
import hobbit_msgs.msg




#class ArmClientSingleton(object):
#    """To avoid running multiple arm clients, this singleton class
#    provides one ArmClientFunctions that is initialised and retrieved
#    via class methods init() and get()
#    """
#    _arm = None
#
#    @classmethod
#    def init(cls):
#        """Ignore multiple calls."""
#        if cls._arm is None:
#            print('Open connection to arm')
#            cls._arm = ArmClientFunctions('192.168.2.190')
#        else:
#            print('connection already established')
#
#    @classmethod
#    def get(cls):
#        """Does initialise if needed, too."""
#        cls.init()
#        return cls._arm

#if not DEBUG:
#    pass
    #new:
arm_client = ArmActionClient()
    #cmd = String ("GetArmState")
    #res = arm_client.arm_action_client(cmd)
    #changes done for new ArmActionServer based communication: arm.command() => arm_client.arm_action_client(String(command))
    #old:
    #arm = ArmClientFunctions('192.168.2.190')
#else:
#    arm = True

def getArmAtPosition(position='home'):
    #status = ast.literal_eval(arm.GetArmState())
    if position == 'home':
        if arm_client.GetArmAtHomePos():
            return True
    elif position == 'cwpos':
        if arm_client.GetTurntableAtCWPos():
            return True
    elif position == 'ccwpos':
        if arm_client.GetTurntableAtCCWPos():
            return True
    elif position == 'learn':
        if arm_client.GetArmAtLearningPos():
            return True
    elif position == 'pregrasp':
        if arm_client.GetArmAtPreGraspFromFloorPos():
            return True
    elif position == 'store':
        if arm_client.GetArmAtHomePos():
            return True
    elif position == 'tray':
        if arm_client.GetArmAtTrayPos():
            return True
    else:
        # Unless the data is correct we just wait 3 seconds and assume the arm reached the goal
	    print "getArmAtPosition result was False at time of execution for position: [=> wait 3 sec and assume arm is in correct position!]",position
    return False


class SetMoveToLearning(State):
    """
    Move the arm to the turntable, grasp it and move to the Learning position.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if not arm_client.GetArmIsEnabled():
            return 'failed'
        if arm_client.SetMoveToLearningPos():
            return 'succeeded'
        else:
            return 'failed'


class StoreTurntable(State):
    """
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if not arm_client.GetArmIsEnabled():
            return 'failed'
        if arm_client.SetStoreTurntable():
            return 'succeeded'
        else:
            return 'failed'


class CheckArmAtHomePos(State):
    """
    Check that the arm is at the home position. This has to be true
    before hobbit will start navigation.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        # TODO: check if this sleep is needed
        rospy.sleep(1)
        result = arm_client.GetArmIsEnabled()
        rospy.loginfo("arm_client.GetArmIsEnabled: "+str(result))
        if not result:
            rospy.loginfo("Arm is not enabled. Can not be sure that it is at the home position")
            return 'aborted'
        result = getArmAtPosition('home')
        rospy.loginfo("ArmAtHomePosition: "+str(result))
        if result:
            return 'succeeded'
        else:
            return 'aborted'


class CheckArmReachedKnownPosition(State):
    """
    Check that the Arm has reached the specified predefined position.
    """
    def __init__(self, position):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )
        self.position = position

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.sleep(2)
        if not arm_client.GetArmIsEnabled():
            return 'failed'
        if getArmAtPosition(self.position):
            return 'succeeded'
        else:
            return 'failed'


class CheckArmIsNotMoving(State):
    """
    Check that the Arm has stopped the movement.
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if not arm_client.GetArmIsEnabled():
            return 'failed'
        if arm_client.GetArmIsMoving():
            return 'failed'
        else:
            return 'succeeded'


class SetArmPosition(State):
    """
    Move the robotarm to a specified position
    """
    def __init__(self, position):
        State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted']
        )
        self.position = position

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if self.position == 'home':
            status = arm_client.SetMoveToHomePos()
            return 'succeeded'
        elif self.position == 'learn':
            status = arm_client.SetMoveToLearningPos()
            return 'succeeded'
        elif self.position == 'tray':
            status = arm_client.SetMoveToTrayPos()
            return 'succeeded'
        elif self.position == 'pregrasp':
            status = arm_client.SetMoveToPreGraspFromFloorPos()
            return 'succeeded'
        elif self.position == 'ccwpos':
            status = arm_client.SetTurnTurntableCCW()
            return 'succeeded'
        elif self.position == 'cwpos':
            status = arm_client.SetTurnTurntableCW()
            return 'succeeded'
        elif self.position == 'store':
            status = arm_client.SetStoreTurntable()
            return 'succeeded'
        elif self.position == 'graspfromfloorcheck': #df new 5.2.2015
            status = arm_client.SetMoveToCheckGraspFromFloorPosition()
            return 'succeeded'
        elif self.position == 'pregraspfromfloormanually': #df new 5.2.2015
            status = arm_client.SetMoveToPreGraspFromFloorPosManually()
            return 'succeeded'
        else:
            return 'failed'
	#df: next lines would have been unreachable anyway
        #if status[1] == 'COMMAND_OK':
        #    return 'succeeded'
        #else:
        #    return 'failed'


def goToPreGraspPosition():
    """
    Return a SMACH Sequence that will move the arm to pregrasp position.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('MOVE_ARM_TO_PREGRASP_POSE',
                     SetArmPosition(position='pregrasp'))
        Sequence.add('ARM_POSE_REACHED',
                     CheckArmReachedKnownPosition(position='pregrasp'),
                     transitions={'failed': 'ARM_POSE_REACHED'})
        Sequence.add('CHECK_ARM_IS_NOT_MOVING', CheckArmIsNotMoving(),
                     transitions={'failed': 'CHECK_ARM_IS_NOT_MOVING'})
    return seq

#df new 5.2.2015
def goToPreGraspPositionManually(): #move via joint values, directly from whatever position the arm is currently in
    """
    Return a SMACH Sequence that will move the arm to pregrasp position with fixed joint values (directly from current arm position).
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('MOVE_ARM_TO_PREGRASP_POSE_MANUALLY',
                     SetArmPosition(position='pregraspfromfloormanually'))
        Sequence.add('CHECK_ARM_IS_NOT_MOVING', CheckArmIsNotMoving(),
                     transitions={'failed': 'CHECK_ARM_IS_NOT_MOVING'})
    return seq


#df new 5.2.2015
def goToCheckGraspPosition():
    """
    Return a SMACH Sequence that will move the arm to check grasp position. (arm not blocking view of robot)
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('MOVE_ARM_TO_CHECK_GRASP_POSE',
                     SetArmPosition(position='graspfromfloorcheck'))
        Sequence.add('CHECK_ARM_IS_NOT_MOVING', CheckArmIsNotMoving(),
                     transitions={'failed': 'CHECK_ARM_IS_NOT_MOVING'})
    return seq






def goToTrayPosition():
    """
    Return a SMACH Sequence that will move the arm to the tray pose.
    """
    #moves automatically to home position!!
    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('MOVE_ARM_TO_TRAY_POSE',
                     SetArmPosition(position='tray'))
        Sequence.add('ARM_POSE_REACHED',
                     CheckArmReachedKnownPosition(position='home'),
                     transitions={'failed': 'ARM_POSE_REACHED'})
        Sequence.add('CHECK_ARM_IS_NOT_MOVING', CheckArmIsNotMoving(),
                     transitions={'failed': 'CHECK_ARM_IS_NOT_MOVING'})
    return seq


def moveToCW():
    """
    Return a SMACH Sequence that will return the turntable after the learning.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('MOVE_ARM_TO_CW',
                     SetArmPosition(position='cwpos'))
        Sequence.add('ARM_POSE_REACHED',
                     CheckArmReachedKnownPosition(position='cwpos'),
                     transitions={'failed': 'ARM_POSE_REACHED'})
        Sequence.add('CHECK_ARM_IS_NOT_MOVING', CheckArmIsNotMoving(),
                     transitions={'failed': 'CHECK_ARM_IS_NOT_MOVING'})


def returnTurnTable():
    """
    Return a SMACH Sequence that will return the turntable after the learning.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('MOVE_ARM_STORE',
                     SetArmPosition(position='store'))
        Sequence.add('ARM_IN_HOME_POSE',
                     CheckArmReachedKnownPosition(position='store'),
                     transitions={'failed': 'ARM_IN_HOME_POSE'})
        Sequence.add('CHECK_TT_IS_NOT_MOVING', CheckArmIsNotMoving(),
                     transitions={'failed': 'CHECK_TT_IS_NOT_MOVING'})
    return seq


def goToLearnPosition():
    """
    Return a SMACH Sequence that will move the arm to the learning pose.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('MOVE_ARM_TO_LEARNING_POSE',
                     SetArmPosition(position='learn'))
        Sequence.add('ARM_POSE_REACHED',
                     CheckArmReachedKnownPosition(position='learn'),
                     transitions={'failed': 'ARM_POSE_REACHED'})
        Sequence.add('CHECK_ARM_IS_NOT_MOVING', CheckArmIsNotMoving(),
                     transitions={'failed': 'CHECK_ARM_IS_NOT_MOVING'})
        #Sequence.add('ROTATE_TT_CCW',
        #             SetArmPosition(position='ccwpos'))
        #Sequence.add('TT_ROTATED',
        #             CheckArmReachedKnownPosition(position='ccwposw'),
        #             transitions={'failed': 'TT_ROTATED'})
        #Sequence.add('CHECK_TT_IS_NOT_MOVING', CheckArmIsNotMoving(),
        #             transitions={'failed': 'CHECK_TT_IS_NOT_MOVING'})
    return seq


def goToHomePosition():
    """
    Return a SMACH Sequence that will move the arm to the home pose.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('MOVE_ARM_TO_HOME',
                     SetArmPosition(position='home'))
        Sequence.add('CHECK_ARM_IS_NOT_MOVING', CheckArmIsNotMoving(),
                     transitions={'failed': 'CHECK_ARM_IS_NOT_MOVING'})
        Sequence.add('ARM_POSE_REACHED',
                     CheckArmReachedKnownPosition(position='home'),
                     transitions={'failed': 'MOVE_ARM_TO_HOME'})

    return seq


def rotateToCW():
    """
    Return a SMACH Sequence that will rotate the turntable to the
    CW position
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('ROTATE_TT_CW',
                     SetArmPosition(position='cwpos'))
        Sequence.add('TT_ROTATED',
                     CheckArmReachedKnownPosition(position='cwpos'),
                     transitions={'failed': 'TT_ROTATED'})
        Sequence.add('CHECK_TT_IS_NOT_MOVING', CheckArmIsNotMoving(),
                     transitions={'failed': 'CHECK_TT_IS_NOT_MOVING'})
    return seq


def rotateToCCW():
    """
    Return a SMACH Sequence that will move the arm to the learning pose.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('ROTATE_TT_CW',
                     SetArmPosition(position='ccwpos'))
        Sequence.add('TT_ROTATED',
                     CheckArmReachedKnownPosition(position='ccwpos'),
                     transitions={'failed': 'TT_ROTATED'})
        Sequence.add('CHECK_TT_IS_NOT_MOVING', CheckArmIsNotMoving(),
                     transitions={'failed': 'CHECK_TT_IS_NOT_MOVING'})
    return seq

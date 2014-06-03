#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'hobbit_move'
DEBUG = True

import roslib
roslib.load_manifest(PKG)
import rospy

from smach import State, Sequence
from smach_ros import ServiceState
from hobbit_msgs.srv import GetCoordinates, GetCoordinatesRequest
from std_msgs.msg import String
#import uashh_smach.util as util
import uashh_smach.platform.move_base as move_base
from math import pi


class Stop(State):
    """
    Publish the stop message to mira
    topic: /stop_request
    """
    def __init__(self, angle=0):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted']
        )
        self.stop_pub = rospy.Publisher('/stop_request', String, latch=False)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        self.stop_pub.publish('stop') 
        return 'succeeded'


class SetRotationGoal(State):
    """
    Read the current pose from userdata and applies the rotation to
    the yaw value, given by the angle parameter.

    angle: defaults to 0
    """
    def __init__(self, angle=0):
        State.__init__(
            self,
            input_keys=['x', 'y', 'yaw'],
            output_keys=['x', 'y', 'yaw'],
            outcomes=['succeeded', 'preempted', 'aborted']
        )
        self._angle = angle

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if DEBUG:
            print('before: %f' % ud.yaw)
        ud.yaw = ud.yaw + (self._angle / 180) * pi
        if DEBUG:
            print('after: %f' % ud.yaw)
        return 'succeeded'


class SetNavigationGoal(ServiceState):
    """
    Given place and room the x,y,theta values are retrieved
    and stored in the userdata.
    """
    def __init__(self, frame='/map', room=None, place='dock'):
        print('Inside SetNavigationGoal __init__')
        ServiceState.__init__(
            self,
            'get_coordinates',
            GetCoordinates,
            request_cb=self.__request_cb,
            response_cb=self.__response_cb,
            input_keys=['room_name', 'location_name'],
            output_keys=['x', 'y', 'yaw']
        )
        self.frame = frame
        self.room = room
        self.place = place

    def __request_cb(self, ud, request):
        print('Inside SetNavigationGoal')
        if ud.room_name:
            self.room = ud.room_name
            if ud.location_name:
                self.place = ud.location_name
            else:
                self.place = String('default')
        request = GetCoordinatesRequest()
        #request.header.stamp = rospy.Time.now()
        request.room_name.data = self.room
        request.location_name.data = self.place
        print(request)
        return request

    def __response_cb(self, ud, response):
        if DEBUG:
            print(response)
        ud.x = response.pose.x
        ud.y = response.pose.y
        ud.yaw = response.pose.theta
        return 'succeeded'


def goToPosition(frame='/map', room='', place='dock'):
    """
    Return a SMACH Sequence for navigation to a new position.
    The default values will move the robot to the docking station.

    frame: defaults to /map
    room: defaults to None
    place: defaults to dock
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded'
    )

    with seq:
    #Sequence.add(
    #    'SET_NAV_GOAL_GOTO',
    #    hobbit_move.get_set_nav_goal_state(),
    #    transitions={'aborted': 'failed'}
    #)
        # Sequence.add('SET_NAV_GOAL', SetNavigationGoal(room, place))
        Sequence.add('MOVE_HOBBIT', move_base.MoveBaseState(frame))
    return seq


def goToPose():
    """
    Returns a move_base.MoveBaseState
    We use it so that only hobbit_move has to be imported and all
    SMACH navigation handling is concentrated around
    hobbit_move_import.
    """
    frame = '/map'
    seq = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
        connector_outcome='succeeded',
        input_keys=['x', 'y', 'yaw'])

    with seq:
        Sequence.add('MOVE_BASE_GOAL', move_base.MoveBaseState(frame),
                     remapping={'x':'x',
                                'y':'y',
                                'yaw':'yaw'}
                    )
    return seq
    #return move_base.MoveBaseState(frame)


def getRobotPose():
    """
    Returns a move_base.ReadRobotPositionState
    We use it so that only hobbit_move has to be imported and all
    SMACH navigation handling is concentrated around
    hobbit_move_import.
    """

    frame = '/map'
    return move_base.ReadRobotPositionState(frame)


def rotateRobot(angle=0, frame='/map'):
    """
    Return a SMACH Sequence for rotating the robot for the angle
    in the mathematical positive direction (ccw).

    angle: defaults to 0 or no rotation
    frame: defaults to /map
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded'
    )

    with seq:
        Sequence.add('GET_ROBOT_POSE', move_base.ReadRobotPositionState())
        Sequence.add('SET_ROT_GOAL', SetRotationGoal(angle=angle))
        Sequence.add('ROTATE_ROBOT', move_base.MoveBaseState())
        return seq

def get_set_nav_goal_state(room_name=None, location_name='dock'):
    return SetNavigationGoal(room=None, place='dock')

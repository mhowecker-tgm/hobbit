#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'MoveTo'
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


class SetNavigationGoal(ServiceState):
    """
    Given place and room the x,y,theta values are retrieved
    and stored in the userdata.
    """
    def __init__(self, frame='/map', room=None, place='dock'):
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
        if ud.room_name:
            self.room = ud.room_name
            if ud.location_name:
                self.place = ud.location_name
            else:
                self.place = String(default)
        request = GetCoordinatesRequest()
        request.header.stamp = rospy.Time.now()
        request.room_name.data = self.room
        request.location_name.data = self.place
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
        Sequence.add('SET_NAV_GOAL', SetNavigationGoal(room, place))
        Sequence.add('MOVE_HOBBIT', move_base.MoveBaseState(frame))
    return seq

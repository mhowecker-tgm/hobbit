#!/usr/bin/env python

## Simple demo of a rospy service that return all rooms in of a map

PKG = 'hobbit_msgs' # this package name
NAME = 'FakeUsersCurrentRoom'

import roslib; roslib.load_manifest(PKG) 

from hobbit_msgs.srv import *
from random import choice
import rospy 


def getUsersCurrentRoom(req):
    print 'Returning users current room'
    room_names = ('kitchen', 'office', 'living room', 'bedroom')
    room_name = choice(room_names)
    print room_name
    return GetUsersCurrentRoomResponse(room_name)

def main():
    rospy.init_node(NAME)
    s = rospy.Service('get_users_current_room', GetUsersCurrentRoom, getUsersCurrentRoom)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    main()

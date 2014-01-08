#!/usr/bin/env python

## Simple demo of a rospy service that return all rooms in of a map

PKG = 'hobbit_msgs' # this package name
NAME = 'FakeGetRobotsCurrentRoom'

import roslib; roslib.load_manifest(PKG) 

# import the AddTwoInts service
from hobbit_msgs.srv import *
from hobbit_msgs.srv import GetName
from random import choice
import rospy 


def getCurrentRoom(req):
    print 'Returning robots current room'
    room_names = ('kitchen', 'office', 'living room', 'bedroom')
    room_name = choice(room_names)
    print room_name
    return room_name

def main():
    rospy.init_node(NAME)
    s = rospy.Service('get_robots_current_room', GetName, getCurrentRoom)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    main()

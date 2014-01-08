#!/usr/bin/env python

# Simple demo of a rospy service that return all rooms
# author: Markus Bajones
# email: markus.bajones@gmail.com
#

PKG = 'hobbit_msgs' # this package name
NAME = 'FakeGetAllRoomsService'

import roslib; roslib.load_manifest(PKG) 

# import the AddTwoInts service
from hobbit_msgs.srv import *
from hobbit_msgs.msg import *
import rospy 


def getRooms(req):
    print 'Returning all available rooms'
    global rooms
    return rooms

def main():

    global rooms 
    rooms = RoomsVector()
    
    room = Room()
    place = Place()
    room.room_name = 'kitchen'
    place.place_name = 'default'
    place.x = 2
    place.y = 1
    place.theta = 0
    room.places_vector.append(place)
    
    place1 = Place()
    place1.place_name = 'table'
    place1.theta = 5
    room.places_vector.append(place1)
    rooms.rooms_vector.append(room)
    
    room1 = Room()
    room1.room_name = 'office'
    place2 = Place()
    place2.place_name = 'default'
    place2.x = 3
    place2.y = 4
    place2.theta = 5
    room1.places_vector.append(place2)
    rooms.rooms_vector.append(room1)

    room2 = Room()
    room2.room_name = 'living room'
    place1 = Place()
    place1.place_name = 'default'
    place1.x = 3
    place1.y = 4
    place1.theta = 5
    room2.places_vector.append(place1)
    
    rooms.rooms_vector.append(room2)

    rospy.init_node(NAME)
    s = rospy.Service('get_all_rooms', GetRooms, getRooms)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    main()

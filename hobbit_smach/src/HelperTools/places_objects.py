#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Service that returns all objects and their possible locations
# author: Markus Bajones
# email: markus.bajones@gmail.com
#

PKG = 'hobbit_msgs'  # this package name
NAME = 'place_handler'
PROJECT = 'Hobbit'
update_diff = 0.5

# FILE = '/home/bajo/work/development/catkin/src/navigation/places.xml'
FILE = '/opt/ros/hobbit_hydro/src/navigation/places.xml'

import sys
import rospy
from hobbit_msgs.msg import ObjectLocationVector, ObjectLocation
from hobbit_msgs.msg import *
from hobbit_msgs.srv import GetObjectLocationsRequest, GetRoomNameRequest
from hobbit_msgs.srv import *
from geometry_msgs.msg import Pose2D
from matplotlib.path import Path
from operator import itemgetter
from std_msgs.msg import String
from xml.dom import minidom
import uashh_smach.util as util
from hobbit_msgs import MMUIInterface as MMUI


try:
    from lxml import etree as ET
    # print("running with lxml.etree")
except ImportError:
    print("Failed to import ElementTree from any known place")


def get_unicode(input_string):
    try:
        output_string = unicode(input_string, encoding='utf-8')
        # print((output_string))
        # print(type(output_string))
        return output_string
    except TypeError:
        # print((input_string))
        # print(type(input_string))
        return input_string


def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def readXml(inFile):
    """ Read the xml file with all locations and rooms.
    Store this information inside an hobbit_msgs/Objects
    """
    try:
        tree = ET.ElementTree(file=inFile)
    except IOError as e:
        print("I/O error({0}):\n\' {1} \': {2}"
              .format(e.errno, inFile, e.strerror))
        return None

    rooms = RoomsVector()

    root = tree.getroot()
    if 'rooms' not in root.tag:
        print 'xml file does not contain \'rooms\' as root node.\n Please check the xml file'
        return None

    for room in tree.findall('room'):
        srv_room = Room()
        name = room.findall('name')
        srv_room.room_name = name[0].text
        # print name[0].text
        for vertices in room.findall('vertices'):
            for vertex in vertices.findall('vertex'):
                srv_vertex = Point2D(float(vertex.attrib.get('x')), float(vertex.attrib.get('y')))
                srv_room.vertices_vector.append(srv_vertex)
        for places in room.findall('places'):
            for place in places.findall('place'):
                srv_place = Place()
                name, types = place.find('name'),place.find('type')
                srv_place.place_name = get_unicode(name.text)
                pose = place.find('pose')
                srv_place.x = float(pose.attrib.get('x'))
                srv_place.y = float(pose.attrib.get('y'))
                srv_place.theta = float(pose.attrib.get('theta'))
                # print srv_place
                # this is not yet in the hobbit_msgs/Place definition
                srv_place.place_type = types.text
                for objects in place.findall('objects'):
                    for obj in objects.findall('object'):
                        # this is not yet in the hobbit_msgs/Object definition
                        srv_place.objects.append(Object(String(obj.attrib.get('name')), float(obj.attrib.get('probability'))))
                        #print obj.tag, obj.attrib
                srv_room.places_vector.append(srv_place)
            rooms.rooms_vector.append(srv_room)
    # print rooms
    return rooms


def writeXml(inFile, rooms):
    """ Read and modify the updated probabilities of the objects to the xml file"""
    # outFile = inFile[:-4]+'-out.xml'
    outFile = inFile


    r = ET.Element('rooms')
    for room in rooms.rooms_vector:
        a = ET.SubElement(r, 'room')
        b = ET.SubElement(a, 'name')
        b.text = room.room_name
        c = ET.SubElement(a, 'vertices')
        for v in room.vertices_vector:
            d = ET.SubElement(c, 'vertex', {'x': str(v.x), 'y': str(v.y)})
        e = ET.SubElement(a, 'places')
        for place in room.places_vector:
            f = ET.SubElement(e, 'place')
            g = ET.SubElement(f, 'name')
            g.text = place.place_name
            h = ET.SubElement(f, 'type')
            h.text = place.place_type
            i = ET.SubElement(f, 'pose', {'x': str(place.x), 'y': str(place.y), 'theta': str(place.theta)})
            j = ET.SubElement(f, 'objects')
            for obj in place.objects:
                if type(obj.name) is unicode:   
                    k = ET.SubElement(j, 'object', {'name': obj.name.title(), 'probability': str(obj.probability)})
                else:
                    k = ET.SubElement(j, 'object', {'name': obj.name.data.title(), 'probability': str(obj.probability)})
    tree = ET.ElementTree(r)
    tree.write(outFile, encoding='UTF-8',  xml_declaration=True, pretty_print=True)
    print 'file:',outFile,'written.'

    return True


def count_locations(rooms):
    """
    returns the integer number of search locations
    """
    count = 0
    for room in rooms.rooms_vector:
        for place in (x for x in room.places_vector if x.place_type.lower() == 'searchable'):
            count += 1
    return count


def updateProb(obj, location, room_name, rooms):
    """ Update the probabilities of the given object for all possible locations.
    It will be lowered for all locations except the one it was found.
    """
    obj = get_unicode(obj)
    location = get_unicode(location)
    room_name = get_unicode(room_name)
    global update_diff
    count = count_locations(rooms)
    if count == 0:
        rospy.loginfo('updateProb: The object is not in the database')
        return False
    for room in rooms.rooms_vector:
        for place in (x for x in room.places_vector if x.place_type.lower() == 'searchable'):
            for ob in (z for z in place.objects if z.name.data.lower() == obj.lower()):
                if ((room.room_name.lower() == room_name.lower()) and (place.place_name.lower() == location.lower())):
                    ob.probability = float(ob.probability) * (1 + update_diff)
                else:
                    ob.probability = float(ob.probability) * (1 - update_diff/(count - 1))
    return True


def addObject(object_name, rooms):
    object_name.data = object_name.data.title()
    locations = count_locations(rooms)
    added = False
    for room in rooms.rooms_vector:
        for place in (x for x in room.places_vector if x.place_type.lower() == 'searchable'):
            new = True
            for obj in place.objects:
                if object_name.data.lower() in obj.name.data.lower():
                    print('addObject: ' + object_name.data + ' is already stored.')
                    new = False
                    break
            if new:
                rospy.loginfo('Adding object: '+ object_name.data)
                place.objects.append(Object(String(object_name.data), 1.0/locations))
                added = True
            else:
                break
    if added:
        update_object_list(rooms)
    return added


def update_object_list(rooms):
    if rospy.has_param('ENV/ObjectList'):
            tmp_list = rospy.get_param('ENV/ObjectList')

    for room in rooms.rooms_vector:
        for place in (x for x in room.places_vector if x.place_type.lower() == 'searchable'):
            for obj in place.objects:
                if not obj.name.data.lower() in tmp_list.lower():
                    tmp_list = tmp_list+"+++"+obj.name.data.title()
                else:
                    rospy.loginfo("no need to change /ENV/ObjectList. "+str(obj.name.data.title())+" is already included")
    rospy.set_param('ENV/ObjectList', tmp_list)
    mmui = MMUI.MMUIInterface()
    mmui.request_mmui_update()
    return
            

def getObjectLocations(req):
    """ Given the name of an object its positions (room and location) are returned.
    """
    rospy.loginfo('/get_object_location: Request received')
    query = get_unicode(req.object_name.data)
    places = []
    global rooms
    print 'Return all locations of requested object:',req.object_name.data
    for room in rooms.rooms_vector:
        for place in (x for x in room.places_vector if x.place_type.lower() == 'searchable'):
            for ob in (z for z in place.objects if z.name.data.lower() == query.lower()):
                print(room)
                places.append({'room': room.room_name, 'location': place.place_name, 'probability': ob.probability})
    sorted_places = sorted(places, key=itemgetter('probability'), reverse=True)
    newlist = ObjectLocationVector()
    for obj in sorted_places:
        tmp_obj = ObjectLocation(obj['room'], obj['location'], obj['probability'])
        newlist.locations.append(tmp_obj)
    return newlist

def get_room_name(req):
    """ Given a x and y coordinate the room name is searched for and returned
    """
    rospy.loginfo('/get_room_name: Request received')
    global rooms
    vertices = []
    for room in rooms.rooms_vector:
        for vert in room.vertices_vector:
            vertices.append([vert.x, vert.y])
        print(vertices)
        poly = Path(vertices)
        if poly.contains_point([req.point.x, req.point.y]):
            print room.room_name
            return GetRoomNameResponse(String(room.room_name))
    return GetRoomNameResponse(String('UNKNOWN'))


def getCoordinates(req):
    """ Given the name of the room and of the location their Pose is retrieved
    and returned
    """
    rospy.loginfo('/get_coordinates: Request received for: '+str(req.location_name.data))
    global rooms
    req_room = get_unicode(req.room_name.data)
    req_location = get_unicode(req.location_name.data)
    if req_room == 'dock' or req_room is None \
            or 'call' in req_location or req_room == 'None':
        if req_room == 'dock' or req_location == 'dock':
            req_location = get_unicode('dock')
        gen = (x for x in rooms.rooms_vector)
    else:
        gen = (x for x in rooms.rooms_vector if req_room in x.room_name)

    for x in gen:
        gen1 = (k for k in x.places_vector if req_location in k.place_name)
        for k in gen1:
            pose = Pose2D(float(k.x), float(k.y), float(k.theta))
            rospy.loginfo(str(pose))
            rospy.loginfo('/get_coordinates: Returned coordinates')
            return pose
    rospy.loginfo('/get_coordinates: Returned empty coordinates')
    return (Pose2D(float(0.0), float(0.0), float(0.0)))


def getAllRooms(req):
    """
    Returns a RoomsVector with all available rooms inside.
    """
    global rooms
    rospy.loginfo('/getRooms: Request received')
    print(req)
    # As rooms is already the RoomsVector we are looking for we just return it
    # TODO: The order of the RoomsVector should be with the 6 most important
    # at the front. kitchen, bedroom, livingroom, dining room
    return rooms


def get_robots_current_room(req_old):
    rospy.loginfo('/get_robots_current_room : Request received')
    req = GetRoomNameRequest()
    x, y, yaw = util.get_current_robot_position(frame='/map')
    req.point.x = x
    req.point.y = y
    req.point.z = 0.0
    resp = get_room_name(req)
    rospy.loginfo(resp)
    return resp.room_name.data


def add_object_to_db(req):
    """
    Add the given object to the database
    """
    global rooms
    #rospy.loginfo('/add_object_to_db: Request received')
    return addObject(String(req.object_name), rooms)


def clean_up():
    rospy.loginfo(NAME + ' is shutting down. Saving places.xml')
    writeXml(FILE, rooms)


def update_object_prob(req):
    """
    Update the probability of a detected object.
    Wrapped in a ROS service call.
    """
    global rooms
    return updateProb(req.object_name, req.location_name, req.room_name, rooms)


def main():
    rospy.init_node(NAME)
    global rooms
    rooms = readXml(FILE)
    if not rooms:
        rospy.signal_shutdown('No rooms were loaded. Check the input xml file')
    else:
        addObject(String('aspirin'), rooms)
        #addObject(String('mug'), rooms)
        # updateProb('Häferl', 'side_desk', 'Küche', rooms)
        writeXml(FILE, rooms)
        update_object_list(rooms)
        mug = GetObjectLocationsRequest()
        mug.object_name = String('mug')
        # print(getObjectLocations(mug))
        rospy.Service(PROJECT+'/'+NAME+'/get_object_locations', GetObjectLocations, getObjectLocations)
        rospy.Service(PROJECT+'/'+NAME+'/get_room_name', GetRoomName, get_room_name)
        rospy.Service('/get_coordinates', GetCoordinates, getCoordinates)
        rospy.Service('/getRooms', GetRooms, getAllRooms)
        rospy.Service('/get_robots_current_room', GetName, get_robots_current_room)
        rospy.Service('/add_object_to_db', AddObject, add_object_to_db)
        rospy.Service('/update_object_probabilities', UpdateObject, update_object_prob)

    # spin() keeps Python from exiting until node is shutdown
    rospy.on_shutdown(clean_up)
    rospy.spin()

if __name__ == "__main__":
    # print sys.stdout.encoding
    # stdin, stdout = sys.stdin, sys.stdout
    # reload(sys)
    # sys.stdin, sys.stdout = stdin, stdout
    # sys.setdefaultencoding('utf-8')
    # print sys.stdout.encoding
    main()

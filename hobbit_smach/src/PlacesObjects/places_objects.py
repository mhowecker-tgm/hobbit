#!/usr/bin/env python

# Service that returns all objects and their possible locations
# author: Markus Bajones
# email: markus.bajones@gmail.com
#

PKG = 'hobbit_msgs' # this package name
NAME = 'place_handler'
PROJECT = 'Hobbit'

# FILE='places.xml'
#FILE='/home/bajo/work/development/catkin/src/navigation/places.xml'
FILE='/opt/ros/hobbit_hydro/src/navigation/places.xml'

import roslib; roslib.load_manifest(PKG)
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


try:
      from lxml import etree as ET
      print("running with lxml.etree")
except ImportError:
    try:
        # Python 2.5
        import xml.etree.cElementTree as ET
        print("running with cElementTree on Python 2.5+")
    except ImportError:
        try:
            # Python 2.5
            import xml.etree.ElementTree as ET
            print("running with ElementTree on Python 2.5+")
        except ImportError:
            try:
                # normal cElementTree install
                import cElementTree as ET
                print("running with cElementTree")
            except ImportError:
                try:
                    # normal ElementTree install
                    import elementtree.ElementTree as ET
                    print("running with ElementTree")
                except ImportError:
                    print("Failed to import ElementTree from any known place")

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
        print "I/O error({0}):\n\' {1} \': {2}".format(e.errno, inFile, e.strerror)
        return None

    rooms = RoomsVector()

    root = tree.getroot()
    if not 'rooms' in root.tag:
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
                #print name.text
                srv_place.place_name = name.text
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
                        # print obj.tag, obj.attrib
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
                k = ET.SubElement(j, 'object', {'name': obj.name.data, 'probability': str(obj.probability)})
    tree = ET.ElementTree(r)
    tree.write(outFile, encoding='UTF-8',  xml_declaration=True, pretty_print=True)
    print 'file:',outFile,'written.'

    return True

def updateProb(obj, location, room_name, rooms):
    """ 
    Update the probabilities of the given object for all possible locations.
    It will be lowered for all locations except the one it was found.
    """

    if room_name == 'None':
        print('no room name')
        count = 1
    else:
        count = 0
    rem_prob = 1.0
    for room in rooms.rooms_vector:
        rem_prob = 1.0
        for place in (x for x in room.places_vector if x.place_type.lower() == 'searchable'):
            count += 1
            if ((room.room_name.lower() == room_name.lower()) and (place.place_name.lower() == location.lower())):
                for ob in (z for z in place.objects if z.name.data.lower() == obj.lower()):
                    diff = (1.0 - float(ob.probability))*0.5
                    ob.probability = float(ob.probability) + diff
                    rem_prob = 1.0 - ob.probability
    count = count - 1
    print('updateProb: %d in total' % count)
    print('updateProb: %d remaining probabilty' % rem_prob)
    if count == -1:
        return False
    new_prob = rem_prob / count
    print('updateProb: %d new probabilty' % new_prob)
    for room in (y for y in rooms.rooms_vector if not y.room_name.lower() == room_name.lower()):
        for place in (x for x in room.places_vector if x.place_type.lower() == 'searchable'):
            print('updateProb: place: %s' %str(place))
            for ob in (z for z in place.objects if z.name.data.lower() == obj.lower()):
                ob.probability = new_prob
    return True


def addObject(object_name, rooms):
    for room in rooms.rooms_vector:
        for place in (x for x in room.places_vector if x.place_type.lower() == 'searchable'):
            if not place.objects:
                place.objects.append(Object(object_name, 0.0))
            else:
                new = True
                for obj in place.objects:
                    if object_name.data in obj.name.data:
                        print('addObject: ' + object_name.data + ' is already stored.')
                        new = False
                if new:
                    place.objects.append(Object(object_name, 0.0))

    #print rooms.rooms_vector
    # updateProb(object_name.data, 'None', 'None', rooms)
    return

def getObjectLocations(req):
    """ Given the name of an object its positions (room and location) are returned.
    """
    rospy.loginfo('/get_object_location: Request received')
    query = req.object_name.data
    places = []
    out_places = []
    global rooms
    #print rooms
    print 'Return all locations of requested object:',req.object_name.data
    for room in rooms.rooms_vector:
        for place in (x for x in room.places_vector if x.place_type.lower() == 'searchable'):
            for ob in (z for z in place.objects if z.name.data.lower() == query.lower()):
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
    rospy.loginfo('/get_coordinates: Request received')
    global rooms
    # print(rooms)
    print req.room_name.data, req.location_name.data
    # print 'call' in req.location_name.data
    if req.room_name.data == 'dock' or req.room_name.data == None \
        or 'call' in req.location_name.data or req.room_name.data == 'None':
        rospy.loginfo('dock or bathroom')
        gen = (x for x in rooms.rooms_vector)
    else:
        gen = (x for x in rooms.rooms_vector if req.room_name.data in x.room_name)

    for x in gen:
        print(x)
        gen1 = (k for k in x.places_vector if req.location_name.data in k.place_name)
        for k in gen1:
            print(k)
            pose = Pose2D(float(k.x), float(k.y), float(k.theta))
            print(pose)
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
    # print(rooms)
    # As rooms is already the RoomsVector we are looking for we just return it
    # TODO: The order of the RoomsVector should be with the 6 most important ones
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
    rospy.loginfo('/add_object_to_db: Request received')
    print(req)
    addObject(req.object_name, rooms)
    return True

def main():
    rospy.init_node(NAME)
    global rooms
    rooms = readXml(FILE)
    if not rooms:
        rospy.signal_shutdown('No rooms were loaded. Check the input xml file')
    else:
        addObject(String('mug'), rooms)
        # updateProb('mug', 'side_desk', 'Office', rooms)
        writeXml(FILE, rooms)
        print(rooms)
        mug = GetObjectLocationsRequest()
        mug.object_name = String('mug')
        print(getObjectLocations(mug))
        s1 = rospy.Service(PROJECT+'/'+NAME+'/get_object_locations', GetObjectLocations, getObjectLocations)
        s2 = rospy.Service(PROJECT+'/'+NAME+'/get_room_name', GetRoomName, get_room_name)
        s3 = rospy.Service('/get_coordinates', GetCoordinates, getCoordinates)
        s4 = rospy.Service('/getRooms', GetRooms, getAllRooms)
        s5 = rospy.Service('/get_robots_current_room', GetName, get_robots_current_room)
        s6 = rospy.Service('/add_object_to_db', AddObject, add_object_to_db)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    main()

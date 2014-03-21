#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_msgs'
NAME = 'ARW_BringObject'

FILE_ALL_POSES = "/home/hobbit/ACIN/get_search_positions.log"
FILE_FILTERED_POSES = "/home/hobbit/ACIN/get_filtered_positions.log"

import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import uashh_smach.util as util
import tf
import math
from rospy.service import ServiceException

from std_msgs.msg import String
from hobbit_msgs.msg import LocateUserAction
from hobbit_msgs.srv import *
from hobbit_msgs.msg import *
from hobbit_msgs.srv import *
from smach_ros import ActionServerWrapper, SimpleActionState, ServiceState
from actionlib import SimpleActionServer
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.srv import GetPlan, GetPlanRequest
#from navfn.srv import MakeNavPlan, MakeNavPlanRequest
from geometry_msgs.msg import Pose2D, PoseStamped, Point, Quaternion, Vector3Stamped, PoseWithCovarianceStamped, Pose, PoseArray
from operator import itemgetter
from sensor_msgs.msg import PointCloud2
from semantic_segmentation.srv import get_search_positions, \
        get_search_positionsRequest
from recognizer_msg_and_services.srv import recognize


def point_cloud_cb(msg, ud):
    ud.cloud= msg
    return True


class CheckPoseRoom(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            input_keys=['index',
                        'robots_room_name',
                        'room_name',
                        'name',
                        'robots_room_name2',
                        'search_poses',
                        'verified_poses'],
            output_keys=['search_poses', 'verified_poses'])

    def execute(self, ud):
        if not ud.verified_poses:
            ud.verified_poses = []
        print(bcolors.FAIL)
        print('POSE: room name')
        ud.room_name.data = ud.name.name
        print(ud.name.name)
        print('ROBOT: room name')
        print ud.robots_room_name
        print ud.robots_room_name2
        print bcolors.ENDC 
        #print(len(ud.search_poses))
        #if ud.room_name.data.lower() == ud.robots_room_name.name.lower():
        if ud.room_name.data.lower() == ud.robots_room_name2.data.lower():
            print(bcolors.OKGREEN+'Same room. Keep pose.'+bcolors.ENDC)
            ud.verified_poses.append(ud.search_poses[ud.index])
        else:
            print('Wrong room. Pose behind a wall?')
            # TODO: Remove this point
        return 'succeeded'


class FilterPositions(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['search_poses', 'index', 'point'],
            output_keys=['positions', 'point', 'x', 'y'])

    def execute(self, ud):
        #print('HALLO')
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        else:
            #print(ud.index)
            #print(ud.search_poses[ud.index])
            #ud.point = Point(ud.search_poses[ud.index].x, ud.search_poses[ud.index].y, 0.0)
            ud.x = ud.search_poses[ud.index].x
            ud.y = ud.search_poses[ud.index].y
            #print(type(ud.point))
            #print(ud.point)
            return 'succeeded'


class DummyGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failure', 'preempted'])

    def execute(self, ud):
        rospy.loginfo('Start object recognition ...')
        return 'succeeded'


class MoveHead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'failure'], input_keys=['goal_reached'])
        self.run_counter = 0
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)

    def execute(self, ud):
        if ud.goal_reached == 'aborted':
            return 'failure'
        self.pub_head.publish('up')
        print('MoveHead counter: '+str(self.run_counter))
        if 0 < self.run_counter < 5:
            rospy.loginfo('Moving head to position %d' % (self.run_counter + 1))
            #self.pub.publish(String('Turn 30.0'))
            self.run_counter += 1
            rospy.sleep(1.0)
            print('MoveHead counter: '+str(self.run_counter))
            return 'succeeded'
        elif self.run_counter == 0:
            self.run_counter += 1
            return 'succeeded'
        else:
            self.run_counter = 0
        return 'failure'


class ObjectDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failure', 'preempted', 'aborted'],
                             input_keys=['ids', 'transforms', 'object_name', 'object_pose', 'verified_poses'],
                             output_keys=['object_pose', 'verified_poses'])
        self.counter = 0
        self.pub_head = rospy.Publisher('HeadMove', String)

    def execute(self, ud):
        self.pub_head.publish('down')
        #rospy.loginfo('Did we find the object?')
        #if self.counter == 3:
        #    return 'aborted'
        print(ud.ids)
        print(type(ud.ids))
        if not ud.ids:
         #   self.counter += 1
            return 'failure'
        try:
            for index, item in enumerate(ud.ids):
                if ud.object_name.data in item.data:
                    ud.object_pose = ud.transforms[index]
        #            self.counter = 0
                    return 'succeeded'
        except:
            if ud.object_name.data in ud.ids.data:
                ud.object_pose = ud.transforms
        #        self.counter = 0
                return 'succeeded'
        return 'failure'


class ObjectRecognition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failure', 'preempted'])

#class Rotate(smach.State):
#
#    def __init__(self):
#        smach.State.__init__(
#            self,
#            outcomes=['succeeded', 'preempted', 'failure'],
#            input_keys=['rotation_counter'],
#            output_keys=['rotation_counter'])
#        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)
#        self.pub_head = rospy.Publisher('HeadMove', String)
#        self.pub_obstacle = rospy.Publisher('/headcam/active', String)
#
#    def execute(self, ud):
#        if self.preempt_requested():
#            self.service_preempt()
#            return 'preempted'
#        # self.pub_obstacle.publish('inactive')
#        # self.pub_head.publish(String('top'))
#        rospy.sleep(1.0)
#        self.pub.publish(String('Turn 30.0'))
#        print bcolors.OKGREEN + 'Should start rotating now...' + bcolors.ENDC
#        return 'succeeded'


class Check(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['finished', 'progress'],
            input_keys=['rotation_counter', 'search_poses'],
            output_keys=['rotation_counter'])
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)
        self.pub_poses = rospy.Publisher('/all_search_poses', PoseArray, latch=True)
        self.poses = PoseArray()
        self.poses.poses = []

    def execute(self, ud):
        self.poses.poses = []
        self.pub_poses.publish(self.poses)
        if ud.rotation_counter < 12:
            return 'progress'
        else:
		print(bcolors.OKGREEN+'length of ud.search_poses: '+str(len(ud.search_poses))+bcolors.ENDC)
		for position in ud.search_poses:
		    # poses are only generated and published to show them in rviz
		    self.poses.header.frame_id = 'map'
		    pose = Pose()
		    pose.position = Point(position.x, position.y, 0.0)
		    pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, position.theta)) 
		    self.poses.poses.append(pose)
		    #self.pub_poses.publish(self.poses)

        ud.rotation_counter = 0
        self.pub_head.publish('down')
        rospy.sleep(1.0)
        self.pub_obstacle.publish('active')
        rospy.sleep(1.0)
        self.pub_poses.publish(self.poses)
        self.poses.poses = []
        return 'finished'


def get_search_positions_cb(ud, response):
    if not response:
        print(bcolors.FAIL + 'NO POSE RECEIEVED' + bcolors.ENDC)
        return 'aborted'
    else:
        #print(bcolors.FAIL + 'POSE RECEIEVED' + bcolors.ENDC)
        for pose in response.positions:
            #print(bcolors.OKBLUE + str(pose) + bcolors.ENDC)
            ud.search_poses.append(pose)
            with open(FILE_ALL_POSES, "a") as myfile:
                myfile.write('Calculated search pose\n')
                myfile.write(str(pose))
                myfile.write('\n----------------\n')
        return 'succeeded'


def rotating_cb(msg, ud):
    print(bcolors.WARNING
          + 'ROTATING: received message: /DiscreteMotionState '
          + bcolors.ENDC)
    print msg.data
    # if (msg.data == 'Moving') or (msg.data == 'Turning'):
    if (msg.data == 'Turning'):
        print bcolors.WARNING + 'Now we are actually moving.' + bcolors.ENDC
        rospy.sleep(0.5)
        return True
    else:
        return False


def goal_reached_cb(msg, ud):
    print bcolors.WARNING + 'received message: /goal_status'# + bcolors.ENDC
    print msg.data + bcolors.ENDC
    ud.goal_reached = msg.data
    if (msg.data == 'reached') or (msg.data == 'idle'):
        print bcolors.OKGREEN + 'position reached. Start rotation.' + bcolors.ENDC
        #rospy.sleep(2.0)
        return True
    elif (msg.data == 'aborted'):
        print bcolors.FAIL + 'Unable to reach location.' + bcolors.ENDC
        return True
    else:
        rospy.sleep(1.0)
        return False

def goal_reached_cb2(msg, ud):
    print bcolors.WARNING + 'received message: /goal_status'# + bcolors.ENDC
    print msg.data + bcolors.ENDC
    ud.goal_reached = msg.data
    if (msg.data == 'reached'):
        print bcolors.OKGREEN + 'position reached. Start rotation.' + bcolors.ENDC
        #rospy.sleep(2.0)
        return True
    elif (msg.data == 'aborted'):
        print bcolors.FAIL + 'Unable to reach location.' + bcolors.ENDC
        return True
    else:
        rospy.sleep(1.0)
        return False

def search_poses_cb(ud, request):
    req = get_search_positionsRequest()
    req.robot_pose.pose.pose = ud.robot_current_pose.pose
    req.scene = ud.scene
    with open(FILE_ALL_POSES, "a") as myfile:
        myfile.write('robots current pose\n')
        myfile.write(str(ud.robot_current_pose.pose))
        myfile.write('\n----------------\n')
    return req


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
def disable(self):
        self.HEADER = ''
        self.OKBLUE = ''
        self.OKGREEN = ''
        self.WARNING = ''
        self.FAIL = ''
        self.ENDC = ''

class Init(smach.State):
    """Class to initialize certain parameters"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'canceled'], input_keys=['command'], output_keys=['social_role'])
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)
        self.pub_face.publish('EMO_NEUTRAL')

    def execute(self,ud):
        self.pub_face.publish('EMO_NEUTRAL')
        self.pub_head.publish('down')
        self.pub_obstacle.publish('active')
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        if ud.command.data == 'cancel':
            return 'canceled'
        return 'succeeded'

class CleanUp(smach.State):
    """Class for setting the result message and clean up persistent variables"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                input_keys=['command', 'visited_places', 'search_poses'],
                output_keys=['result','command', 'visited_places'])
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
        ud.visited_places = []
        ud.result = String('Object not detected')
        with open(FILE_FILTERED_POSES, "a") as myfile:
            myfile.write('Remaining search poses\n')
            myfile.write(str(ud.search_poses))
            myfile.write('\n----------------\n')
        return 'succeeded'

class SetSuccess(smach.State):
    """Class for setting the success message in the actionlib result and clean up of persistent variables"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                input_keys=['search_poses'],
                output_keys=['result', 'visited_places'])
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        ud.visited_places = []
        self.pub_face.publish('EMO_HAPPY')
        self.pub.publish('Stop')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('object detected')
        return 'succeeded'

class CleanPositions(smach.State):
    """Class for removing unneeded positions from the rooms. Only use the default 'user search' positions. \
            Remove the waiting, 'object search' and recharge positions"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['response', 'positions'], output_keys=['positions', 'plan'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.positions = []
        for room in ud.response.rooms.rooms_vector:
            for position in room.places_vector:
                # Search position is the position the robot should be able to rotate 360 degree without hitting any objects (e.g. walls)
                if 'default' in position.place_name:
                    ud.positions.append({'x': position.x, 'y': position.y, 'theta': position.theta, 'room': room.room_name, 'distance': 'None', 'place_name':position.place_name, 'penalty':1})
        ud.plan = None
        return 'succeeded'

class PlanPath(smach.State):
    """Class to determine the shortest path to all possible positions, start in the users last known room"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['get_path', 'movement', 'preempted', 'failure'],
                input_keys=['robot_current_pose', 'pose', 'positions', 'detection', 'plan', 'users_current_room', 'visited_places'],
                output_keys=['plan_request', 'detection', 'visited_places', 'robot_end_pose'])
        self.positions=[]
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)
        self.getPlan = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan, persistent=True)
        self.shortest_path = 99999.99

    def execute(self, ud):
    ### The next line of code is dangerous
    ### This will disable the obstacle detection
        self.shortest_path = 99999.99
        #self.pub_obstacle.publish('inactive')
        if self.preempt_requested():
            print 'Preempt requested:'
            self.service_preempt()
            self.getPlan.close()
            return 'preempted'
        try:
            if ud.visited_places[0] is None:
                pass
            else:
                print ud.visited_places
        except:
            ud.visited_places = []

        for position in ud.positions:
            #if  position['room'] in ud.users_current_room.room_name:
            #    print 'User is in %s . Let\'s start there.'%position['room']
            #    position['penalty'] = 0
            position['penalty'] = 0

            print position['room'], position['place_name']
            end_pose = PoseStamped()
            end_pose.header.frame_id = 'map'
            end_pose.pose.position = Point(position['x'], position['y'], 0.0)
            end_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, position['theta']))
            req = GetPlanRequest(ud.robot_current_pose, end_pose, 0.01)
            #print req

            for visited in ud.visited_places:
                if position['room'] == visited['room']:
                    if position['place_name'] == visited['place']:
                        print bcolors.FAIL + 'Remove \'%s\' in %s from list of search locations.'%(position['place_name'], position['room']) + bcolors.ENDC
                        position['penalty'] = 2
                        print position['penalty']
            try:
                resp = self.getPlan(req)
                #print req
            except ServiceException:
                self.getPlan.close()
                return 'failure'

            if resp.plan.poses:
                print bcolors.OKBLUE + 'Plan received'
                print str(len(resp.plan.poses))+ bcolors.ENDC
                # calculate distance
                last = (end_pose.pose.position.x, end_pose.pose.position.y)
                distance = 0.0
                resp.plan.poses.reverse()
                for item in resp.plan.poses:
                    dx = item.pose.position.x - last[0]
                    dy = item.pose.position.y - last[1]
                    distance = round(math.sqrt(dx**2 + dy**2) + distance, 2)
                    last = (item.pose.position.x, item.pose.position.y)
                print bcolors.WARNING + 'distance to %s in %s is %.2f meter'%(position['place_name'],position['room'] ,distance) + bcolors.ENDC
                if (distance < self.shortest_path) and position['penalty'] < 2:
                    self.shortest_path = distance
                    print bcolors.OKGREEN + 'shortest path is now to the %s in the %s'%(position['place_name'],position['room']) + bcolors.ENDC
                    ud.robot_end_pose = {'room': position['room'], 'place':position['place_name'], 'distance':distance, 'penalty':position['penalty']}
                else:
                    pass
        if len(ud.visited_places) == len(ud.positions):
            return 'failure'
        else:
            return 'movement'

class PlanPath2(smach.State):
    """Class to get the next pose from search_poses and converts it to the needed format"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['movement', 'preempted', 'failure'],
                input_keys=['robot_current_pose', 'robot_end_pose', 'pose', 'search_poses', 'verified_poses'],
                output_keys=['detection', 'robot_end_pose'])
        self.pub_search_poses = rospy.Publisher('/search_poses', PoseArray, latch=True)
        self.search_poses = PoseArray()
        self.search_poses.poses = []

    def execute(self, ud):
        if self.preempt_requested():
            print 'Preempt requested:'
            self.service_preempt()
            return 'preempted'
        if len(ud.verified_poses) == 0:
            print(bcolors.OKGREEN+'All poses in this room have been visited. Moving to the next'+bcolors.ENDC)
            return 'failure'
        print(bcolors.OKGREEN)
        print(ud.verified_poses)
        print(bcolors.ENDC)

        for position in ud.verified_poses:
            #print(bcolors.FAIL + 'VISITED_PLACES + position'+bcolors.ENDC)
            #print position
            end_pose = Pose2DStamped()
            end_pose.x = position.x
            end_pose.y = position.y
            end_pose.theta = position.theta
            ud.robot_end_pose = end_pose
            print('length of ud.verified_poses: '+str(len(ud.verified_poses)))
            ud.verified_poses.remove(position)
            print('length of ud.verified_poses: '+str(len(ud.verified_poses)))
            # search_poses are only generated and published to show them in rviz
            self.search_poses.header.frame_id = 'map'
            search_pose = Pose()
            search_pose.position = Point(end_pose.x, end_pose.y, 0.0)
            search_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, end_pose.theta)) 
            self.search_poses.poses.append(search_pose)
            self.pub_search_poses.publish(self.search_poses)
            ud.search_poses.remove(position)
            rospy.sleep(2.0)
            return 'movement'

class MoveBase2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'failure'],
                input_keys=['robot_end_pose'],
                output_keys=['robot_end_pose'])
        self.pub_goal = rospy.Publisher('goal_pose', Pose2DStamped)
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)

    def execute(self, ud):
        self.pub_head.publish(String('down'))
        rospy.sleep(1.0)
        #goal = Pose2DStamped()
        #goal.x = ud.robot_end_pose.x
        #goal.y = ud.robot_end_pose.y
        #goal.theta = ud.robot_end_pose.theta
        self.pub_goal.publish(ud.robot_end_pose)
        rospy.sleep(2.0)
        return 'succeeded'
 

class MoveBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'failure'],
                input_keys=['robot_end_pose', 'visited_places', 'robots_room_name2'],
                output_keys=['robot_end_pose', 'visited_places', 'robots_room_name2'])
        self.pub_room = rospy.Publisher('/room_name_target', String)
        self.pub_place = rospy.Publisher('/place_name_target', String)
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)

    def execute(self, ud):
        self.pub_head.publish(String('down'))
        rospy.sleep(2.0)
        self.pub_obstacle.publish('active')
        print bcolors.OKGREEN+ 'Moving to %s in %s'%(ud.robot_end_pose['place'], ud.robot_end_pose['room'])  + bcolors.ENDC
        self.pub_room.publish(String(ud.robot_end_pose['room']))
        ud.robots_room_name2 = (String(ud.robot_end_pose['room']))
        print ud.robots_room_name2
        rospy.sleep(1.0)
        self.pub_place.publish(String(ud.robot_end_pose['place']))
        rospy.sleep(3.0)
        try:
            ud.visited_places.append(ud.robot_end_pose)
        except:
            ud.visited_places = []
            ud.visited_places.append(ud.robot_end_pose)
        return 'succeeded'

class Rotate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'failure'],
                input_keys=['users_current_room','robot_end_pose', 'visited_places'],
                output_keys=['users_current_room', 'visited_places'])
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.users_current_room = None
        self.pub_obstacle.publish('inactive')
        self.pub_head.publish(String('middle'))
        rospy.sleep(1.0)
        self.pub.publish(String('Turn 30.0'))
        print bcolors.OKGREEN + 'Should start rotating now...' + bcolors.ENDC
        return 'succeeded'


class Update(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            input_keys=['positions', 'rotation_counter'],
            output_keys=['positions', 'rotation_counter'])
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)

    def execute(self, ud):
        ud.rotation_counter += 1
        #self.pub_obstacle.publish('inactive')
        #self.pub_head.publish('up')
        rospy.sleep(0.5)
        return 'succeeded'


class SavePCD(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            input_keys=['scene'])
        self.pub_scene = rospy.Publisher('/save_pcd', PointCloud2)

    def execute(self, ud):
        self.pub_scene.publish(ud.scene)
        return 'succeeded'

def userdetection_cb(msg, ud):
    print bcolors.WARNING + 'received message: '+ bcolors.ENDC
    print msg.header.frame_id
    if not msg.header.frame_id == 'invalid':
        print bcolors.OKGREEN + 'User detected!'+ bcolors.ENDC
        return True
    else:
        return False

def rotating_cb(msg, ud):
    print bcolors.WARNING + 'ROTATING: received message: /DiscreteMotionState '+ bcolors.ENDC
    print msg.data
    #if (msg.data == 'Moving') or (msg.data == 'Turning'):
    if (msg.data == 'Turning'):
        print bcolors.WARNING + 'Now we are actually moving.'+ bcolors.ENDC
        rospy.sleep(0.5)
        return True
    else:
        return False

def rotation_cb(msg, ud):
    print bcolors.WARNING + 'ROTATION FINISHED?: received message: /DiscreteMotionState '+ bcolors.ENDC
    #print bcolors.FAIL
    #print msg.data
    #print bcolors.ENDC
    if (msg.data == 'Stopping') or (msg.data == 'Idle'):
        print bcolors.OKGREEN + '30 degree rotation completed.'+ bcolors.ENDC
        rospy.sleep(1)
        return True
    else:
        return False

def get_robot_pose_cb(msg, ud):
    #print bcolors.WARNING + 'received message: '+ bcolors.ENDC
    #print msg
    try:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = msg.pose.pose.position
        pose.pose.orientation = msg.pose.pose.orientation
        ud.robot_current_pose = pose
        return True
    except:
        print bcolors.FAIL + 'no robot pose message received.'+ bcolors.ENDC
        return False


def scene_cb(msg, ud):
    ud.scene = msg
    return True

def room_name_cb(userdata, response):
    print response
    userdata.room_name = response
    return 'succeeded'

def main():
    rospy.init_node(NAME)

    lu_sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'],
        input_keys = ['command', 'search_poses'],
        output_keys = ['result'])

    lu_sm.userdata.result = String('started')
    lu_sm.userdata.detection = False

    with lu_sm:

        loc_sm = smach.StateMachine(
            outcomes=['succeeded', 'aborted', 'preempted', 'next_room'],
            input_keys=['command', 'search_poses', 'verified_poses'],
            output_keys=['result', 'search_poses', 'verified_poses'])
        loc_sm.userdata.visited_places = []
        loc_sm.userdata.positions = []
        loc_sm.userdata.result = String('started locating')
        loc_sm.userdata.object_name = String('asus_box.pcd')

        gsp_sm = smach.StateMachine(
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['command'],
            output_keys=['result', 'search_poses'])

        gsp_sm.userdata.result = String('started')
        gsp_sm.userdata.rotation_counter = 0
        gsp_sm.userdata.search_poses = []

        with gsp_sm:
            smach.StateMachine.add(
                'INIT',
                Init(),
                transitions={
                    'succeeded': 'ROTATE',
                    'canceled': 'aborted'
                }
            )
            smach.StateMachine.add(
                'ROTATE',
                Rotate(),
                transitions={'succeeded': 'ROTATION',
                                'preempted': 'preempted',
                                'failure': 'aborted'
                                }
            )
            smach.StateMachine.add(
                'ROTATION',
                util.WaitForMsgState(
                    '/DiscreteMotionState',
                    String,
                    rotating_cb,
                    timeout=15),
                transitions={'aborted': 'ROTATION',
                                'succeeded': 'ROTATION_FINISHED',
                                'preempted': 'preempted'
                                }
            )
            smach.StateMachine.add(
                'ROTATION_FINISHED',
                util.WaitForMsgState(
                    '/DiscreteMotionState',
                    String,
                    rotation_cb,
                    timeout=2),
                transitions={'aborted': 'ROTATION_FINISHED',
                                'succeeded': 'UPDATE'
                                }
            )
            smach.StateMachine.add(
                'UPDATE',
                Update(),
                transitions={'succeeded': 'GET_SCENE',
                                'preempted': 'preempted'
                                }
            )
            smach.StateMachine.add(
                'GET_SCENE',
                util.WaitForMsgState(
                    '/headcam/depth_registered/points',
                    PointCloud2,
                    scene_cb,
                    output_keys=['scene'],
                    timeout=2
                ),
                transitions={'succeeded': 'SAVE_PCD',
                            'preempted': 'preempted'}
            )
            smach.StateMachine.add(
                'SAVE_PCD',
                SavePCD(),
                transitions={'succeeded':'GET_ROBOT_POSE_2',
                             'preempted':'preempted'}
            )
            smach.StateMachine.add(
                'GET_ROBOT_POSE_2', util.WaitForMsgState(
                    '/amcl_pose',
                    PoseWithCovarianceStamped,
                    get_robot_pose_cb,
                    output_keys=['robot_current_pose'],
                    timeout=120
                ),
                transitions={'succeeded': 'GET_CURRENT_ROOM2',
                            'aborted': 'GET_ROBOT_POSE_2',
                            'preempted': 'preempted'}
            )
            smach.StateMachine.add(
                'GET_CURRENT_ROOM2', 
                ServiceState(
                    'getCurrentRoom', 
                    GetName, 
                    response_key='robots_room_name'
                ), 
                transitions={'succeeded':'GET_SEARCH_POSITIONS'})
            smach.StateMachine.add(
                'GET_SEARCH_POSITIONS',
                ServiceState(
                    'get_search_positions',
                    get_search_positions,
                    request_cb=search_poses_cb,
                    response_cb=get_search_positions_cb,
                    input_keys=['scene', 'robot_current_pose', 'positions', 'search_poses'],
                    output_keys=['positions', 'search_poses']
                ),
                transitions={'succeeded': '360_CHECK',
                            'aborted': 'ROTATE'}
            )
            smach.StateMachine.add(
                '360_CHECK',
                Check(),
                transitions={'finished': 'succeeded',
                            'progress': 'ROTATE'
                            }
            )
        pose_iterator = smach.Iterator(
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['search_poses', 'robots_room_name', 'robots_room_name2'],
            output_keys=['search_poses', 'verified_poses'],
            it=lambda: range(0, len(gsp_sm.userdata.search_poses)),
            it_label='index',
            exhausted_outcome='succeeded'
        )
        with pose_iterator:
            iterator_sm = smach.StateMachine(
                outcomes=['succeeded', 'aborted', 'preempted', 'continue'],
                input_keys=['search_poses', 'index', 'robots_room_name', 'robots_room_name2'],
                output_keys=['search_poses', 'verified_poses']
            )
            iterator_sm.userdata.verified_poses = []

            with iterator_sm:
                smach.StateMachine.add(
                    'FILTER_POSITIONS',
                    FilterPositions(),
                    transitions={'succeeded': 'GET_POSE_ROOM_NAME',
                                 'preempted': 'preempted',
                                 'aborted': 'aborted'
                                 }
                )
                iterator_sm.userdata.room_name = String('')
                smach.StateMachine.add(
                    'GET_POSE_ROOM_NAME',
                    ServiceState(
                        #'/Hobbit/ObjectService/get_room_name',
                        #GetRoomName,
                        'getRoomName',
                        GetNameOfRoom,
                        request_slots = ['x', 'y'],
                        #request_slots = ['point'],
                        response_key='name'),
                        #response_cb=room_name_cb),
                    transitions={'succeeded': 'CHECK_POSE_IN_ROOM'}
                )
                smach.StateMachine.add(
                    'CHECK_POSE_IN_ROOM',
                    CheckPoseRoom(),
                    transitions={'succeeded': 'continue'}
                )

            smach.Iterator.set_contained_state(
                'CONTAINER_STATE',
                iterator_sm,
                loop_outcomes=['continue']
            )


        with loc_sm:
            smach.StateMachine.add(
                'INIT',
                Init(),
                transitions={
                    'succeeded': 'GET_ROBOT_POSE_3',
                    'canceled': 'aborted'
                }
            )
            smach.StateMachine.add(
                'GET_ROBOT_POSE_3', util.WaitForMsgState(
                    '/amcl_pose',
                    PoseWithCovarianceStamped,
                    get_robot_pose_cb,
                    output_keys=['robot_current_pose'],
                    timeout=120
                ),
                transitions={'succeeded': 'PLAN_PATH_2',
                            'aborted': 'GET_ROBOT_POSE_3',
                            'preempted': 'preempted'}
            )
            #smach.StateMachine.add('PLAN_PATH_2', PlanPath2(), transitions={'movement':'MOVE_BASE_GO_2', 'preempted':'aborted', 'failure':'next_room'})
            smach.StateMachine.add('PLAN_PATH_2', PlanPath2(), transitions={'movement':'MOVE_BASE_GO_2', 'preempted':'aborted', 'failure':'next_room'})
            smach.StateMachine.add('MOVE_BASE_GO_2', MoveBase2(), transitions={'succeeded':'LOCATION_REACHED_2', 'preempted':'aborted', 'failure':'aborted'})
            smach.StateMachine.add('LOCATION_REACHED_2', util.WaitForMsgState('/goal_status', String, goal_reached_cb, timeout=5, output_keys=['goal_reached']), transitions={'aborted':'LOCATION_REACHED_2', 'succeeded':'MOVE_HEAD', 'preempted':'aborted' })
            smach.StateMachine.add('MOVE_HEAD', MoveHead(), transitions={'succeeded':'GET_POINT_CLOUD', 'preempted':'aborted', 'failure':'PLAN_PATH_2'})
            smach.StateMachine.add('GET_POINT_CLOUD',
                    util.WaitForMsgState('/headcam/depth_registered/points', PointCloud2, point_cloud_cb, timeout=5, output_keys=['cloud']),
                    transitions={'succeeded':'START_OBJECT_RECOGNITION', 'aborted':'GET_POINT_CLOUD', 'preempted':'aborted'})
            smach.StateMachine.add('START_OBJECT_RECOGNITION',
                    ServiceState('mp_recognition', recognize, request_slots=['cloud'], response_slots=['ids', 'transforms']),
                    transitions={'succeeded':'OBJECT_DETECTED', 'preempted':'aborted', 'aborted':'MOVE_HEAD'})
            smach.StateMachine.add('OBJECT_DETECTED', ObjectDetected(), transitions={'succeeded':'GRASP_OBJECT', 'failure':'MOVE_HEAD', 'preempted':'aborted', 'aborted':'PLAN_PATH_2'})
            smach.StateMachine.add('GRASP_OBJECT', DummyGrasp(), transitions={'succeeded':'succeeded', 'failure':'aborted', 'preempted':'aborted'})
            

        smach.StateMachine.add('INIT', Init(), transitions={'succeeded':'GET_ALL_POSITIONS', 'canceled':'CLEAN_UP'})
        smach.StateMachine.add('GET_ALL_POSITIONS', ServiceState('getRooms', GetRooms, response_key='response'), transitions={'succeeded':'GET_ROBOT_POSE'})
        smach.StateMachine.add('GET_ROBOT_POSE', util.WaitForMsgState('/amcl_pose', PoseWithCovarianceStamped, get_robot_pose_cb, output_keys=['robot_current_pose'], timeout=120),
                transitions={'succeeded':'CLEAN_POSITIONS', 'aborted':'GET_ROBOT_POSE', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('CLEAN_POSITIONS', CleanPositions(), transitions={'succeeded':'GET_CURRENT_ROOM'})
        smach.StateMachine.add('GET_CURRENT_ROOM', ServiceState('getCurrentRoom', GetName, response_key='robots_room_name'), transitions={'succeeded':'PLAN_PATH'})
        smach.StateMachine.add('PLAN_PATH', PlanPath(), transitions={'movement':'MOVE_BASE_GO', 'preempted':'CLEAN_UP', 'get_path':'GET_PATH', 'failure':'CLEAN_UP'})
        smach.StateMachine.add('GET_PATH',
                ServiceState('/move_base/NavfnROS/make_plan', GetPlan, request_key='plan_request', response_key='plan'),
                transitions={'succeeded':'PLAN_PATH', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('MOVE_BASE_GO', MoveBase(), transitions={'succeeded':'LOCATION_REACHED', 'preempted':'CLEAN_UP', 'failure':'CLEAN_UP'})
        smach.StateMachine.add('LOCATION_REACHED', util.WaitForMsgState('/goal_status', String, goal_reached_cb, timeout=5, output_keys=['goal_reached']), transitions={'aborted':'LOCATION_REACHED', 'succeeded':'SEGMENTATION', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('SEGMENTATION', gsp_sm, transitions={'succeeded':'POSE_ITERATOR', 'aborted':'CLEAN_UP', 'preempted':'preempted'})
        smach.StateMachine.add(
            'POSE_ITERATOR',
            pose_iterator,
            transitions={'succeeded': 'LOCATE_OBJECT',
                         'aborted': 'aborted'}
        )
        smach.StateMachine.add('LOCATE_OBJECT', loc_sm, transitions={'succeeded':'SET_SUCCESS', 'aborted':'CLEAN_UP', 'preempted':'preempted', 'next_room':'GET_ROBOT_POSE'})
        smach.StateMachine.add('SET_SUCCESS', SetSuccess(), transitions={'succeeded':'succeeded', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('CLEAN_UP', CleanUp(), transitions={'succeeded':'preempted'})

    asw = smach_ros.ActionServerWrapper(
            'ARW', LocateUserAction, lu_sm,
            ['succeeded'], ['aborted'],['preempted'],
            result_slots_map = {'result':'result'},
            goal_slots_map = {'command':'command'})

    sis = smach_ros.IntrospectionServer('smach_server', lu_sm, '/HOBBIT/ARW_BO_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

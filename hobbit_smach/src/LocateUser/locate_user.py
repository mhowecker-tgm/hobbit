#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_msgs'
NAME = 'LocateUser'

import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import uashh_smach.util as util
import tf
import math
#from rospy.service import ServiceException

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
from geometry_msgs.msg import Pose2D, PoseStamped, Point, Quaternion, Vector3Stamped, PoseWithCovarianceStamped, Pose
from operator import itemgetter

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
                input_keys=['command', 'visited_places'],
                output_keys=['result','command', 'visited_places'])
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
        ud.visited_places = []
        ud.result = String('user not detected')
        return 'succeeded'

class SetSuccess(smach.State):
    """Class for setting the success message in the actionlib result and clean up of persistent variables"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
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
        ud.result = String('user detected')
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
            if  position['room'] in ud.users_current_room.room_name:
                print 'User is in %s . Let\'s start there.'%position['room']
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


        #for position in ud.positions:
        #    if 'None' in position['distance']:
        #        end_pose = PoseStamped()
        #        end_pose.header.frame_id = 'map'
        #        end_pose.pose.position = Point(position['x'], position['y'], 0.0)
        #        end_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, position['theta']))
        #        req = GetPlanRequest(ud.robot_current_pose, end_pose, 0.01)
        #        try:
        #            resp = self.getPlan(req)
        #            #print req
        #        except ServiceException:
        #            self.getPlan.close()
        #            return 'failure'
        #        if resp.plan:
        #            print bcolors.OKBLUE + 'Plan received'
        #            print len(resp.plan.poses)
        #            print bcolors.ENDC
        #        if  position['room'] in ud.users_current_room.room_name:
        #            print 'User is in %s . Let\'s start there.'%position['room']
        #            position['penalty'] = 0
        #        try:
        #            #if ud.plan.plan.poses[-1].pose.position == end_pose.pose.position:
        #            if resp.plan.poses[-1].pose.position == end_pose.pose.position:
        #                last = (end_pose.pose.position.x, end_pose.pose.position.y)
        #                distance = 0.0
        #                #ud.plan.plan.poses.reverse()
        #                resp.plan.poses.reverse()
        #                #for item in ud.plan.plan.poses:
        #                for item in resp.plan.poses:
        #                    dx = item.pose.position.x - last[0]
        #                    dy = item.pose.position.y - last[1]
        #                    distance = math.sqrt(dx**2 + dy**2) + distance
        #                    last = (item.pose.position.x, item.pose.position.y)
        #                distance = round(distance, 2)
        #                print bcolors.WARNING + 'distance to %s in %s is %.2f meter'%(position['place_name'],position['room'] ,distance) + bcolors.ENDC
        #                self.positions.append({'room': position['room'], 'place':position['place_name'], 'distance':distance, 'penalty':position['penalty']})
        #                ud.positions.remove(position)
        #            else:
        #                print bcolors.FAIL + 'I should not be in here' + bcolors.ENDC
        #                pass
        #        except:
        #            #print bcolors.WARNING + 'Unable to access ud.plan.poses' + bcolors.ENDC
        #            pass
        #        #tmp_request = GetPlanRequest(ud.robot_current_pose, end_pose, 0.01)
        #        #tmp_request = MakeNavPlanRequest(ud.robot_current_pose, end_pose)
        #        #ud.plan_request = tmp_request
        #        #return 'get_path'

        #tmp = sorted(self.positions, key=itemgetter('distance'), reverse = False)
        #sorted_positions = sorted(tmp, key=itemgetter('penalty'), reverse = False)
        #self.positions=[]
	    #print sorted_positions

        #try:
        #    for visited in ud.visited_places:
        #        for position in sorted_positions:
        #            if position['room'] == visited['room']:
        #                if position['place'] == visited['place']:
        #                    sorted_positions.remove(position)
        #                    print bcolors.OKBLUE + 'Remove \'%s\' in %s from list of search locations.'%(position['place'], position['room']) + bcolors.ENDC
        #except:
        #    print bcolors.WARNING + 'Started search. No place visited yet.' + bcolors.ENDC

        #try:
        #    ud.robot_end_pose=sorted_positions[0]
        #except:
        #    #self.getPlan.close()
        #    return 'failure'
        #self.getPlan.close()
        #return 'movement'

class MoveBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'failure'],
                input_keys=['robot_end_pose', 'visited_places'],
                output_keys=['robot_end_pose', 'visited_places'])
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
        rospy.sleep(1.0)
        self.pub_place.publish(String(ud.robot_end_pose['place']))
        try:
            ud.visited_places.append(ud.robot_end_pose)
        except:
            ud.visited_places = []
            ud.visited_places.append(ud.robot_end_pose)
        return 'succeeded'

class Rotate180(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'failure'],
                input_keys=['users_current_room','robot_end_pose', 'visited_places'],
                output_keys=['users_current_room', 'visited_places'])
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)
        self.pub_head = rospy.Publisher('HeadMove', String)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.users_current_room = None
        #self.pub_head.publish(String('top'))
        rospy.sleep(1.0)
        self.pub.publish(String('Turn 180.0'))
        print bcolors.OKGREEN + 'Should start rotating now...' + bcolors.ENDC
        #rospy.sleep(3.0)
        return 'succeeded'

class Rotate360(smach.State):
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
        self.pub_head.publish(String('top'))
        rospy.sleep(1.0)
        self.pub.publish(String('Turn 360.0'))
        print bcolors.OKGREEN + 'Should start rotating now...' + bcolors.ENDC
        #rospy.sleep(3.0)
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
    #print msg.data
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
        print bcolors.OKGREEN + '360 degree rotation completed. User not detected'+ bcolors.ENDC
        #rospy.sleep(1.5)
        return True
    else:
        return False

def goal_reached_cb(msg, ud):
    print bcolors.WARNING + 'received message: /goal_status'+ bcolors.ENDC
    #print msg.data
    if (msg.data == 'reached') or (msg.data == 'idle'):
        print bcolors.OKGREEN + 'position reached. Start rotation.' + bcolors.ENDC
        #rospy.sleep(2.0)
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

def main():
    rospy.init_node(NAME)

    lu_sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'],
        input_keys = ['command'],
        output_keys = ['result'])

    lu_sm.userdata.result = String('started')
    lu_sm.userdata.detection = False

    with lu_sm:
        smach.StateMachine.add('INIT', Init(), transitions={'succeeded':'GET_ALL_POSITIONS', 'canceled':'CLEAN_UP'})
        smach.StateMachine.add('GET_ALL_POSITIONS', ServiceState('get_all_rooms', GetRooms, response_key='response'), transitions={'succeeded':'GET_ROBOT_POSE'})
        #smach.StateMachine.add('GET_ROBOT_POSE', DummyGetRobotPose(), transitions={'succeeded':'CLEAN_POSITIONS'})
        smach.StateMachine.add('GET_ROBOT_POSE', util.WaitForMsgState('/amcl_pose', PoseWithCovarianceStamped, get_robot_pose_cb, output_keys=['robot_current_pose'], timeout=120),
                transitions={'succeeded':'CLEAN_POSITIONS', 'aborted':'GET_ROBOT_POSE', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('CLEAN_POSITIONS', CleanPositions(), transitions={'succeeded':'GET_CURRENT_ROOM'})
        smach.StateMachine.add('GET_CURRENT_ROOM', ServiceState('get_robots_current_room', GetName, response_key='robots_room_name'), transitions={'succeeded':'GET_USERS_ROOM'})
        #smach.StateMachine.add('GET_USERS_ROOM', util.WaitForMsgState('/AAL_users_last_known_room', String, get_users_current_room, output_keys=['users_current_room'], timeout=5), transitions={'succeeded':'PLAN_PATH','aborted':'CLEAN_UP', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('GET_USERS_ROOM', ServiceState('get_users_current_room', GetUsersCurrentRoom, response_key='users_current_room'), transitions={'succeeded':'PLAN_PATH', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('PLAN_PATH', PlanPath(), transitions={'movement':'MOVE_BASE_GO', 'preempted':'CLEAN_UP', 'get_path':'GET_PATH', 'failure':'CLEAN_UP'})
        smach.StateMachine.add('GET_PATH',
                ServiceState('/move_base/NavfnROS/make_plan', GetPlan, request_key='plan_request', response_key='plan'),
                #ServiceState('/move_base/NavfnROS/make_plan', MakeNavPlan, request_key='plan_request', response_key='path'),
                transitions={'succeeded':'PLAN_PATH', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('MOVE_BASE_GO', MoveBase(), transitions={'succeeded':'LOCATION_REACHED', 'preempted':'CLEAN_UP', 'failure':'CLEAN_UP'})
        smach.StateMachine.add('LOCATION_REACHED', util.WaitForMsgState('/goal_status', String, goal_reached_cb, timeout=30), transitions={'aborted':'LOCATION_REACHED', 'succeeded':'ROTATE_360', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('ROTATE_360', Rotate360(), transitions={'succeeded':'ROTATION', 'preempted':'CLEAN_UP', 'failure':'GET_ROBOT_POSE'})
        smach.StateMachine.add('ROTATION', util.WaitForMsgState('/DiscreteMotionState', String, rotating_cb, timeout=15), transitions={'aborted':'ROTATION', 'succeeded':'ROTATION_FINISHED', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('ROTATION_FINISHED', util.WaitForMsgState('/DiscreteMotionState', String, rotation_cb, timeout=2), transitions={'aborted':'USER_DETECTION', 'preempted':'CLEAN_UP', 'succeeded':'GET_ROBOT_POSE'})
        smach.StateMachine.add('USER_DETECTION', util.WaitForMsgState('/User', Vector3Stamped, userdetection_cb, timeout=2), transitions={'succeeded':'SET_SUCCESS', 'preempted':'CLEAN_UP', 'aborted':'ROTATION_FINISHED'})
        smach.StateMachine.add('SET_SUCCESS', SetSuccess(), transitions={'succeeded':'succeeded', 'preempted':'CLEAN_UP'})
        smach.StateMachine.add('CLEAN_UP', CleanUp(), transitions={'succeeded':'preempted'})

    asw = smach_ros.ActionServerWrapper(
            'locate_user', LocateUserAction, lu_sm,
            ['succeeded'], ['aborted'],['preempted'],
            result_slots_map = {'result':'result'},
            goal_slots_map = {'command':'command'})

    sis = smach_ros.IntrospectionServer('smach_server', lu_sm, '/HOBBIT/LU_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

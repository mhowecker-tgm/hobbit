#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
PROJECT = 'Hobbit'
NAME = 'BringObject'

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import uashh_smach.util as util
import tf
import math

from smach_ros import SimpleActionState
from std_msgs.msg import String
from hobbit_msgs.msg import GeneralHobbitAction, GeneralHobbitGoal
from hobbit_msgs.srv import GetObjectLocations, GetCoordinates,\
    GetCoordinatesRequest, GetName
from smach_ros import ActionServerWrapper, ServiceState, IntrospectionServer
from sensor_msgs.msg import PointCloud2
from recognizer_msg_and_services.srv import recognize
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, Point, Quaternion,\
    PoseWithCovarianceStamped
import hobbit_smach.head_move_import as head_move
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.bcolors as bcolors
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.logging_import as log


def patrol_cb(ud, goal):
    goal = GeneralHobbitGoal(
        command=String('locateUser'),
        previous_state=String('call_hobbit'),
        parameters=[])
    return goal


class Init(smach.State):
    """Class to initialize certain parameters"""
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'canceled'],
            input_keys=['parameters'],
            output_keys=['social_role', 'object_name'])
        self.pub_obstacle = rospy.Publisher(
            '/headcam/active', String, queue_size=10)

    def execute(self, ud):
        self.pub_obstacle.publish('active')
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.object_name = ud.parameters[0]
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        rospy.set_param('/hobbit/object_to_bring', ud.parameters[0].data)
        return 'succeeded'


class CleanUp(smach.State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['command', 'visited_places'],
            output_keys=['result', 'command', 'visited_places'])

    def execute(self, ud):
        ud.visited_places = []
        ud.result = String('object not found')
        return 'succeeded'


class MoveCounter(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'failure'])
        self.run_counter = -1

    def execute(self, ud):
        print('Recognizer counter: ' + str(self.run_counter))
        if self.run_counter < 3:
            rospy.loginfo('Recognition run: #%d' % (self.run_counter + 2))
            self.run_counter += 1
            return 'succeeded'
        else:
            self.run_counter = -1
        return 'failure'


class SetSuccess(smach.State):
    """
    Class for setting the success message in the actionlib result
    and clean up of persistent variables
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['result'],
                             output_keys=['result', 'visited_places'])
        self.pub = rospy.Publisher(
            '/DiscreteMotionCmd', String, queue_size=10)

    def execute(self, ud):
        ud.visited_places = []
        self.pub.publish('Stop')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('object detected')
        rospy.loginfo(ud.result)
        return 'succeeded'


class CleanPositions(smach.State):
    """Class for removing unneeded positions from the rooms.
    Remove the waiting, 'object search' and recharge positions"""
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'failure'],
            input_keys=['response', 'positions'],
            output_keys=['positions', 'plan', 'visited_places'])
        self.getCoordinates = rospy.ServiceProxy(
            '/get_coordinates', GetCoordinates, persistent=True)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.positions = []
        ud.visited_places = []
        for pos in ud.response.object_locations.locations:
            print pos.room, pos.location, pos.probability
            req = GetCoordinatesRequest(String(pos.room), String(pos.location))
            try:
                resp = self.getCoordinates(req)
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position = Point(
                    resp.pose.x, resp.pose.y, resp.pose.theta)
                pose.pose.orientation = Quaternion(
                    *tf.transformations.quaternion_from_euler(
                        0, 0, resp.pose.theta))
                ud.positions.append(
                    {'pose': pose,
                     'room': pos.room,
                     'distance': 'None',
                     'place_name': pos.location,
                     'penalty': 1,
                     'theta': resp.pose.theta})
            except rospy.ServiceException:
                self.getCoordinates.close()
                return 'failure'
        return 'succeeded'


class PlanPath(smach.State):
    """
    Class to determine the shortest path to all possible positions of the
    desired object
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['success', 'preempted', 'failure'],
            input_keys=['robot_current_pose',
                        'pose',
                        'positions',
                        'detection',
                        'plan',
                        'users_current_room',
                        'visited_places',
                        'robot_end_pose'],
            output_keys=['plan_request',
                         'detection',
                         'visited_places',
                         'robot_end_pose',
                         'goal_position_x',
                         'goal_position_y',
                         'goal_position_yaw'])
        self.positions = []
        self.pub_obstacle = rospy.Publisher(
            '/headcam/active', String, queue_size=10)
        self.getPlan = rospy.ServiceProxy(
            '/make_plan', GetPlan, persistent=True)
        self.shortest_path = 99999.99

    def execute(self, ud):
        # The next line is dangerous
        self.pub_obstacle.publish('inactive')
        if self.preempt_requested():
            print 'Preempt requested: '
            self.pub_obstacle.publish('active')
            self.service_preempt()
            return 'preempted'
        try:
            if ud.visited_places[0] is None:
                pass
            print ud.visited_places
            self.shortest_path = 99999.99
        except:
            ud.visited_places = []
            print bcolors.FAIL + 'ud.visited not set yet.' + bcolors.ENDC

        for position in ud.positions:
            if not (
                any(d['room'] == position['room'] for d in ud.visited_places)
                and any(d['place'] == position['place_name']
                        for d in ud.visited_places)):
                print 'Calculating plan'
                req = GetPlanRequest(
                    ud.robot_current_pose, position['pose'], 0.01)
                try:
                    resp = self.getPlan(req)
                except rospy.ServiceException:
                    self.getPlan.close()
                    print('No plan received')
                    return 'failure'
                if resp.plan.poses:
                    print bcolors.OKBLUE + 'Plan received'+bcolors.ENDC
                    # calculate distance
                    last = (position['pose'].pose.position.x,
                            position['pose'].pose.position.y)
                    distance = 0.0
                    resp.plan.poses.reverse()
                    for item in resp.plan.poses:
                        dx = item.pose.position.x - last[0]
                        dy = item.pose.position.y - last[1]
                        distance = round(math.sqrt(dx**2 + dy**2)
                                         + distance, 2)
                        last = (item.pose.position.x, item.pose.position.y)
                    print(
                        bcolors.WARNING +
                        'distance to %s in %s is %.2f meter' % (
                            position['place_name'], position['room'], distance)
                        + bcolors.ENDC)
                    if (distance < self.shortest_path)\
                            and position['penalty'] < 2:
                        self.shortest_path = distance
                        print('shortest path is now to the %s in the %s'
                              % (position['place_name'], position['room']))
                        ud.robot_end_pose = {
                            'room': position['room'],
                            'place': position['place_name'],
                            'distance': distance,
                            'penalty': position['penalty']}
                        ud.goal_position_x = position['pose'].pose.position.x
                        ud.goal_position_y = position['pose'].pose.position.y
                        ud.goal_position_yaw = position['theta']
                    else:
                        pass
            else:
                pass
        if len(ud.visited_places) == len(ud.positions):
            print(len(ud.visited_places))
            print(len(ud.positions))
            print bcolors.FAIL+'Visited all positions'+bcolors.ENDC
            return 'failure'
        try:
            ud.visited_places.append(ud.robot_end_pose)
        except:
            ud.visited_places = []
            ud.visited_places.append(ud.robot_end_pose)
        return 'success'


class DummyGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failure', 'preempted'])

    def execute(self, ud):
        rospy.loginfo('Start DUMMY object recognition')
        return 'succeeded'


class ObjectDetected(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failure', 'preempted'],
            input_keys=['ids', 'transforms', 'object_name', 'object_pose'],
            output_keys=['object_pose'])

    def execute(self, ud):
        # rospy.loginfo('Did we find the object?')
        print(ud.ids)
        print(type(ud.ids))
        if not ud.ids:
            return 'failure'
        try:
            for index, item in enumerate(ud.ids):
                if ud.object_name.data in item.data:
                    ud.object_pose = ud.transforms[index]
                    return 'succeeded'
        except:
            if ud.object_name.data in ud.ids.data:
                ud.object_pose = ud.transforms
                return 'succeeded'
        return 'failure'


class ObjectRecognition(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failure', 'preempted'])
        self.cloud_sub = rospy.Subscriber('')

    def execute(self, ud):
        rospy.loginfo('Start object recognition')
        return 'succeeded'


def goal_reached_cb(msg, ud):
    print bcolors.WARNING + 'received message: /goal_status' + bcolors.ENDC
    if (msg.data == 'reached') or (msg.data == 'idle'):
        print bcolors.OKGREEN + 'position reached.' + bcolors.ENDC
        return True
    else:
        return False


def get_robot_pose_cb(msg, ud):
    try:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = msg.pose.pose.position
        pose.pose.orientation = msg.pose.pose.orientation
        ud.robot_current_pose = pose
        return True
    except:
        print bcolors.FAIL + 'no robot pose message received.' + bcolors.ENDC
        return False


def point_cloud_cb(msg, ud):
    print('point cloud received')
    ud.cloud = msg
    return True


def main():
    rospy.init_node(NAME)

    bo_sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['parameters'],
        output_keys=['result'])

    bo_sm.userdata.result = String('started')
    bo_sm.userdata.detection = False
    bo_sm.userdata.object_name = String('object_name')

    with bo_sm:
        smach.StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'GET_OBJECTS_POSITIONS',
                         'canceled': 'LOG_ABORT'})
        smach.StateMachine.add(
            'GET_OBJECTS_POSITIONS',
            ServiceState(
                '/Hobbit/ObjectService/get_object_locations',
                GetObjectLocations,
                request_key='object_name',
                response_key='response'),
            transitions={'succeeded': 'CLEAN_POSITIONS',
                         'preempted': 'LOG_PREEMPT'})
        smach.StateMachine.add(
            'CLEAN_POSITIONS',
            CleanPositions(),
            transitions={'succeeded': 'GET_ROBOT_POSE',
                         'failure': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'})
        smach.StateMachine.add(
            'GET_ROBOT_POSE',
            util.WaitForMsgState(
                '/amcl_pose',
                PoseWithCovarianceStamped,
                get_robot_pose_cb,
                output_keys=['robot_current_pose'],
                timeout=5),
            transitions={
                'succeeded': 'GET_ROBOTS_CURRENT_ROOM',
                'aborted': 'GET_ROBOT_POSE',
                'preempted': 'LOG_PREEMPT'})
        smach.StateMachine.add(
            'GET_ROBOTS_CURRENT_ROOM',
            ServiceState(
                'get_robots_current_room',
                GetName,
                response_key='robots_room_name'),
            transitions={'succeeded': 'PLAN_PATH'})
        smach.StateMachine.add(
            'PLAN_PATH',
            PlanPath(),
            transitions={'success': 'MOVE_HEAD_DOWN',
                         'preempted': 'LOG_PREEMPT',
                         'failure': 'LOG_ABORT'})
        smach.StateMachine.add(
            'MOVE_HEAD_DOWN',
            head_move.MoveTo(pose='down_center'),
            transitions={'succeeded': 'MOVE_BASE',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'MOVE_BASE'})
        smach.StateMachine.add(
            'MOVE_BASE',
            hobbit_move.goToPose(),
            transitions={'succeeded': 'MOVE_HEAD',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORT'},
            remapping={'x': 'goal_position_x',
                       'y': 'goal_position_y',
                       'yaw': 'goal_position_yaw'})
        smach.StateMachine.add(
            'MOVE_HEAD',
            head_move.MoveTo(pose='search_table'),
            transitions={'succeeded': 'REC_COUNTER',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'PLAN_PATH'})
        smach.StateMachine.add(
            'REC_COUNTER',
            MoveCounter(),
            transitions={'succeeded': 'GET_POINT_CLOUD',
                         'preempted': 'LOG_PREEMPT',
                         'failure': 'PLAN_PATH'})
        smach.StateMachine.add(
            'GET_POINT_CLOUD',
            util.WaitForMsgState(
                '/headcam/depth_registered/points',
                PointCloud2,
                point_cloud_cb,
                timeout=5,
                output_keys=['cloud']),
            transitions={'succeeded': 'START_OBJECT_RECOGNITION',
                         'aborted': 'GET_POINT_CLOUD',
                         'preempted': 'LOG_PREEMPT'})
        smach.StateMachine.add(
            'START_OBJECT_RECOGNITION',
            ServiceState(
                'mp_recognition',
                recognize,
                request_slots=['cloud'],
                response_slots=['ids', 'transforms']),
            transitions={'succeeded': 'OBJECT_DETECTED',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'REC_COUNTER'})
        smach.StateMachine.add(
            'OBJECT_DETECTED',
            ObjectDetected(),
            transitions={'succeeded': 'GRASP_OBJECT',
                         'failure': 'MOVE_HEAD',
                         'preempted': 'LOG_PREEMPT'})
        smach.StateMachine.add(
            'GRASP_OBJECT',
            DummyGrasp(),
            transitions={'succeeded': 'GOTO_USER',
                         'failure': 'LOG_ABORT',
                         'preempted': 'LOG_PREEMPT'})
        smach.StateMachine.add(
            'GOTO_USER',
            SimpleActionState('locate_user',
                              GeneralHobbitAction,
                              goal_cb=patrol_cb,
                              input_keys=['parameters', 'params']),
            transitions={'succeeded': 'TELL_USER',
                         'aborted': 'LOG_ABORT'}
        )
        smach.StateMachine.add(
            'TELL_USER',
            speech_output.sayTextObject(
                info='T_BM_HERE_IS_THE_OBJECT_O_I_SHOULD_BRING_TO_YOU',
                object_name=bo_sm.userdata.object_name.data
            ),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'LOG_ABORT'}
        )
        smach.StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='BringObject'),
            transitions={'succeeded': 'CLEAN_UP_PREEMPT'}
        )
        smach.StateMachine.add(
            'LOG_ABORT',
            log.DoLogPreempt(scenario='BringObject'),
            transitions={'succeeded': 'CLEAN_UP_ABORT'}
        )
        smach.StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogPreempt(scenario='BringObject'),
            transitions={'succeeded': 'CLEAN_UP_SUCCESS'}
        )
        smach.StateMachine.add(
            'SET_SUCCESS',
            SetSuccess(),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'preempted': 'CLEAN_UP_PREEMPT'})
        smach.StateMachine.add(
            'CLEAN_UP_ABORT',
            CleanUp(),
            transitions={'succeeded': 'aborted'})
        smach.StateMachine.add(
            'CLEAN_UP_PREEMPT',
            CleanUp(),
            transitions={'succeeded': 'preempted'})
        smach.StateMachine.add(
            'CLEAN_UP_SUCCESS',
            CleanUp(),
            transitions={'succeeded': 'succeeded'})

    asw = ActionServerWrapper(
        'bring_object', GeneralHobbitAction, bo_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'parameters': 'parameters'})

    sis = IntrospectionServer('smach_server', bo_sm, '/HOBBIT/BO_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

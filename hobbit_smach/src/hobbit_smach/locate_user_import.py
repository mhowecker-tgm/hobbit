#!/usr/bin/python
# -*- coding: utf-8 -*-

steps = 12

import rospy
import smach
import uashh_smach.util as util
import uashh_smach.platform.move_base as move_base
import tf
import math

from std_msgs.msg import String
from hobbit_msgs.srv import *
from hobbit_msgs.msg import *
from hobbit_msgs.srv import *
from smach import Sequence
from smach_ros import ServiceState
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import hobbit_smach.hobbit_move_import as hobbit_move
# import hobbit_smach.recharge_import as recharge
from hobbit_smach import bcolors
from rgbd_acquisition.msg import Person
import hobbit_smach.head_move_import as head_move


def detectUser():
    sm = smach.StateMachine(
        outcomes=['succeeded', 'preempted', 'failed']
    )
    with sm:
        smach.StateMachine.add(
            'ROTATE',
            hobbit_move.rotateRobot(angle=360/steps, frame='/map'),
            transitions={'succeeded': 'DETECTION',
                         'aborted': 'failed',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'DETECTION',
            util.WaitForMsgState(
                '/persons',
                Person,
                userdetection_cb,
                timeout=1),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'COUNTER',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'COUNTER',
            UserCounter(),
            transitions={'succeeded': 'ROTATE',
                         'aborted': 'failed',
                         'preempted': 'preempted'}
        )
        return sm


class UserCounter(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'])
        self.counter = 0

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('Counter: %d' % self.counter)
        self.counter += 1
        if self.counter < steps:
            return 'succeeded'
        else:
            self.counter = 0
            return 'aborted'


class Init(smach.State):
    """
    Class to initialize certain parameters
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'canceled'],
            input_keys=['command'],
            output_keys=['social_role'])
        self.pub_obstacle = rospy.Publisher(
            '/headcam/active',
            String,
            queue_size=50)
        self.pub_face = rospy.Publisher(
            '/Hobbit/Emoticon', String,
            queue_size=50)
        self.pub_face.publish('EMO_NEUTRAL')

    def execute(self, ud):
        self.pub_face.publish('EMO_NEUTRAL')
        self.pub_obstacle.publish('active')
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        if ud.command.data == 'cancel':
            return 'canceled'
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
        self.pub_face = rospy.Publisher(
            '/Hobbit/Emoticon',
            String,
            queue_size=50)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
        ud.visited_places = []
        ud.result = String('user not detected')
        return 'succeeded'


class SetSuccess(smach.State):
    """
    Class for setting the success message in the actionlib result and clean
    up of persistent variables
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            output_keys=['result', 'visited_places'])
        self.pub_face = rospy.Publisher(
            '/Hobbit/Emoticon',
            String,
            queue_size=50)

    def execute(self, ud):
        ud.visited_places = []
        self.pub_face.publish('EMO_HAPPY')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('user detected')
        return 'succeeded'


class CleanPositions(smach.State):
    """
    Class for removing unneeded positions from the rooms.
    Only use the default 'user search' positions.
    Remove the waiting, 'object search' and recharge positions
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            input_keys=['response', 'positions'],
            output_keys=['positions', 'plan'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.positions = []
        for room in ud.response.rooms.rooms_vector:
            for position in room.places_vector:
                if 'default' in position.place_name:
                    ud.positions.append(
                        {'x': position.x,
                         'y': position.y,
                         'theta': position.theta,
                         'room': room.room_name,
                         'distance': 'None',
                         'place_name': position.place_name,
                         'penalty': 1})
        ud.plan = None
        return 'succeeded'


class PlanPath(smach.State):
    """
    Class to determine the shortest path to all possible positions,
    starting in the users last known room
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['success', 'preempted', 'failure'],
            input_keys=['pose', 'positions', 'detection', 'plan',
                        'users_current_room', 'visited_places',
                        'robot_end_pose', 'x', 'y', 'yaw'],
            output_keys=['detection', 'visited_places', 'robot_end_pose',
                         'goal_position_x', 'goal_position_y',
                         'goal_position_yaw'])
        self.positions = []
        self.pub_obstacle = rospy.Publisher(
            '/headcam/active',
            String,
            queue_size=50)
        self.getPlan = rospy.ServiceProxy(
            'make_plan',
            GetPlan,
            persistent=True)
        self.shortest_path = 99999.99

    def execute(self, ud):
        # The next line of code is dangerous
        # This will disable the obstacle detection
        # self.pub_obstacle.publish('inactive')
        self.shortest_path = 99999.99
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
            if position['room'] in ud.users_current_room.room_name:
                print 'User is in %s . Let\'s start there.' % position['room']
                position['penalty'] = 0

            print position['room'], position['place_name']
            robot_pose = PoseStamped()
            robot_pose.header.frame_id = 'map'
            robot_pose.pose.position = Point(ud.x, ud.y, 0.0)
            robot_pose.pose.orientation =\
                Quaternion(*tf.transformations.quaternion_from_euler(
                    0, 0, ud.yaw))
            end_pose = PoseStamped()
            end_pose.header.frame_id = 'map'
            end_pose.pose.position = Point(position['x'], position['y'], 0.0)
            end_pose.pose.orientation =\
                Quaternion(*tf.transformations.quaternion_from_euler(
                    0, 0, position['theta']))
            req = GetPlanRequest(robot_pose, end_pose, 0.01)
            # print req

            print('ud.visited_places')
            print(ud.visited_places)
            for visited in ud.visited_places:
                if position['room'] == visited['room']:
                    if position['place_name'] == visited['place']:
                        print bcolors.FAIL +\
                            'Remove \'%s\' in %s from list of search locations.\
                            ' % (position['place_name'],
                                 position['room'])\
                            + bcolors.ENDC
                        position['penalty'] = 2
                        print position['penalty']
            try:
                resp = self.getPlan(req)
                # print req
            except ServiceException:
                self.getPlan.close()
                return 'failure'

            if resp.plan.poses:
                print bcolors.OKBLUE + 'Plan received'
                print str(len(resp.plan.poses)) + bcolors.ENDC
                # calculate distance
                last = (end_pose.pose.position.x, end_pose.pose.position.y)
                distance = 0.0
                resp.plan.poses.reverse()
                for item in resp.plan.poses:
                    dx = item.pose.position.x - last[0]
                    dy = item.pose.position.y - last[1]
                    distance = round(math.sqrt(dx**2 + dy**2) + distance, 2)
                    last = (item.pose.position.x, item.pose.position.y)
                print bcolors.WARNING + 'distance to %s in %s is %.2f meter'\
                    % (position['place_name'],
                       position['room'], distance) \
                    + bcolors.ENDC
                if (distance < self.shortest_path) and position['penalty'] < 2:
                    self.shortest_path = distance
                    print bcolors.OKGREEN + \
                        'shortest path is now to the %s in the %s'\
                        % (position['place_name'], position['room'])\
                        + bcolors.ENDC
                    ud.robot_end_pose = {'room': position['room'],
                                         'place': position['place_name'],
                                         'distance': distance,
                                         'penalty': position['penalty']}
                    ud.goal_position_x = position['x']
                    ud.goal_position_y = position['y']
                    ud.goal_position_yaw = position['theta']
                    print(position['theta'])
                else:
                    pass
        print('len(ud.visited_places) %d' % len(ud.visited_places))
        print('len(ud.positions) %d' % len(ud.positions))
        if len(ud.visited_places) == len(ud.positions):
            print bcolors.FAIL+'Visited all positions'+bcolors.ENDC
            return 'failure'
        else:
            ud.visited_places.append(ud.robot_end_pose)
            print(ud.robot_end_pose)
            return 'success'


class Rotate180(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'failure'])
        self.pub = rospy.Publisher(
            '/DiscreteMotionCmd',
            String,
            queue_size=50)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.sleep(1.0)
        self.pub.publish(String('Turn 180.0'))
        print bcolors.OKGREEN + 'Should start rotating now...' + bcolors.ENDC
        return 'succeeded'


class Counter(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['counter'],
            output_keys=['counter'])

    def execute(self, ud):
        print('Counter: %d' % ud.counter)
        ud.counter += 1
        if ud.counter > 1000:
            return 'succeeded'
        else:
            ud.counter = 0
            return 'aborted'


def rotating_cb(msg, ud):
    print bcolors.WARNING +\
        'ROTATING: received message: /DiscreteMotionState '\
        + bcolors.ENDC
    # print msg.data
    if (msg.data == 'Turning'):
        print bcolors.WARNING +\
            'Now we are actually moving.'\
            + bcolors.ENDC
        rospy.sleep(0.5)
        return True
    else:
        return False


def rotation_cb(msg, ud):
    print bcolors.WARNING + 'ROTATION FINISHED?:\
        received message: /DiscreteMotionState ' + bcolors.ENDC
    # print bcolors.FAIL
    # print msg.data
    # print bcolors.ENDC
    if (msg.data == 'Stopping') or (msg.data == 'Idle'):
        print bcolors.OKGREEN +\
            '180 degree rotation completed. User not detected'\
            + bcolors.ENDC
        return True
    else:
        return False


def goal_reached_cb(msg, ud):
    print bcolors.WARNING +\
        'received message: /goal_status'\
        + bcolors.ENDC
    # print msg.data
    if (msg.data == 'reached') or (msg.data == 'idle'):
        print bcolors.OKGREEN +\
            'position reached. Start rotation.'\
            + bcolors.ENDC
        return True
    else:
        return False


def userdetection_cb(msg, ud):
    print bcolors.OKGREEN + 'received message: '
    print msg
    print bcolors.ENDC
    if 0.39 < msg.confidence <= 0.61:
        print bcolors.OKGREEN + 'Face detected!' + bcolors.ENDC
        rospy.loginfo('The face detection is not good enough to rely on.')
        return False
    elif msg.confidence > 0.61:
        print bcolors.OKGREEN + 'Skeleton detected!' + bcolors.ENDC
        return True
    else:
        print bcolors.FAIL + 'NO USER' + bcolors.ENDC
        return False


def child_term_cb(outcome_map):
    if outcome_map['DETECT_USER'] == 'succeeded'\
            or outcome_map['ROTATE'] == 'succeeded':
        print('child_term_cb: succeeded')
        return True
    else:
        return False


def out_cb(outcome_map):
    if outcome_map['DETECT_USER'] == 'succeeded':
        print('out_cb: succeeded')
        return 'succeeded'
    else:
        return 'failed'


def get_detect_user():
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command'],
        output_keys=['result'])

    sm.userdata.result = String('started')
    sm.userdata.detection = False

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='aborted'
    )
    seq.userdata.counter = 0

    with seq:
        Sequence.add('USER_DETECTOR',
                     util.WaitForMsgState(
                         'persons',
                         Person,
                         userdetection_cb,
                         timeout=1)
                     )
        Sequence.add('COUNTER',
                     Counter(),
                     transitions={'succeeded': 'succeeded',
                                  'aborted': 'USER_DETECTOR'}
                     )

    with sm:
        smach.StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'GET_ALL_POSITIONS',
                         'canceled': 'CLEAN_UP'}
        )
        # smach.StateMachine.add(
        #     'DOCK_CHECK',
        #     hobbit_move.undock_if_needed(),
        #     transitions={'succeeded': 'GET_ALL_POSITIONS',
        #                  'canceled': 'CLEAN_UP'}
        # )
        smach.StateMachine.add(
            'GET_ALL_POSITIONS',
            ServiceState('getRooms',
                         GetRooms,
                         response_key='response'),
            transitions={'succeeded': 'GET_ROBOT_POSE',
                         'aborted': 'CLEAN_UP'}
        )
        smach.StateMachine.add(
            'GET_ROBOT_POSE',
            move_base.ReadRobotPositionState(),
            transitions={'succeeded': 'CLEAN_POSITIONS'}
        )
        smach.StateMachine.add(
            'CLEAN_POSITIONS',
            CleanPositions(),
            transitions={'succeeded': 'GET_CURRENT_ROOM'}
        )
        smach.StateMachine.add(
            'GET_CURRENT_ROOM',
            ServiceState('get_robots_current_room',
                         GetName,
                         response_key='robots_room_name'),
            transitions={'succeeded': 'GET_USERS_ROOM',
                         'aborted': 'CLEAN_UP'}
        )
        smach.StateMachine.add(
            'GET_USERS_ROOM',
            ServiceState('get_users_current_room',
                         GetUsersCurrentRoom,
                         response_key='users_current_room'),
            transitions={'succeeded': 'PLAN_PATH',
                         'preempted': 'CLEAN_UP',
                         'aborted': 'CLEAN_UP'}
        )
        smach.StateMachine.add(
            'PLAN_PATH',
            PlanPath(),
            transitions={'success': 'MOVE_HEAD_DOWN',
                         'preempted': 'CLEAN_UP',
                         'failure': 'CLEAN_UP'}
        )
        smach.StateMachine.add(
            'MOVE_HEAD_DOWN',
            head_move.MoveTo(pose='down_center'),
            transitions={'succeeded': 'MOVE_BASE',
                         'preempted': 'CLEAN_UP',
                         'aborted': 'MOVE_BASE'}
        )
        smach.StateMachine.add(
            'MOVE_BASE',
            hobbit_move.goToPose(),
            transitions={'succeeded': 'MOVE_HEAD_UP',
                         'preempted': 'CLEAN_UP',
                         'aborted': 'CLEAN_UP'},
            remapping={'x': 'goal_position_x',
                       'y': 'goal_position_y',
                       'yaw': 'goal_position_yaw'}
        )
        smach.StateMachine.add(
            'MOVE_HEAD_UP',
            head_move.MoveTo(pose='littledown_center'),
            transitions={'succeeded': 'WAIT',
                         'preempted': 'CLEAN_UP',
                         'aborted': 'WAIT'}
        )
        smach.StateMachine.add(
            'WAIT',
            util.SleepState(duration=1),
            transitions={'succeeded': 'USER_DETECTION'}
        )
        smach.StateMachine.add(
            'USER_DETECTION',
            detectUser(),
            transitions={'succeeded': 'SET_SUCCESS',
                         'preempted': 'preempted',
                         'failed': 'PLAN_PATH'}
        )
        smach.StateMachine.add(
            'SET_SUCCESS',
            SetSuccess(),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'CLEAN_UP'}
        )
        smach.StateMachine.add(
            'CLEAN_UP',
            CleanUp(),
            transitions={'succeeded': 'preempted'}
        )
    return sm

if __name__ == '__main__':
    print("Do not call this directly. Import it into your node.")

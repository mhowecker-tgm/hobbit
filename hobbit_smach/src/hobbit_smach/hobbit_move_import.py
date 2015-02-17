#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'hobbit_move'
DEBUG = False
TIMEOUT = 5

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import uashh_smach.util as util
import math

from smach import State, Sequence, StateMachine
from smach_ros import ServiceState, SimpleActionState
from mira_msgs.srv import UserNavMode, ObsNavMode, EmergencyStop
from hobbit_msgs.srv import GetCoordinates, GetCoordinatesRequest, GetName, \
    SwitchVision, SwitchVisionRequest
from move_base_msgs.msg import MoveBaseAction
#from interfaces_mira.msg import MiraDockingAction, MiraDockingGoal
from hobbit_msgs.msg import MiraDockingAction, MiraDockingGoal
from std_msgs.msg import String
from mira_msgs.msg import BatteryState
from hobbit_user_interaction import HobbitMMUI
from uashh_smach.util import SleepState, WaitForMsgState
from actionlib import SimpleActionClient
import uashh_smach.platform.move_base as move_base
import head_move_import as head_move
import speech_output_import as speech_output
# import service_disable_import as service_disable
import arm_move_import as arm_move
from math import pi


def switch_vision_cb(ud, response):
    if response.result:
        return 'succeeded'
    else:
        return 'aborted'

def battery_cb(msg, ud):
    print('Received battery_state message')
    rospy.sleep(2.0)
    if msg.powerSupplyPresent:
        print('I am in the docking station')
        return True
    else:
        print('I am outside of the docking station')
        return False


def undock_if_needed():
    """
    Returns a SMACH StateMachine that check if it is needed to undock
    before any movement is done.
    """
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'])
    with sm:
        StateMachine.add(
            'CHARGE_CHECK',
            WaitForMsgState(
                '/battery_state',
                BatteryState,
                msg_cb=battery_cb
                ),
            transitions={'succeeded': 'UNDOCK',
                         'aborted': 'succeeded',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'UNDOCK',
            # get_undock(),
            get_undock_action(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )
        return sm


def get_undock():
    """
    Returns a SMACH StateMachine that check if it is needed to undock
    before any movement is done.
    """
    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded')
    with seq:
        Sequence.add(
            'UNDOCK',
            Undock()
        )
        Sequence.add(
            'WAIT_BEFORE_MOVEMENT',
            SleepState(duration=5)
        )
    return seq


def docking_result_cb(userdata, status, result):
    rospy.loginfo("docking_result_cb"+str(result))


def get_dock_action():
    """
    Return exactly on SimpleActionState to start the dock procedure
    """
    docking_goal = MiraDockingGoal(docking_task=0)
    state = SimpleActionState('docking_task',
                              MiraDockingAction,
                              goal=docking_goal,
                              result_cb=docking_result_cb,
                              preempt_timeout=rospy.Duration(TIMEOUT),
                              server_wait_timeout=rospy.Duration(TIMEOUT)
                              )
    return state


def get_undock_action():
    """
    Return exactly on SimpleActionState to start the undock procedure
    """
    docking_goal = MiraDockingGoal(docking_task=1)
    state = SimpleActionState('docking_task',
                              MiraDockingAction,
                              goal=docking_goal,
                              result_cb=docking_result_cb,
                              preempt_timeout=rospy.Duration(TIMEOUT),
                              server_wait_timeout=rospy.Duration(TIMEOUT)
                              )
    return state


class SetObstacles(State):
    """

    """
    def __init__(self, active=True):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted']
        )
        self.obstacles = rospy.Publisher('headcam/active', String,
                                         latch=False, queue_size=50)
        self.active = active

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if self.active:
            self.obstacles.publish('active')
        else:
            self.obstacles.publish('inactive')
        return 'succeeded'


class Undock(State):
    """
    Publish the docking message to mira
    topic: /docking_task
    """
    def __init__(self, angle=0):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted']
        )
        self.stop_pub = rospy.Publisher('/docking_task', String,
                                        latch=False, queue_size=50)
        # self.motion_pub = rospy.Publisher('DiscreteMotionCmd', String,
        #                                  latch=False, queue_size=50)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        self.stop_pub.publish('docking_off')
        return 'succeeded'

class MoveDiscrete(State):
    """
    Publish the docking message to mira
    topic: /docking_task
    """
    def __init__(self, motion=None, value=None):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted']
        )
        self.init = False
        self.valid = ['move', 'rotate']
        self._motion = motion
        self._value = str(value)

    def execute(self, ud):
        
        rospy.loginfo(str(self._motion) +' '+str(self._value))
        if self._motion.lower() not in self.valid:
            return 'aborted'
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if not self.init:
            self.motion_pub = rospy.Publisher(
                'DiscreteMotionCmd', String,
                latch=False, queue_size=50)
            self.init = True
        self.motion_pub.publish(String(self._motion +' '+self._value))
        return 'succeeded'


class TestData(State):
    """
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('TEST_DATA: ' + str(ud.robots_room_name))
        return 'succeeded'


class Dock(State):
    """
    Publish the docking message to mira
    topic: /docking_task
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted']
        )
        self.stop_pub = rospy.Publisher('/docking_task', String,
                                        latch=False, queue_size=50)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        self.stop_pub.publish('docking_on')
        return 'succeeded'


class Stop(State):
    """
    Publish the stop message to mira
    topic: /stop_request
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted']
        )
        self.stop_pub = rospy.Publisher('/stop_request', String,
                                        latch=False, queue_size=50)

    def execute(self, ud):
        # This is a safety feature and should not be interruptable
        # if self.preempt_requested():
        #     self.service_preempt()
        #     return 'preempted'
        rospy.loginfo('DO FULL STOP')
        self.stop_pub.publish('stop')
        client = SimpleActionClient(
            'mira_move_base',
            MoveBaseAction)
        if client.wait_for_server(timeout=rospy.Duration(3)):
            client.cancel_all_goals()
            rospy.loginfo('FINISH FULL STOP')
            return 'succeeded'
        else:
            rospy.loginfo('Failed to reach actionserver@FULL STOP')
            return 'aborted'


def get_full_stop():
    """
    Return a SMACH Sequence that cancels all goals on the movebase
    action server.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded',
    )

    rospy.loginfo('CANCEL all Movement')

    with seq:
        Sequence.add(
            'STOP_ALL_MOVEMENT',
            Stop()
        )
        Sequence.add(
            'HEAD_CENTER',
            head_move.MoveTo(pose='center_center')
        )
    return seq


class SetRotationGoal(State):
    """
    Read the current pose from userdata and applies the rotation to
    the yaw value, given by the angle parameter.

    angle: defaults to 0
    """
    def __init__(self, angle=0):
        State.__init__(
            self,
            input_keys=['x', 'y', 'yaw'],
            output_keys=['x', 'y', 'yaw'],
            outcomes=['succeeded', 'preempted', 'aborted']
        )
        self._angle = float(angle)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if DEBUG:
            print('before: %f' % ud.yaw)
        ud.yaw = ud.yaw + (self._angle / 180) * pi
        if DEBUG:
            print('after: %f' % ud.yaw)
        print('goal_x: %s' % ud.x)
        print('goal_y: %s' % ud.y)
        print('goal_yaw: %s' % ud.yaw)
        return 'succeeded'


class SetNavigationGoal(ServiceState):
    """
    Given place and room the x,y,theta values are retrieved
    and stored in the userdata.
    """
    def __init__(self, frame='/map', room=String('None'), place='dock'):
        # print('Inside SetNavigationGoal __init__')
        ServiceState.__init__(
            self,
            'get_coordinates',
            GetCoordinates,
            request_cb=self.__request_cb,
            response_cb=self.__response_cb,
            input_keys=['room_name', 'location_name', 'room'],
            output_keys=['x', 'y', 'yaw', 'room_name', 'location_name']
        )
        self.frame = frame
        self.room = room
        self.place = place

    def __request_cb(self, ud, request):
        # print('Inside SetNavigationGoal')
        # print('userdata: ', ud.room_name, ud.location_name)
        # print(self.room, self.place)
        if not ud.room_name == 'None':
            self.room = ud.room_name
            if not ud.location_name == 'None':
                self.place = ud.location_name
            else:
                self.place = String('default')
        else:
            print('No userdata received.')
            print(self.room)
            print(self.place)
        request = GetCoordinatesRequest()
        # request.header.stamp = rospy.Time.now()
        request.room_name.data = self.room
        request.location_name.data = self.place
        print(request)
        return request

    def __response_cb(self, ud, response):
        if DEBUG:
            print(response)
        ud.x = response.pose.x
        ud.y = response.pose.y
        ud.yaw = response.pose.theta
        # ud.room_name = 'None'
        # ud.location_name = 'None'
        return 'succeeded'


class SetNavGoal(State):
    """
    """
    def __init__(self, room, place):
        State.__init__(
            self,
            output_keys=['x', 'y', 'yaw'],
            outcomes=['succeeded', 'preempted']
        )
        self.goalX, self.goalY, self.goalYAW = self._getGoal(room, place)

    def _getGoal(self, room, place):
        rospy.wait_for_service('get_coordinates')
        serv = rospy.ServiceProxy(
            'get_coordinates',
            GetCoordinates,
            persistent=False
        )
        req = GetCoordinatesRequest(String(room), String(place))
        try:
            resp = serv(req)
        # print(resp.pose.x, resp.pose.y, resp.pose.theta)
            return (resp.pose.x, resp.pose.y, resp.pose.theta)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return (0.0, 0.0, 0.0)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.x, ud.y, ud.yaw = (self.goalX, self.goalY, self.goalYAW)
        return 'succeeded'


def getPosition(room='none', place='dock'):
    rospy.wait_for_service('get_coordinates')
    serv = rospy.ServiceProxy(
        'get_coordinates',
        GetCoordinates,
        persistent=False
    )
    req = GetCoordinatesRequest(String(room), String(place))
    try:
        resp = serv(req)
        return (resp.pose.x, resp.pose.y, resp.pose.theta)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
        return (0.0, 0.0, 0.0)


def goToPosition(frame='/map', room='None', place='dock'):
    """
    Return a SMACH Sequence for navigation to a new position.
    The default values will move the robot to the docking station.

    frame: defaults to /map
    room: defaults to 'None'
    place: defaults to 'dock'
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded',
        input_keys=['room_name', 'location_name']
    )
    rospy.loginfo('goToPosition: {0} {1}'.format(room, place))
    seq.userdata.room_name = room
    seq.userdata.location_name = place

    rospy.loginfo(room + place)
    with seq:
        Sequence.add(
            'MMUI_SAY_IAmComing',
            speech_output.sayText(info='T_CA_IAmComing'),
            transitions={'failed': 'HEAD_UP'}
            #transitions={'failed': 'aborted'}
        )
        Sequence.add(
            'UNDOCK_IF_NEEDED',
            undock_if_needed()
        )
        # Sequence.add('DISABLE_GESTURES',
        #              service_disable.disable_gestures())
        Sequence.add(
            'SWITCH_VISION',
            ServiceState(
                '/vision_system/navigating',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb
            )
        )
        Sequence.add(
            'PREPARE_MOVEMENT',
            ServiceState(
                '/obs_nav_mode',
                ObsNavMode
            )
        )
        Sequence.add('HEAD_DOWN_BEFORE_MOVEMENT',
                     head_move.MoveTo(pose='down_center'))
        Sequence.add('WAIT', SleepState(duration=1))
        Sequence.add('ACTIVATE_OBSTACLES',
                     SetObstacles(active=True))
        Sequence.add('SET_NAV_GOAL', SetNavigationGoal(room, place))
        Sequence.add(
            'CHECK_ARM',
            arm_move.CheckArmAtHomePos()
        )
        if not DEBUG:
            Sequence.add(
                'MOVE_HOBBIT',
                move_base.MoveBaseState(frame),
                transitions={'aborted': 'HEAD_UP'}
            )
        Sequence.add(
            'PREPARE_STOP',
            ServiceState(
                '/user_nav_mode',
                UserNavMode
            )
        )
        Sequence.add(
            'HEAD_UP_AFTER_MOVEMENT',
            head_move.MoveTo(pose='center_center')
        )
        Sequence.add(
            'GET_ROBOTS_CURRENT_ROOM',
            ServiceState(
                'get_robots_current_room',
                GetName,
                response_key='robots_room_name')
        )
        Sequence.add(
            'MMUI_SAY_ReachedPlace',
            speech_output.sayText(info='T_GT_ReachedMyDestination2'),
            transitions={'failed': 'aborted'}
        )
        Sequence.add(
            'SHOW_MENU_MAIN',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'failed': 'HEAD_UP',
                         'succeeded': 'succeeded'}
        )
        Sequence.add(
            'HEAD_UP',
            head_move.MoveTo(pose='center_center')
        )
        Sequence.add(
            'PREPARE_STOP_1',
            ServiceState(
                '/user_nav_mode',
                UserNavMode
            )
        )
        Sequence.add(
            'SHOW_MENU_MAIN_1',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'succeeded': 'aborted',
                         'failed': 'aborted'}
        )
    return seq


def prepareMovement():
    """
    Returns a SMACH Sequence which has to be executed before each
    navigation command.
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded')

    with seq:
        # Sequence.add('DISABLE_GESTURES',
        #              service_disable.disable_gestures())
        Sequence.add('HEAD_DOWN_BEFORE_MOVEMENT',
                     head_move.MoveTo(pose='down_center'))
        Sequence.add('WAIT', SleepState(duration=1))
        Sequence.add(
            'PREPARE_MOVEMENT',
            ServiceState(
                '/obs_nav_mode',
                ObsNavMode
            )
        )
        Sequence.add('ACTIVATE_OBSTACLES',
                     SetObstacles(active=True))
    return seq


def goToPose():
    """
    Returns a move_base.MoveBaseState
    We use it so that only hobbit_move has to be imported and all
    SMACH navigation handling is concentrated around
    hobbit_move_import.
    """
    frame = '/map'
    seq = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                   connector_outcome='succeeded',
                   input_keys=['x', 'y', 'yaw', 'room_name', 'location_name'])

    with seq:
        # TODO: Check if room_name and location_name are available
        Sequence.add(
            'MMUI_SAY_IAmComing',
            speech_output.sayText(info='T_CA_IAmComing'),
            #transitions={'failed': 'aborted'}
            transitions={'failed': 'HEAD_UP'}
        )
        Sequence.add(
            'UNDOCK_IF_NEEDED',
            undock_if_needed()
        )
        Sequence.add(
            'SWITCH_VISION',
            ServiceState(
                '/vision_system/navigating',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb
            )
        )
        Sequence.add(
            'PREPARE_MOVEMENT',
            ServiceState(
                '/obs_nav_mode',
                ObsNavMode
            )
        )
        Sequence.add('HEAD_DOWN_BEFORE_MOVEMENT',
                     head_move.MoveTo(pose='down_center'))
        Sequence.add('WAIT', SleepState(duration=1))
        Sequence.add('ACTIVATE_OBSTACLES',
                     SetObstacles(active=True))
        if not DEBUG:
            Sequence.add('MOVE_BASE_GOAL', move_base.MoveBaseState(frame),
                         remapping={'x': 'x',
                                    'y': 'y',
                                    'yaw': 'yaw'},
                         transitions={'aborted': 'HEAD_UP'}
            )
        Sequence.add(
            'PREPARE_STOP',
            ServiceState(
                '/user_nav_mode',
                UserNavMode
            )
        )
        Sequence.add(
            'HEAD_UP_AFTER_MOVEMENT',
            head_move.MoveTo(pose='center_center')
        )
        # Sequence.add('ENABLE_GESTURES',
        #              service_disable.enable_gestures())
        Sequence.add(
            'MMUI_SAY_ReachedPlace',
            speech_output.sayText(info='T_GT_ReachedMyDestination2'),
            transitions={'failed': 'aborted'}
        )
        Sequence.add(
            'SHOW_MENU_MAIN',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'failed': 'HEAD_UP',
                         'succeeded': 'succeeded'}
        )
        Sequence.add(
            'HEAD_UP',
            head_move.MoveTo(pose='center_center')
        )
        Sequence.add(
            'PREPARE_STOP_1',
            ServiceState(
                '/user_nav_mode',
                UserNavMode
            )
        )
        Sequence.add(
            'SHOW_MENU_MAIN_1',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'succeeded': 'aborted',
                         'failed': 'aborted'}
        )
    return seq


def goToPoseSilent():
    """
    Returns a move_base.MoveBaseState
    We use it so that only hobbit_move has to be imported and all
    SMACH navigation handling is concentrated around
    hobbit_move_import.
    """
    frame = '/map'
    seq = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                   connector_outcome='succeeded',
                   input_keys=['x', 'y', 'yaw'])

    with seq:
        Sequence.add(
            'MMUI_SAY_IAmComing',
            speech_output.sayText(info='T_CA_IAmComing'),
            #transitions={'failed': 'aborted'}
            transitions={'failed': 'HEAD_UP'}
        )
        Sequence.add(
            'UNDOCK_IF_NEEDED',
            undock_if_needed()
        )
        Sequence.add(
            'SWITCH_VISION',
            ServiceState(
                '/vision_system/navigating',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb
            )
        )
        Sequence.add(
            'PREPARE_MOVEMENT',
            ServiceState(
                '/obs_nav_mode',
                ObsNavMode
            )
        )
        Sequence.add('HEAD_DOWN_BEFORE_MOVEMENT',
                     head_move.MoveTo(pose='down_center'))
        Sequence.add('WAIT', SleepState(duration=1))
        Sequence.add('ACTIVATE_OBSTACLES',
                     SetObstacles(active=True))
        if not DEBUG:
            Sequence.add('MOVE_BASE_GOAL', move_base.MoveBaseState(frame),
                         remapping={'x': 'x',
                                    'y': 'y',
                                    'yaw': 'yaw'},
                         transitions={'aborted': 'PREPARE_STOP'}
            )
        Sequence.add(
            'PREPARE_STOP',
            ServiceState(
                '/user_nav_mode',
                UserNavMode
            )
        )
        Sequence.add(
            'HEAD_UP_AFTER_MOVEMENT',
            head_move.MoveTo(pose='center_center')
        )
        # Sequence.add('ENABLE_GESTURES',
        #              service_disable.enable_gestures())
        Sequence.add(
            'SHOW_MENU_MAIN',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'failed': 'HEAD_UP',
                         'succeeded': 'succeeded'}
        )
        Sequence.add(
            'HEAD_UP',
            head_move.MoveTo(pose='center_center')
        )
        Sequence.add(
            'PREPARE_STOP_1',
            ServiceState(
                '/user_nav_mode',
                UserNavMode
            )
        )
        Sequence.add(
            'SHOW_MENU_MAIN_1',
            HobbitMMUI.ShowMenu(menu='MAIN'),
            transitions={'succeeded': 'aborted',
                         'failed': 'aborted'}
        )
    return seq


def getRobotPose():
    """
    Returns a move_base.ReadRobotPositionState
    We use it so that only hobbit_move has to be imported and all
    SMACH navigation handling is concentrated around
    hobbit_move_import.
    """

    frame = '/map'
    return move_base.ReadRobotPositionState(frame)


def rotateRobot(angle=0, frame='/map'):
    """
    Return a SMACH Sequence for rotating the robot for the angle
    in the mathematical positive direction (ccw).

    angle: defaults to 0 or no rotation
    frame: defaults to /map
    """

    seq = Sequence(
        outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded'
    )
    frame = '/map'

    with seq:
        Sequence.add('GET_ROBOT_POSE', move_base.ReadRobotPositionState())
        Sequence.add('SET_ROT_GOAL', SetRotationGoal(angle=angle))
        Sequence.add('ROTATE_ROBOT', move_base.MoveBaseState(frame))
        return seq


class HasMovedFromPreDock(State):
    """
    Return whether the robot moved beyond a given minimum distance
    in a given frame from the pre-dock pose.

    minimum_distance: distance threshold to control outcomes
    frame: frame in which to retrieve the robot's pose, defaults to /map
    """

    def __init__(self, minimum_distance, frame='/map'):
        smach.State.__init__(
            self,
            outcomes=['movement_exceeds_distance',
                      'movement_within_distance',
                      'counter'])
        util.TransformListenerSingleton.init()
        self.minimum_distance = minimum_distance
        self.frame = frame
        self.lastX, self.lastY = self._getXYDock()
        self.counter = 0

    def _getXY(self):
        x, y, _yaw = util.get_current_robot_position(self.frame)
        return x, y

    def _getXYDock(self):
        rospy.wait_for_service('get_coordinates')
        serv = rospy.ServiceProxy(
            'get_coordinates',
            GetCoordinates,
            persistent=False
        )
        req = GetCoordinatesRequest(String('dock'), String('dock'))
        try:
            resp = serv(req)
            return (resp.pose.x, resp.pose.y)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return (0.0, 0.0)
        return (resp.pose.x, resp.pose.y)

    def execute(self, userdata):
        rospy.sleep(1.5)
        currentX, currentY = self._getXY()
        current_distance = math.sqrt(
            math.pow(currentX - self.lastX, 2) +
            math.pow(currentY - self.lastY, 2)
        )
        rospy.loginfo(
            "current XY: %f,%f last XY: %f,%f\
            current distance: %f minimum distance: %f",
            currentX, currentY, self.lastX, self.lastY,
            current_distance, self.minimum_distance)
        if current_distance >= self.minimum_distance:
            return 'movement_exceeds_distance'
        else:
            if self.counter > 10:
                self.counter = 0
                return 'counter'
            else:
                self.counter += 1
                return 'movement_within_distance'


def get_set_nav_goal_state(room_name=None, location_name='dock'):
    return SetNavigationGoal(room=None, place='dock')

#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import smach
import uashh_smach.util as util
import tf
import math

from hobbit_msgs.srv import SwitchVision, GetObjectLocations,\
    SwitchVisionRequest, GetCoordinates, GetCoordinatesRequest
from std_msgs.msg import String
from smach_ros import ServiceState, MonitorState
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import PointCloud2
from recognizer_msg_and_services.srv import recognize
# from hobbit_smach.bcolors import bcolors
import hobbit_smach.hobbit_move_import as hobbit_move
import uashh_smach.platform.move_base as move_base
import hobbit_smach.head_move_import as head_move
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.locate_user_simple_import as locate_user
import hobbit_smach.logging_import as log
import hobbit_smach.call_hobbit_2_import as call_hobbit
import hobbit_smach.record_data_import as record_data
from hobbit_user_interaction import HobbitMMUI
from rospy.exceptions import TransportTerminated

bring_origin = None
bring_destination = None


def switch_vision_cb(ud, response):
    if response.result:
        return 'succeeded'
    else:
        return 'aborted'

class ObjectDetected(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['ids', 'transforms', 'object_name', 'object_pose'],
            output_keys=['object_pose', 'object_room', 'object_location'])

    def execute(self, ud):
        rospy.loginfo('Did we find the object?')
        rospy.loginfo(str(ud.ids))
        #if not ud.ids:
        #    return 'aborted'
        try:
            for index, item in enumerate(ud.ids):
                if ud.object_name.data.lower() in item.data.lower():
                    ud.object_pose = ud.transforms[index]
                    return 'succeeded'
        except:
            if ud.object_name.data.lower() in ud.ids.data.lower():
                ud.object_pose = ud.transforms
                return 'succeeded'
        return 'aborted'

class StoreOrigin(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'])

    def execute(self, ud):
        global bring_origin
        bring_origin = util.get_current_robot_position(frame='/map')
        rospy.loginfo("ORIGIN set to: "+str(bring_origin))
        return 'succeeded'

class GetOrigin(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['x', 'y', 'yaw'],
            output_keys=['x', 'y', 'yaw'])

    def execute(self, ud):
        global bring_origin
        ud.x, ud.y, ud.yaw = bring_origin
        rospy.loginfo('Set x, y, yaw to: '+str(ud.x)+' ,'+str(ud.y)+' ,'+str(ud.yaw))
        return 'succeeded'

class GetDestination(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['x', 'y', 'yaw'],
            output_keys=['x', 'y', 'yaw'])

    def execute(self, ud):
        global bring_destination
        ud.x, ud.y, ud.yaw = bring_destination
        rospy.loginfo('Set x, y, yaw to: '+str(ud.x)+' ,'+str(ud.y)+' ,'+str(ud.yaw))
        return 'succeeded'

class StoreDestination(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'])

    def execute(self, ud):
        global bring_destination
        bring_destination = util.get_current_robot_position(frame='/map')
        rospy.loginfo("DESTINATION set to: "+str(bring_destination))
        return 'succeeded'

def point_cloud_cb(msg, ud):
#def point_cloud_cb(ud, msg):
    print('point cloud received')
    ud.cloud = msg
    return True

def detect_object():
    sm = smach.StateMachine(
        outcomes=['succeeded', 'preempted', 'aborted'],
        input_keys=['object_name', 'object_pose'],
        output_keys=['object_pose', 'object_room', 'object_location']
    )
    counter_it = smach.Iterator(outcomes = ['succeeded', 'preempted', 'aborted'],
                              input_keys = ['object_name', 'object_pose'],
                              output_keys = [],
                              it = lambda: range(0, 3),
                              it_label = 'index',
                              exhausted_outcome = 'aborted')

    container_sm = smach.StateMachine(
        outcomes = ['succeeded','aborted','preempted', 'continue'],
        input_keys=['object_name', 'object_pose'])
    with container_sm:
        smach.StateMachine.add(
            'GET_POINT_CLOUD',
            util.WaitForMsgState(
                '/headcam/depth_registered/points',
                PointCloud2,
                point_cloud_cb,
                output_keys=['cloud'],
                timeout=2
            ),
            transitions={'succeeded': 'START_OBJECT_RECOGNITION',
                         'aborted': 'continue',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'START_OBJECT_RECOGNITION',
            ServiceState(
                'mp_recognition',
                recognize,
                request_slots=['cloud'],
                response_slots=['ids', 'transforms']),
            transitions={'succeeded': 'STORE_DATA_SUCCESS',
                         'preempted': 'preempted',
                         'aborted': 'STORE_DATA_FAILURE'})
        smach.StateMachine.add(
            'STORE_DATA_FAILURE',
            record_data.GrabAndSendData(topic_in='/headcam/depth_registered/points',
                                        topic_out='/bringobject/top_failure/points'),
            transitions={'succeeded': 'continue',
                         'aborted': 'continue',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'STORE_DATA_SUCCESS',
            record_data.GrabAndSendData(topic_in='/headcam/depth_registered/points',
                                        topic_out='/bringobject/top_success/points'),
            transitions={'succeeded': 'OBJECT_DETECTED',
                         'aborted': 'OBJECT_DETECTED',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'OBJECT_DETECTED',
            ObjectDetected(),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'continue',
                         'preempted': 'preempted'})

    with counter_it:
        smach.Iterator.set_contained_state(
            'CONTAINER_STATE',
            container_sm,
            loop_outcomes = ['continue']
        )

    with sm:
        smach.StateMachine.add(
            'MOVE_HEAD',
            head_move.MoveTo(pose='search_table'),
            transitions={'succeeded': 'OBJECT_RECOGNITION_ITERATOR',
                         'preempted': 'preempted',
                         'aborted': 'aborted'})
        smach.StateMachine.add(
            'OBJECT_RECOGNITION_ITERATOR',
            counter_it,
            transitions={'succeeded': 'STORE_DESTINATION',
                         'preempted': 'preempted',
                         'aborted': 'aborted'})
        smach.StateMachine.add(
            'STORE_DESTINATION',
            StoreDestination(),
            transitions={'succeeded': 'succeeded'}
        )
    return sm


class Init(smach.State):
    """
    Class to initialize certain parameters
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'canceled'],
            input_keys=['command', 'parameters'],
            output_keys=['social_role', 'object_name', 'visited_places'])

    def execute(self, ud):
        ud.object_name = ud.parameters[0]
        ud.visited_places = []

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
        ud.result = String('object not detected')
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
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['response', 'positions'],
            output_keys=['positions', 'plan'])
        self.getCoordinates = rospy.ServiceProxy(
            '/get_coordinates',
            GetCoordinates,
            persistent=True)

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            self.getCoordinates.close()
            return 'preempted'
        ud.positions = []
        for location in ud.response.object_locations.locations:
            req = GetCoordinatesRequest(String(location.room), String(location.location))
            try:
                resp = self.getCoordinates(req)
                ud.positions.append(
                    {'x': resp.pose.x,
                     'y': resp.pose.y,
                     'theta': resp.pose.theta,
                     'distance': 'None',
                     'room': location.room,
                     'place_name': location.location})
            except rospy.ServiceException:
                self.getCoordinates.close()
                return 'aborted'
        ud.plan = None
        rospy.loginfo('CleanPositions: ' + str(len(ud.positions)))
        return 'succeeded'


class PlanPath(smach.State):
    """
    Class to determine the shortest path to all possible positions,
    starting in the users last known room
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['positions', 'x', 'y', 'yaw'],
            output_keys=['goal_position_x', 'goal_position_y',
                         'goal_position_yaw'])
        self.pub_obstacle = rospy.Publisher(
            '/headcam/active',
            String,
            queue_size=50)
        self.getPlan = rospy.ServiceProxy(
            'make_plan',
            GetPlan,
            persistent=True)
        self.shortest_path = 99999.99
        self.index = 0
        self.first = True

    def reset(self, ud):
        self.shortest_path = 99999.99
        self.index = 0

    def execute(self, ud):
        # The next line of code is dangerous
        # This will disable the obstacle detection
        # self.pub_obstacle.publish('inactive')
        if self.preempt_requested():
            print 'Preempt requested:'
            self.service_preempt()
            self.getPlan.close()
            return 'preempted'
        if self.first:
            self.positions = ud.positions
            self.first = False

        robot_pose = get_pose_from_xytheta(ud.x, ud.y, ud.yaw)
        if len(self.positions) == 0:
            self.reset(ud)
            self.first = True
            rospy.loginfo('Abort because the list of self.positions is empty.')
            return 'aborted'
        rospy.loginfo('PlanPath: number of possible locations:'
                      + str(len(self.positions)))
        for index, position in enumerate(self.positions):
            rospy.loginfo('index of positions: '+str(index))
            end_pose = get_pose_from_xytheta(
                position['x'], position['y'], position['theta'])
            req = GetPlanRequest(robot_pose, end_pose, 0.01)

            try:
                resp = self.getPlan(req)
            except (rospy.ServiceException, TransportTerminated):
                self.getPlan.close()
                return 'aborted'

            if resp.plan.poses:
                distance = calc_path_length(end_pose, resp.plan.poses)
                rospy.loginfo('Path length: ' + str(distance))
                if (distance < self.shortest_path):
                    self.index = index
                    self.shortest_path = distance
                    ud.goal_position_x = position['x']
                    ud.goal_position_y = position['y']
                    ud.goal_position_yaw = position['theta']
                    rospy.loginfo('Found shorter path')
                else:
                    rospy.loginfo('Another path is shorter')
                    pass
            else:
                rospy.loginfo('We did not receive a plan.')
                self.index = index
                ud.goal_position_x = position['x']
                ud.goal_position_y = position['y']
                ud.goal_position_yaw = position['theta']
                rospy.loginfo('Select random goal')
        try:
            rospy.loginfo('Trying to remove the position#: ' + str(self.index))
            del self.positions[self.index]
            rospy.loginfo('Successfully removed the position')
        except Exception as e:
            print(e)
            rospy.loginfo('All rooms visited')
            self.reset(ud)
            return 'aborted'
        rospy.loginfo('Moving to next position')
        self.reset(ud)
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


def get_pose_from_xytheta(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position = Point(x, y, 0.0)
    pose.pose.orientation =\
        Quaternion(*tf.transformations.quaternion_from_euler(
            0, 0, yaw))
    return pose


def calc_path_length(end_pose, poses):
    last = (end_pose.pose.position.x, end_pose.pose.position.y)
    distance = 0.0
    poses.reverse()
    for item in poses:
        dx = item.pose.position.x - last[0]
        dy = item.pose.position.y - last[1]
        distance = round(math.sqrt(dx**2 + dy**2) + distance, 2)
        last = (item.pose.position.x, item.pose.position.y)
    return distance


def get_bring_object():
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command', 'parameters'],
        output_keys=['result'])

    sm.userdata.result = String('started')
    sm.userdata.detection = False

    with sm:
        smach.StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'GET_ALL_POSITIONS',
                         'canceled': 'LOG_ABORTED'}
        )
        smach.StateMachine.add(
            'GET_ALL_POSITIONS',
            ServiceState(
                '/Hobbit/places_object/get_object_locations',
                GetObjectLocations,
                request_key='object_name',
                response_key='response'),
            transitions={'succeeded': 'UNDOCK_IF_NEEDED',
                         'aborted': 'LOG_ABORTED'}
        )
        smach.StateMachine.add(
            'UNDOCK_IF_NEEDED',
            hobbit_move.undock_if_needed(),
            transitions={'aborted': 'BACK_IF_NEEDED',
                         'succeeded': 'GET_ROBOT_POSE'}
        )
        smach.StateMachine.add(
            'BACK_IF_NEEDED',
            hobbit_move.back_if_needed(),
            transitions={'succeeded': 'GET_ROBOT_POSE',
                         'aborted': 'GET_ROBOT_POSE'}
        )
        smach.StateMachine.add(
            'GET_ROBOT_POSE',
            move_base.ReadRobotPositionState(),
            transitions={'succeeded': 'STORE_ORIGIN'}
        )
        smach.StateMachine.add(
            'STORE_ORIGIN',
            StoreOrigin(),
            transitions={'succeeded': 'CLEAN_POSITIONS'}
        )
        smach.StateMachine.add(
            'CLEAN_POSITIONS',
            CleanPositions(),
            transitions={'succeeded': 'PLAN_PATH',
                         'aborted' : 'BACK_TO_USER_ABORT'}
        )
        smach.StateMachine.add(
            'PLAN_PATH',
            PlanPath(),
            transitions={'succeeded': 'MOVE_BASE',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'BACK_TO_USER_ABORT'}
        )
        smach.StateMachine.add(
            'MOVE_BASE',
            hobbit_move.goToPoseSilent(),
            transitions={'succeeded': 'SWITCH_VISION',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORTED'},
            remapping={'x': 'goal_position_x',
                       'y': 'goal_position_y',
                       'yaw': 'goal_position_yaw'}
        )
        smach.StateMachine.add_auto(
            'SWITCH_VISION',
            ServiceState(
                '/vision_system/locateUser',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb
            ),
            connector_outcomes=['succeeded', 'preempted', 'aborted']
        )
        smach.StateMachine.add(
            'MOVE_HEAD_UP',
            head_move.MoveTo(pose='littledown_center'),
            transitions={'succeeded': 'WAIT',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'WAIT'}
        )
        smach.StateMachine.add(
            'WAIT',
            util.SleepState(duration=0.1),
            transitions={'succeeded': 'OBJECT_DETECTION'}
        )
        smach.StateMachine.add(
            'OBJECT_DETECTION',
            detect_object(),
            transitions={'succeeded': 'BACK_TO_USER_SUCCESS',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'PLAN_PATH'}
        )
        smach.StateMachine.add(
            'BACK_TO_USER_SUCCESS',
            back_to_user(),
            transitions={'succeeded': 'SAY_FOUND_OBJECT',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'SAY_FOUND_OBJECT'}
        )
        smach.StateMachine.add(
            'SAY_FOUND_OBJECT',
            speech_output.say_text_found_object(),
            transitions={'succeeded': 'SAY_BRING_YOU_TO_OBJECT',
                         'failed': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT'}
        )
        smach.StateMachine.add(
            'SAY_BRING_YOU_TO_OBJECT',
            HobbitMMUI.AskYesNo(question='T_BM_GUIDE_USER_TO_OBJECT'),
            transitions={'yes': 'SAY_FOLLOW_ME',
                         'no': 'LOG_SUCCESS',
                         'failed': 'LOG_ABORTED',
                         'timeout': 'LOG_ABORTED',
                         '3times': 'LOG_ABORTED'}
        )
        smach.StateMachine.add(
            'SAY_FOLLOW_ME',
            speech_output.sayText(info='T_BM_FOLLOW_TO_OBJECT'),
            transitions={'succeeded': 'BACK_TO_OBJECT',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'LOG_ABORTED'}
        )
        smach.StateMachine.add(
            'BACK_TO_OBJECT',
            back_to_object(),
            transitions={'succeeded': 'SAY_DETECTED_HERE',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORTED'}
        )
        smach.StateMachine.add(
            'SAY_DETECTED_HERE',
            speech_output.sayText(info='T_BM_FOUND_OBJECT_HERE'),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'LOG_ABORTED'}
        )
        smach.StateMachine.add(
            'BACK_TO_USER_ABORT',
            back_to_user(),
            transitions={'succeeded': 'SAY_NOT_DETECTED',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'SAY_NOT_DETECTED'}
        )
        smach.StateMachine.add(
            'SAY_NOT_DETECTED',
            speech_output.say_object_not_found(),
            transitions={'succeeded': 'LOG_ABORTED',
                         'preempted': 'LOG_PREEMPT',
                         'failed': 'LOG_ABORTED'}
        )
        smach.StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Bring object'),
            transitions={'succeeded': 'succeeded'}
        )
        smach.StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Bring object'),
            transitions={'succeeded': 'preempted'}
        )
        smach.StateMachine.add(
            'LOG_ABORTED',
            log.DoLogAborted(scenario='Bring object'),
            transitions={'succeeded': 'aborted'}
        )
        smach.StateMachine.add(
            'SET_SUCCESS',
            SetSuccess(),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'LOG_PREEMPT'}
        )
    return sm

def back_to_user():
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with sm:
        smach.StateMachine.add_auto(
            'GET_ORIGIN',
            GetOrigin(),
            connector_outcomes=['succeeded']
        )
        smach.StateMachine.add(
            'GOTO_ORIGIN',
            hobbit_move.goToPoseSilent(),
            transitions={'succeeded': 'COME_CLOSER',
                         'preempted': 'preempted',
                         'aborted': 'aborted'}
        )
        smach.StateMachine.add(
            'COME_CLOSER',
            call_hobbit.come_closer_from_everywhere(),
            transitions={'succeeded': 'COME_CLOSER',
                         'preempted': 'preempted',
                         'aborted': 'aborted'}
        )
    return sm

def back_to_object():
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with sm:
        smach.StateMachine.add_auto(
            'GET_DESTINATION',
            GetDestination(),
            connector_outcomes=['succeeded']
        )
        smach.StateMachine.add(
            'GOTO_DESTINATION',
            hobbit_move.goToPose(),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'preempted',
                         'aborted': 'aborted'}
        )
    return sm

if __name__ == '__main__':
    print("Do not call this directly. Import it into your node.")

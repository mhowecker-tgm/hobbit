#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
import uashh_smach.util as util
import tf
import math

from smach import StateMachine, State, Sequence
from std_msgs.msg import String
from hobbit_msgs.srv import GetObjectLocations, GetCoordinates,\
    GetCoordinatesRequest, GetName
from smach_ros import ServiceState
from sensor_msgs.msg import PointCloud2
from recognizer_msg_and_services.srv import recognize
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, Point, Quaternion,\
    PoseWithCovarianceStamped
import hobbit_smach.head_move_import as head_move
import hobbit_smach.arm_move_import as arm_move
import hobbit_smach.hobbit_move_import as hobbit_move
import hobbit_smach.speech_output_import as speech_output
import hobbit_smach.bcolors as bcolors
import hobbit_smach.locate_user_import as locate_user
import hobbit_smach.social_role_import as social_role
import hobbit_smach.logging_import as logging
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions


class Init(State):

    """Class to initialize certain parameters"""

    def __init__(self):
        State.__init__(
            self,
            outcomes=[
                'succeeded',
                'canceled'],
            input_keys=['object_name'],
            output_keys=['social_role'])
        self.pub_obstacle = rospy.Publisher(
            '/headcam/active',
            String,
            queue_size=50)

    def execute(self, ud):
        self.pub_obstacle.publish('active')
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        return 'succeeded'


class CleanUp(State):

    """
    Class for setting the result message and clean up persistent variables
    """

    def __init__(self):
        State.__init__(
            self, outcomes=['succeeded'], input_keys=[
                'command', 'visited_places'], output_keys=[
                'result', 'command', 'visited_places'])

    def execute(self, ud):
        ud.visited_places = []
        ud.result = String('object not found')
        return 'succeeded'


class MoveCounter(State):

    def __init__(self):
        State.__init__(
            self,
            outcomes=[
                'succeeded',
                'preempted',
                'failure'])
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


class Counter(State):

    def __init__(self):
        State.__init__(
            self,
            outcomes=[
                'succeeded',
                'preempted',
                'aborted'])
        self.run_counter = 1

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('counter: %s' % str(self.run_counter))
        if self.run_counter < 2:
            self.run_counter += 1
            return 'succeeded'
        else:
            self.run_counter = 1
        return 'failure'


class SetSuccess(State):

    """
    Class for setting the success message in the actionlib result\
        and clean up of persistent variables
    """

    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'preempted'],
                       input_keys=['result'],
                       output_keys=['result', 'visited_places'])
        self.pub = rospy.Publisher(
            '/DiscreteMotionCmd',
            String,
            queue_size=50)

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


class CleanPositions(State):

    """
    Class for removing unneeded positions from the rooms.
    Remove the waiting, 'object search' and recharge positions
    """

    def __init__(self):
        State.__init__(
            self, outcomes=[
                'succeeded', 'preempted', 'failure'], input_keys=[
                'response', 'positions'], output_keys=[
                'positions', 'plan', 'visited_places'])
        self.getCoordinates = rospy.ServiceProxy(
            '/get_coordinates',
            GetCoordinates,
            persistent=True)

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
                # print resp
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position = Point(
                    resp.pose.x,
                    resp.pose.y,
                    resp.pose.theta)
                pose.pose.orientation = Quaternion(
                    *
                    tf.transformations.quaternion_from_euler(
                        0,
                        0,
                        resp.pose.theta))
                ud.positions.append({'pose': pose,
                                     'room': pos.room,
                                     'distance': 'None',
                                     'place_name': pos.location,
                                     'penalty': 1,
                                     'theta': resp.pose.theta})
            except rospy.ServiceException:
                self.getCoordinates.close()
                return 'failure'
        return 'succeeded'


class PlanPath(State):

    """
    Class to determine the shortest path to all possible positions of the\
    desired object
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=[
                'success',
                'preempted',
                'failure'],
            input_keys=[
                'robot_current_pose',
                'pose',
                'positions',
                'detection',
                'plan',
                'users_current_room',
                'visited_places',
                'robot_end_pose'],
            output_keys=[
                'plan_request',
                'detection',
                'visited_places',
                'robot_end_pose',
                'goal_position_x',
                'goal_position_y',
                'goal_position_yaw'])
        self.positions = []
        self.pub_obstacle = rospy.Publisher(
            '/headcam/active',
            String,
            queue_size=50)
        self.getPlan = rospy.ServiceProxy(
            '/make_plan',
            GetPlan,
            persistent=True)
        self.shortest_path = 99999.99

    def execute(self, ud):
        # The next line is dangerous
        self.pub_obstacle.publish('inactive')
        if self.preempt_requested():
            print 'Preempt requested:'
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
            if not (any(d['room'] ==
                        position['room'] for d in ud.visited_places)
                    and
                    any(d['place'] ==
                        position['place_name'] for d in ud.visited_places)):
                print 'Calculating plan'
                req = GetPlanRequest(
                    ud.robot_current_pose,
                    position['pose'],
                    0.01)
                try:
                    resp = self.getPlan(req)
                except rospy.ServiceException:
                    self.getPlan.close()
                    print('No plan received')
                    return 'failure'
                # print(resp.plan)
                if resp.plan.poses:
                    print bcolors.OKBLUE + 'Plan received' + bcolors.ENDC
                    # calculate distance
                    last = (
                        position['pose'].pose.position.x,
                        position['pose'].pose.position.y)
                    distance = 0.0
                    resp.plan.poses.reverse()
                    for item in resp.plan.poses:
                        dx = item.pose.position.x - last[0]
                        dy = item.pose.position.y - last[1]
                        distance = round(
                            math.sqrt(
                                dx ** 2 +
                                dy ** 2) +
                            distance,
                            2)
                        last = (item.pose.position.x, item.pose.position.y)
                    print bcolors.WARNING +\
                        'distance to %s in %s is %.2f meter' % \
                        (position['place_name'], position['room'], distance)\
                        + bcolors.ENDC
                    if (distance < self.shortest_path) and position[
                            'penalty'] < 2:
                        self.shortest_path = distance
                        print bcolors.OKGREEN +\
                            'shortest path is now to the %s in the %s' %\
                            (position['place_name'], position['room'])\
                            + bcolors.ENDC
                        ud.robot_end_pose = {
                            'room': position['room'],
                            'place': position['place_name'],
                            'distance': distance,
                            'penalty': position['penalty']}
                        ud.goal_position_x = position['pose'].pose.position.x
                        ud.goal_position_y = position['pose'].pose.position.y
                        ud.goal_position_yaw = position['theta']
                        # print(position['theta'])
                    else:
                        pass
            else:
                # print bcolors.FAIL +position['room'], position['place_name']
                # + bcolors.ENDC
                pass
        if len(ud.visited_places) == len(ud.positions):
            print(len(ud.visited_places))
            print(len(ud.positions))
            print bcolors.FAIL + 'Visited all positions' + bcolors.ENDC
            return 'failure'
        try:
            ud.visited_places.append(ud.robot_end_pose)
        except:
            ud.visited_places = []
            ud.visited_places.append(ud.robot_end_pose)
        return 'success'


class DummyGrasp(State):

    def __init__(self):
        State.__init__(
            self,
            outcomes=[
                'succeeded',
                'failure',
                'preempted'])

    def execute(self, ud):
        rospy.loginfo('Start DUMMY object grasping')
        return 'succeeded'


class ObjectDetected(State):

    def __init__(self):
        State.__init__(
            self,
            outcomes=[
                'succeeded',
                'failure',
                'preempted'],
            input_keys=[
                'ids',
                'transforms',
                'object_name',
                'object_pose'],
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


class ObjectRecognition(State):

    def __init__(self):
        State.__init__(
            self,
            outcomes=[
                'succeeded',
                'failure',
                'preempted'])
        self.cloud_sub = rospy.Subscriber('')

    def execute(self, ud):
        rospy.loginfo('Start object recognition')
        return 'succeeded'


def goal_reached_cb(msg, ud):
    print bcolors.WARNING + 'received message: /goal_status' + bcolors.ENDC
    # print msg.data
    if (msg.data == 'reached') or (msg.data == 'idle'):
        print bcolors.OKGREEN + 'position reached.' + bcolors.ENDC
        # rospy.sleep(2.0)
        return True
    else:
        return False


def get_robot_pose_cb(msg, ud):
    # print bcolors.WARNING + 'received message: '+ bcolors.ENDC
    # print msg
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


def get_bring_object():
    bo_sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['object_name'],
        output_keys=['result'])

    bo_sm.userdata.result = String('started')
    bo_sm.userdata.detection = False

    with bo_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={
                'succeeded': 'GET_OBJECTS_POSITIONS',
                'canceled': 'CLEAN_UP'})
        StateMachine.add(
            'GET_OBJECTS_POSITIONS',
            ServiceState(
                '/Hobbit/ObjectService/get_object_locations',
                GetObjectLocations,
                request_key='object_name',
                response_key='response'),
            transitions={
                'succeeded': 'CLEAN_POSITIONS',
                'preempted': 'preempted'})
        StateMachine.add(
            'CLEAN_POSITIONS',
            CleanPositions(),
            transitions={
                'succeeded': 'GET_ROBOT_POSE',
                'failure': 'CLEAN_UP',
                'preempted': 'CLEAN_UP'})
        StateMachine.add(
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
                'preempted': 'CLEAN_UP'})
        StateMachine.add(
            'GET_ROBOTS_CURRENT_ROOM',
            ServiceState(
                'get_robots_current_room',
                GetName,
                response_key='robots_room_name'),
            transitions={
                'succeeded': 'PLAN_PATH'})
        StateMachine.add(
            'PLAN_PATH',
            PlanPath(),
            transitions={
                'success': 'MOVE_HEAD_DOWN',
                'preempted': 'CLEAN_UP',
                'failure': 'CLEAN_UP'})
        StateMachine.add(
            'MOVE_HEAD_DOWN',
            head_move.MoveTo(
                pose='down_center'),
            transitions={
                'succeeded': 'MOVE_BASE',
                'preempted': 'CLEAN_UP',
                'aborted': 'MOVE_BASE'})
        StateMachine.add(
            'MOVE_BASE',
            hobbit_move.goToPose(),
            transitions={
                'succeeded': 'MOVE_HEAD',
                'preempted': 'CLEAN_UP',
                'aborted': 'CLEAN_UP'},
            remapping={
                'x': 'goal_position_x',
                'y': 'goal_position_y',
                'yaw': 'goal_position_yaw'})
        StateMachine.add(
            'MOVE_HEAD',
            head_move.MoveTo(
                pose='search_table'),
            transitions={
                'succeeded': 'REC_COUNTER',
                'preempted': 'CLEAN_UP',
                'aborted': 'PLAN_PATH'})
        StateMachine.add(
            'REC_COUNTER',
            MoveCounter(),
            transitions={
                'succeeded': 'GET_POINT_CLOUD',
                'preempted': 'aborted',
                'failure': 'PLAN_PATH'})
        StateMachine.add(
            'GET_POINT_CLOUD',
            util.WaitForMsgState(
                '/headcam/depth_registered/points',
                PointCloud2,
                point_cloud_cb,
                timeout=5,
                output_keys=['cloud']),
            transitions={
                'succeeded': 'START_OBJECT_RECOGNITION',
                'aborted': 'GET_POINT_CLOUD',
                'preempted': 'CLEAN_UP'})
        StateMachine.add('START_OBJECT_RECOGNITION',
                         ServiceState('mp_recognition',
                                      recognize,
                                      request_slots=['cloud'],
                                      response_slots=[
                                          'ids',
                                          'transforms']),
                         transitions={'succeeded': 'OBJECT_DETECTED',
                                      'preempted': 'CLEAN_UP',
                                      'aborted': 'REC_COUNTER'})
        StateMachine.add(
            'OBJECT_DETECTED',
            ObjectDetected(),
            transitions={
                'succeeded': 'SET_SUCCESS',
                'failure': 'MOVE_HEAD',
                'preempted': 'CLEAN_UP'})
        # StateMachine.add(
        #     'GRASP_OBJECT',
        #     DummyGrasp(),
        #     transitions={
        #         'succeeded': 'SET_SUCCESS',
        #         'failure': 'CLEAN_UP',
        #         'preempted': 'CLEAN_UP'})
        StateMachine.add(
            'SET_SUCCESS',
            SetSuccess(),
            transitions={
                'succeeded': 'succeeded',
                'preempted': 'CLEAN_UP'})
        StateMachine.add(
            'CLEAN_UP',
            CleanUp(),
            transitions={
                'succeeded': 'preempted'})
    return bo_sm


def get_grasping():
    seq = Sequence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        connector_outcome='succeeded'
    )
    with seq:
        Sequence.add(
            'CALC_GRASP_POSE',
            DummyGrasp()
        )
        Sequence.add(
            'MOVE_TO_GRASP',
            hobbit_move.goToPose()
        )
        Sequence.add(
            'SAY_GRASP',
            speech_output.sayText(
                info='T_BM_I_WILL_GRASP_THE_OBJECT'
            )
        )
        Sequence.add(
            'GRASP',
            DummyGrasp()
        )
        Sequence.add(
            'HAPPY',
            speech_output.emo_say_something(
                emo='HAPPY',
                time=4,
                text='T_BM_I_FETCHED_THE_OBJECT_O_FOR_YOU'
            )
        )
        Sequence.add(
            'PUT_ON_TRAY',
            arm_move.goToTrayPosition()
        )
    return seq


def get_bring_scenario():
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['object_name'],
        output_keys=['result']
    )

    with sm:
        StateMachine.add(
            'START',
            speech_output.emo_say_something(
                emo='WONDERING',
                time=4,
                text='T_BM_ConfirmBringMeObject'
            ),
            transitions={'succeeded': 'HAPPY_SAY',
                         'aborted': 'aborted',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'HAPPY_SAY',
            speech_output.emo_say_something(
                emo='HAPPY',
                time=4,
                text='T_BM_WILL_FETCH_OBJECT_O_FOR_YOU'
            ),
            transitions={'succeeded': 'SEARCH_FOR_OBJECT',
                         'aborted': 'aborted',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'SEARCH_FOR_OBJECT',
            get_bring_object(),
            transitions={'succeeded': 'VHAPPY_SAY',
                         'aborted': 'LOCATE_USER_2',
                        'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'VHAPPY_SAY',
            speech_output.emo_say_something(
                emo='VERY_HAPPY',
                time=4,
                text='T_BM_ObjectFoundRepositioning'
            ),
            transitions={'succeeded': 'GRASP_OBJECT',
                         'aborted': 'aborted',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'GRASP_OBJECT',
            DummyGrasp(),
            transitions={
                'succeeded': 'LOCATE_USER',
                'failure': 'LOG_NOT_COMPLETE',
                'preempted': 'LOG_TASK_HAS_PREEMPTED'})
        StateMachine.add(
            'LOCATE_USER',
            locate_user.get_detect_user(),
            transitions={'succeeded': 'HAPPY_SAY_2',
                         'aborted': 'aborted',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'HAPPY_SAY_2',
            speech_output.emo_say_something(
                emo='HAPPY',
                time=4,
                text='T_BM_HERE_IS_THE_OBJECT_O_I_SHOULD_BRING_TO_YOU'
            ),
            transitions={'succeeded': 'HELP_ACCEPTED?',
                         'aborted': 'aborted',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'HELP_ACCEPTED?',
            social_role.CheckHelpAccepted(),
            transitions={'succeeded': 'SAY_THANKS',
                         'aborted': 'succeeded',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'SAY_THANKS',
            speech_output.sayText(
                info='T_BM_THANK_YOU_FOR_HELPING_ME'),
            transitions={'succeeded': 'SAY_ROF',
                         'failed': 'succeeded',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'SAY_ROF',
            HobbitMMUI.AskYesNo(
                question='T_BM_PLEASE_LET_ME_RETURN_THE_FAVOUR'),
            transitions={'yes': 'VHAPPY_SAY_2',
                         'no': 'SAY_MAYBE',
                         'failed': 'SAY_MAYBE',
                         '3times': 'LOG_NOT_COMPLETE',
                         'timeout': 'SAY_MAYBE',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'VHAPPY_SAY_2',
            speech_output.emo_say_something(
                emo='VERRY_HAPPY',
                time=4,
                text='T_BM_SAY_SOMETHING_NICE'
            ),
            transitions={'succeeded': 'MAIN_MENU',
                         'aborted': 'succeeded',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'SAY_MAYBE',
            speech_output.sayText(
                info='T_BM_Maybe_next_time'),
            transitions={'succeeded': 'MAIN_MENU',
                         'failed': 'succeeded',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'MAIN_MENU',
            HobbitMMUI.ShowMenu(
                menu='MAIN'
            ),
            transitions={'succeeded': 'succeeded',
                         'failed': 'succeeded',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'LOCATE_USER_2',
            locate_user.get_detect_user(),
            transitions={'succeeded': 'SAD',
                         'aborted': 'succeeded',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'SAD',
            speech_output.emo_say_something(
                emo='SAD',
                time=4,
                text='T_BM_SORRY_NOT_ABLE_TO_FIND_OBJECT_O'
            ),
            transitions={'succeeded': 'COUNTER',
                         'aborted': 'succeeded',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'COUNTER',
            Counter(),
            transitions={'succeeded': 'CHECK_ROLE',
                         'aborted': 'LOG_NOT_COMPLETE',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}

        )
        StateMachine.add(
            'CHECK_ROLE',
            social_role.GetSocialRole(),
            transitions={'companion': 'NEUTRAL',
                         'butler': 'LOG_NOT_COMPLETE',
                         'tool': 'LOG_NOT_COMPLETE',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'NEUTRAL',
            speech_output.emo_yes_no_something(
                emo='NEUTRAL',
                time=0,
                text='T_BM_WOULD_YOU_HELP_ME_FIND_OBJECT_O'
            ),
            transitions={'yes': 'VHAPPY_SAY_3',
                         'no': 'LOG_NO_HELP',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'VHAPPY_SAY_3',
            speech_output.emo_say_something(
                emo='VERY_HAPPY',
                time=4,
                text='T_BM_THANKS_WHERE_COULD_THE_OBJECT_BE'
            ),
            transitions={'succeeded': 'SAY_SELECT_ROOM',
                         'aborted': 'succeeded',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'SAY_SELECT_ROOM',
            speech_output.sayText(
                info='T_GT_SelectRoom'
            ),
            transitions={'succeeded': 'HAPPY_SAY_3',
                         'failed': 'succeeded',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'HAPPY_SAY_3',
            speech_output.emo_say_something(
                emo='HAPPY',
                time=4,
                text='T_BM_I_WILL_LOOK_AT_PLACE_P_FOR_OBJECT_O'
            ),
            transitions={'succeeded': 'SAY_SELECT_ROOM',
                         'aborted': 'LOG_NOT_COMPLETE',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'MOVE_TO_P',
            hobbit_move.goToPosition(
                frame='/map',
                room='corridor',
                place='search'
            ),
            transitions={'succeeded': 'SEARCH_FOR_OBJECT',
                         'aborted': 'LOG_NOT_COMPLETE',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'LOG_NO_HELP',
            logging.DoLog(
                scenario='bring object',
                data='Help was not accepted'
            ),
            transitions={'succeeded': 'SAD_2',
                         'aborted': 'SAD_2',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'SAD_2',
            HobbitEmotions.ShowEmotions(
                emotion='SAD',
                emo_time=4
            ),
            transitions={'succeeded': 'LOG_NOT_COMPLETE',
                         'failed': 'LOG_NOT_COMPLETE',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'LOG_NOT_COMPLETE',
            logging.DoLog(
                scenario='bring object',
                data='Task is not accomplished.'
            ),
            transitions={'succeeded': 'LOG_TASK_HAS_ENDED',
                         'aborted': 'LOG_TASK_HAS_ENDED',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'LOG_TASK_HAS_ENDED',
            logging.DoLog(
                scenario='bring object',
                data='Task has ended.'
            ),
            transitions={'succeeded': 'MAIN_MENU',
                         'aborted': 'aborted',
                         'preempted': 'LOG_TASK_HAS_PREEMPTED'}
        )
        StateMachine.add(
            'LOG_TASK_HAS_PREEMPTED',
            logging.DoLog(
                scenario='bring object',
                data='Task has preempted.'
            ),
            transitions={'succeeded': 'MAIN_MENU',
                         'aborted': 'aborted',
                         'preempted': 'preempted'}
        )

    return sm

if __name__ == '__main__':
    print("Do not call this directly. Import it into your node.")

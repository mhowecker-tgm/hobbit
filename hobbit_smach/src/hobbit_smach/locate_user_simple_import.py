#!/usr/bin/python
# -*- coding: utf-8 -*-

PREEMPT_TIMEOUT = 5
SERVER_TIMEOUT = 5

import rospy
import uashh_smach.util as util
import tf
import math

from hobbit_msgs.srv import SwitchVision, SwitchVisionRequest
from std_msgs.msg import String
from smach_ros import ServiceState, SimpleActionState
from smach import State, StateMachine, Concurrence, Iterator
from mira_msgs.srv import UserNavMode, ObsNavMode
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, Point, Quaternion
# from hobbit_smach.bcolors import bcolors
from rgbd_acquisition.msg import Person
from hobbit_msgs.srv import GetRooms
import hobbit_smach.hobbit_move_import as hobbit_move
import uashh_smach.platform.move_base as move_base
import hobbit_smach.head_move_import as head_move
from uashh_smach.util import SleepState, WaitForMsgState, WaitAndCheckMsgState
from hobbit_msgs.srv import SetCloserStateRequest
from hobbit_msgs.srv import SwitchVision, SwitchVisionRequest, SetCloserState
from hobbit_msgs.msg import GeneralHobbitAction, GeneralHobbitGoal, Event
import hobbit_smach.logging_import as log
from hobbit_user_interaction import HobbitMMUI
import hobbit_smach.sos_call_import as sos_call
first = True


def gesture_sm():

    
    sm = StateMachine(outcomes = ['succeeded','aborted','preempted'])

    with sm:
        counter_it = Iterator(outcomes = ['succeeded', 'preempted', 'aborted'],
                              input_keys = [],
                              output_keys = [],
                              it = lambda: range(0, 3),
                              it_label = 'index',
                              exhausted_outcome = 'succeeded')

        with counter_it:
            def child_term_cb(outcome_map):
                rospy.loginfo('cc1: child_term_cb: ')
                rospy.loginfo(str(outcome_map))
                return True
            def out_cb(outcome_map):
                rospy.loginfo(str(outcome_map))
                if outcome_map['YES_NO'] == 'yes':
                    return 'succeeded'
                elif outcome_map['TOPIC'] == 'succeeded':
                    return 'succeeded'
                elif outcome_map['TOPIC'] == 'aborted':
                    return 'aborted'
                elif outcome_map['YES_NO'] in ['no', 'timeout', '3times', 'failed']:
                    return 'aborted'
                else:
                    return 'preempted'


            container_sm = StateMachine(outcomes = ['succeeded','aborted','preempted'])
        
            cc1 = Concurrence(outcomes=['aborted', 'succeeded', 'preempted'],
                             default_outcome='aborted',
                             child_termination_cb=child_term_cb,
                             outcome_cb=out_cb
            )
            with container_sm:
                def g_come_cb(event_message, ud):
                    if event_message.event.upper() in ['G_COME', 'A_CLOSER']:
                         rospy.loginfo('Was G_COME or A_CLOSER! Close MMUI Prompt!')
                         mmui = MMUI.MMUIInterface()
                         mmui.remove_last_prompt()
                         return 'succeeded'

                StateMachine.add(
                    'WAIT_FOR_CLOSER',
                    cc1,
                    transitions={'succeeded': 'MOVE',
                                 'preempted': 'preempted',
                                 'aborted': 'aborted'}
                )
                StateMachine.add(
                    'MOVE',
                    hobbit_move.move_discrete(in_motion='Move', in_value=0.1),
                    transitions={'succeeded': 'MOVED_BACK',
                                 'preempted': 'preempted',
                                 'aborted': 'aborted'}
                )
                StateMachine.add(
                    'MOVED_BACK',
                    ServiceState(
                        '/came_closer/set_closer_state',
                        SetCloserState,
                        request=SetCloserStateRequest(state=True),
                    ),
                    transitions={'succeeded': 'succeeded',
                                 'preempted': 'preempted',}
                )

            with cc1:
                Concurrence.add(
                    'YES_NO',
                    HobbitMMUI.AskYesNo(question='T_CLOSER_QUESTION'),
                )
                Concurrence.add(
                    'TOPIC',
                    WaitAndCheckMsgState(
                        '/Event',
                        Event,
                        msg_cb=g_come_cb,
                        timeout=45
                    )
                )

            Iterator.set_contained_state('CONTAINER_STATE',
                                         container_sm,
                                         loop_outcomes = ['succeeded']
                                        )

        StateMachine.add('COUNT_GESTURES', counter_it,
                        {'succeeded':'succeeded',
                         'preempted':'preempted',
                         'aborted':'aborted'})
    return sm

def switch_vision_cb(ud, response):
    if response.result:

        return 'succeeded'
    else:
        return 'aborted'

def closer_cb(ud, goal):
    params=[]
    params.append(String(str(ud.person_z/1000)))
    params.append(String(str(-ud.person_x/1000)))
    goal = GeneralHobbitGoal(
        command=String('start'),
        parameters=params
    )
    return goal

def detectUser():
    sm = StateMachine(
        outcomes=['succeeded', 'preempted', 'failed']
    )
    with sm:
        StateMachine.add(
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
        StateMachine.add(
            'COUNTER',
            UserCounter(),
            transitions={'succeeded': 'DETECTION',
                         'aborted': 'failed',
                         'preempted': 'preempted'}
        )
        return sm

def msg_timer_sm():
    sm = StateMachine(outcomes = ['succeeded','aborted','preempted'],
                      output_keys=['person_x', 'person_z'])

    def child_term_cb(outcome_map):
        rospy.loginfo(str(outcome_map))
        return True

    def person_cb(msg, ud):
        rospy.loginfo(str(msg))
        if (rospy.Time.now() - msg.stamp) > rospy.Duration(1):
            rospy.loginfo('Person data is too old. Ignore.')
            return False
        if msg.source == 6:
            rospy.loginfo('Do not use this data')
            return False
        ud.person_x = msg.x
        ud.person_z = msg.z
        rospy.loginfo("OK. Use it.")
        return True

    cc = Concurrence(outcomes=['aborted', 'succeeded', 'preempted'],
                     default_outcome='aborted',
                     child_termination_cb=child_term_cb,
                     output_keys=['person_x', 'person_z'],
                     outcome_map={'succeeded': {'LISTENER': 'succeeded'},
                                  'aborted': {'TIMER': 'succeeded',
                                              'LISTENER': 'preempted'}})
    with sm:
        StateMachine.add(
            'GET_PERSON',
            WaitForMsgState(
                '/persons',
                Person,
                msg_cb=person_cb,
                timeout=3,
                output_keys=['user_pose', 'person_z', 'person_x']
            ),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'GET_PERSON'}
        )
    with cc:
        Concurrence.add('LISTENER', sm)
        Concurrence.add('TIMER', SleepState(duration=3))
    return cc

class UserCounter(State):
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'])
        self.counter = 0
        self.limit = 10

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print('Counter: %d' % self.counter)
        self.counter += 1
        if self.counter < self.limit:
            return 'succeeded'
        else:
            self.counter = 0
            return 'aborted'


class Init(State):
    """
    Class to initialize certain parameters
    """

    def __init__(self):
        State.__init__(
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
        try:
            if ud.command.data == 'cancel':
                return 'canceled'
        except KeyError:
            pass
        return 'succeeded'


class CleanUp(State):
    """
    Class for setting the result message and clean up persistent variables
    """

    def __init__(self):
        State.__init__(
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


class SetSuccess(State):
    """
    Class for setting the success message in the actionlib result and clean
    up of persistent variables
    """

    def __init__(self):
        State.__init__(
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


class CleanPositions(State):
    """
    Class for removing unneeded positions from the rooms.
    Only use the default 'user search' positions.
    Remove the waiting, 'object search' and recharge positions
    """

    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            input_keys=['response', 'positions'],
            output_keys=['positions', 'plan'])

    def execute(self, ud):
        global first
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.positions = []
        for room in ud.response.rooms.rooms_vector:
            for position in room.places_vector:
                if 'default' in position.place_type:
                    ud.positions.append(
                        {'x': position.x,
                         'y': position.y,
                         'theta': position.theta,
                         'distance': 'None',
                         'room': room.room_name,
                         'place_name': position.place_name})
        ud.plan = None
        rospy.loginfo('CleanPositions: ' + str(len(ud.positions)))
        first = True
        return 'succeeded'


class PlanPath(State):
    """
    Class to determine the shortest path to all possible positions,
    starting in the users last known room
    """
    def __init__(self):
        State.__init__(
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
        global first
        # The next line of code is dangerous
        # This will disable the obstacle detection
        # self.pub_obstacle.publish('inactive')
        if self.preempt_requested():
            print 'Preempt requested:'
            self.service_preempt()
            self.getPlan.close()
            return 'preempted'
        if self.first or first:
            self.positions = ud.positions
            self.first = False
            first = False

        robot_pose = get_pose_from_xytheta(ud.x, ud.y, ud.yaw)
        if len(self.positions) == 0:
            self.reset(ud)
            self.first = True
            first = True
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
            except rospy.ServiceException:
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

def rotate_and_detect():
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        output_keys=['person_z', 'person_x'],
        input_keys=['person_z', 'person_x']
    )
    with sm:
        StateMachine.add(
            'USER_DETECTION',
            msg_timer_sm(),
            transitions={'succeeded': 'STOP_ROTATION',
                         'preempted': 'preempted',
                         'aborted': 'ROTATE'}
        )
        StateMachine.add(
            'ROTATE',
            hobbit_move.MoveDiscrete(motion='Turn', value='45'),
            transitions={'succeeded': 'aborted',
                         'preempted': 'preempted',
                         'aborted': 'USER_DETECTION'}
        )
        StateMachine.add(
            'STOP_ROTATION',
            hobbit_move.Stop(),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'preempted',
                         'aborted': 'aborted'}
        )
    it = Iterator(
        outcomes = ['succeeded', 'preempted', 'aborted'],
        input_keys = ['person_x', 'person_z'],
        output_keys = ['person_x', 'person_z'],
        it = lambda: range(0, 8),
        it_label = 'index',
        exhausted_outcome = 'aborted'
    )
    with it:
        Iterator.set_contained_state(
            'CONTAINER_SM',
            sm,
            loop_outcomes = ['aborted']
        )
    return it


class Counter(State):
    def __init__(self):
        State.__init__(
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


def userdetection_cb(msg, ud):
    # print bcolors.OKGREEN + 'received message: '
    # print msg
    # print bcolors.ENDC
    if 0.39 < msg.confidence <= 0.61:
        # print bcolors.OKGREEN + 'Face detected!' + bcolors.ENDC
        rospy.loginfo('Face detected!')
        return True
    elif msg.confidence > 0.61:
        # print bcolors.OKGREEN + 'Skeleton detected!' + bcolors.ENDC
        rospy.loginfo('Skeleton detected!')
        return True
    else:
        rospy.loginfo('NO USER')
        # print bcolors.FAIL + 'NO USER' + bcolors.ENDC
        return False


def get_detect_user():
    sm = StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command'],
        output_keys=['result'])

    sm.userdata.result = String('started')
    sm.userdata.detection = False

    with sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'GET_ALL_POSITIONS',
                         'canceled': 'LOG_ABORT'}
        )
        StateMachine.add(
            'GET_ALL_POSITIONS',
            ServiceState('getRooms',
                         GetRooms,
                         response_key='response'),
            transitions={'succeeded': 'GET_ROBOT_POSE',
                         'aborted': 'LOG_ABORT'}
        )
        StateMachine.add(
            'GET_ROBOT_POSE',
            move_base.ReadRobotPositionState(),
            transitions={'succeeded': 'CLEAN_POSITIONS'}
        )
        StateMachine.add(
            'CLEAN_POSITIONS',
            CleanPositions(),
            transitions={'succeeded': 'PLAN_PATH'}
        )
        StateMachine.add(
            'PLAN_PATH',
            PlanPath(),
            transitions={'succeeded': 'MOVE_BASE',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORT'}
        )
        StateMachine.add(
            'MOVE_BASE',
            hobbit_move.goToPose(),
            transitions={'succeeded': 'SWITCH_VISION',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORT'},
            remapping={'x': 'goal_position_x',
                       'y': 'goal_position_y',
                       'yaw': 'goal_position_yaw'}
        )
        StateMachine.add_auto(
            'SWITCH_VISION',
            ServiceState(
                '/vision_system/locateUser',
                SwitchVision,
                request=SwitchVisionRequest(dummyInput=True),
                response_cb=switch_vision_cb
            ),
            connector_outcomes=['succeeded', 'preempted', 'aborted']
        )
        StateMachine.add(
            'PREPARE_STOP',
            ServiceState(
                '/user_nav_mode',
                UserNavMode
            ),
            transitions={'succeeded': 'MOVE_HEAD_UP',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'MOVE_HEAD_UP'}
        )
        StateMachine.add(
            'MOVE_HEAD_UP',
            head_move.MoveTo(pose='center_center'),
            transitions={'succeeded': 'WAIT',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'WAIT'}
        )
        StateMachine.add(
            'WAIT',
            util.SleepState(duration=0.1),
            transitions={'succeeded': 'ROTATE_AND_DETECT'}
        )
        StateMachine.add(
            'ROTATE_AND_DETECT',
            rotate_and_detect(),
            transitions={'succeeded': 'CLOSER',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'PREPARE_MOVEMENT'}
        )
        StateMachine.add(
            'PREPARE_MOVEMENT',
            ServiceState(
                '/obs_nav_mode',
                ObsNavMode
            ),
            transitions={'succeeded': 'PLAN_PATH',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'PLAN_PATH'}
        )
        StateMachine.add(
            'CLOSER',
            SimpleActionState(
                'come_closer',
                GeneralHobbitAction,
                goal_cb=closer_cb,
                input_keys=['person_z', 'person_x'],
                preempt_timeout=rospy.Duration(PREEMPT_TIMEOUT),
                server_wait_timeout=rospy.Duration(SERVER_TIMEOUT)
            ),
            transitions={'succeeded': 'MOVED',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'GESTURE_HANDLING'}
        )
        StateMachine.add(
            'MOVED',
            ServiceState(
                '/came_closer/set_closer_state',
                SetCloserState,
                request=SetCloserStateRequest(state=True),
            ),
            transitions={'succeeded': 'GESTURE_HANDLING',
                         'preempted': 'LOG_PREEMPT'}
        )
        StateMachine.add(
            'GESTURE_HANDLING',
            gesture_sm(),
            transitions={'succeeded': 'CALL_FOR_THE_USER',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORT'}
        )
        StateMachine.add(
            'CALL_FOR_THE_USER',
            call_for_the_user(),
            transitions={'succeeded': 'LOG_SUCCESS',
                         'preempted': 'LOG_PREEMPT',
                         'aborted': 'LOG_ABORT'}
        )
        StateMachine.add(
            'CLEAN_UP',
            CleanUp(),
            transitions={'succeeded': 'LOG_ABORT'}
        )
        StateMachine.add(
            'LOG_PREEMPT',
            log.DoLogPreempt(scenario='Locate User'),
            transitions={'succeeded': 'preempted'}
        )
        StateMachine.add(
            'LOG_ABORT',
            log.DoLogAborted(scenario='Locate User'),
            transitions={'succeeded': 'aborted'}
        )
        StateMachine.add(
            'LOG_SUCCESS',
            log.DoLogSuccess(scenario='Locate User'),
            transitions={'succeeded': 'succeeded'}
        )
    return sm

def call_for_the_user():
    sm = StateMachine(outcomes = ['succeeded','aborted','preempted'])
    first_loop = StateMachine(outcomes = ['succeeded','aborted','preempted'])
    second_loop = StateMachine(outcomes = ['succeeded','aborted','preempted'])
    first_it = Iterator(outcomes = ['succeeded', 'preempted', 'aborted'],
                              input_keys = [],
                              output_keys = [],
                              it = lambda: range(0, 3),
                              it_label = 'index',
                              exhausted_outcome = 'succeeded')
    second_it = Iterator(outcomes = ['succeeded', 'preempted', 'aborted'],
                              input_keys = [],
                              output_keys = [],
                              it = lambda: range(0, 3),
                              it_label = 'index',
                              exhausted_outcome = 'succeeded')

    def child_term_cb(outcome_map):
        rospy.loginfo('cc1: child_term_cb: ')
        rospy.loginfo(str(outcome_map))
        return True

    def out_cb(outcome_map):
        rospy.loginfo(str(outcome_map))
        if outcome_map['YES_NO'] == 'yes':
            return 'succeeded'
        elif outcome_map['WAIT'] == 'succeeded':
            return 'succeeded'
        elif outcome_map['WAIT'] == 'aborted':
            return 'aborted'
        elif outcome_map['YES_NO'] in ['no', 'timeout', '3times', 'failed']:
            return 'aborted'
        else:
            return 'preempted'

    cc1 = Concurrence(outcomes=['aborted', 'succeeded', 'preempted'],
                             default_outcome='aborted',
                             child_termination_cb=child_term_cb,
                             outcome_cb=out_cb
    )

    with first_it:
        Iterator.set_contained_state(
            'FIRST_SM',
            first_loop,
            loop_outcomes = ['succeeded']
        )

    with cc1:
        Concurrence.add(
            'YES_NO',
            HobbitMMUI.AskYesNo(question='T_HM_CallUser'),
            )
        Concurrence.add(
            'WAIT',
            util.SleepState(duration=20)
        )

    with first_loop:
        StateMachine.add(
            'SET_VOLUME',
            HobbitMMUI.SetVolumeHigher(),
            transitions={'succeeded': 'CALL_USER',
                         'aborted': 'CALL_USER',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'CALL_USER',
            cc1,
            transitions={'succeeded': 'WAIT_15SEC',
                         'aborted': 'WAIT_15SEC',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'WAIT_15SEC',
            util.SleepState(duration=15)
        )

    with second_loop:
        StateMachine.add(
            'CALL_USER',
            cc1,
            transitions={'succeeded': 'WAIT_15SEC',
                         'aborted': 'WAIT_15SEC',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'WAIT_15SEC',
            util.SleepState(duration=15)
        )

    with second_it:
        Iterator.set_contained_state(
            'SECOND_SM',
            second_loop,
            loop_outcomes = ['succeeded']
        )
    with sm:
        StateMachine.add(
            'CALL_FOR_THE_USER',
            first_it,
            transitions={'succeeded':'RESET_VOLUME',
                         'preempted':'preempted',
                         'aborted':'aborted'}
        )
        StateMachine.add(
            'SET_VOLUME',
            HobbitMMUI.SetAbsVolume(volume='100'),
            transitions={'succeeded':'CALL_FOR_THE_USER_LOUDER',
                         'preempted':'preempted',
                         'aborted':'aborted'}
        )
        StateMachine.add(
            'CALL_FOR_THE_USER_LOUDER',
            second_it,
            transitions={'succeeded':'RESET_VOLUME',
                         'preempted':'preempted',
                         'aborted':'aborted'}
        )
        StateMachine.add(
            'RESET_VOLUME',
            HobbitMMUI.SetAbsVolume(volume='30'),
            transitions={'succeeded':'succeeded',
                         'preempted':'preempted',
                         'aborted':'aborted'}
        )
        StateMachine.add(
            'EMERGENCY_CALL',
            sos_call.get_call_sos_simple(),
            transitions={'succeeded': 'succeeded',
                         'failed': 'aborted',
                         'preempted': 'preempted',
                         'aborted': 'aborted'}
        )
    return sm

if __name__ == '__main__':
    print("Do not call this directly. Import it into your node.")

#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'end_user_interaction'

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import uashh_smach.util as util
#from rospy.service import ServiceException

from std_msgs.msg import String
from hobbit_msgs.msg import EndUserInteractionAction
from hobbit_msgs.srv import GetName
from smach_ros import ActionServerWrapper, ServiceState
from hobbit_user_interaction import HobbitMMUI
import datetime
import random


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
        smach.State.__init__(self, outcomes=['succeeded', 'canceled'],
                             input_keys=['command'])

    def execute(self, ud):
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        # TODO:
        #depending on the social_role the correct text should be queried
        ud.question = 'Can I do something else for you?'
        return 'succeeded'


class ChangeSocialRole(smach.State):
    """Class to interact with the MMUI"""
    def __init__(self, change=0):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'preempted'])
        self.change = change

    def execute(self, ud):
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        if abs(self.change) == 1:
            old = rospy.get_param('social_role')
            if abs(old) <= 2:
                rospy.set_param('social_role', old + self.change)
            else:
                rospy.loginfo('Already at the highest or lowest social role')
        else:
            return 'failed'
        return 'succeeded'


class AskForCmd(smach.State):
    """Class to interact with the MMUI"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, ud):
        return 'succeeded'


class select_end_pose(smach.State):
    """Class to choose robots end_pose depending on the social role"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['end_pose'])
        self.social_role = 2

    def execute(self, ud):
        return 'succeeded'


class TimeDiffCheck(smach.State):
    """
    check the days between the last time we asked the user about the
    social role and today
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['yes', 'no', 'failed', 'preempted'])
        self.last = 0
        random.seed()

    def execute(self, ud):
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        if rospy.has_param('social_role_check_day'):
            self.last = rospy.get_param('social_role_check_day')
        today = datetime.datetime.today().weekday()
        limit = random.randint(3, 4)
        if abs(today - self.last) > limit:
            return 'yes'
        else:
            return 'no'
        return 'failed'


class process_cmd(smach.State):
    """Class to process user response and extract command from it"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['end_pose'])
        self.social_role = 2

    def execute(self, ud):
        return 'succeeded'


class CleanUp(smach.State):
    """Class for setting the result message and clean up persistent
    variables"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['command', 'visited_places'],
                             output_keys=['result', 'command',
                                          'visited_places'])
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
        ud.visited_places = []
        ud.result = String('user not detected')
        return 'succeeded'


class SetSuccess(smach.State):
    """Class for setting the success message in the actionlib result and clean
    up of persistent variables"""
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
    """Class for removing unneeded positions from the rooms. Only use the
    default 'user search' positions. \
            Remove the waiting, 'object search' and recharge positions"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['response', 'positions'],
                             output_keys=['positions', 'plan'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.positions = []
        for room in ud.response.rooms.rooms_vector:
            for position in room.places_vector:
                # Search position is the position the robot should be able to
                #rotate 360 degree without hitting any objects (e.g. walls)
                if 'default' in position.place_name:
                    ud.positions.append({'x': position.x, 'y': position.y,
                                         'theta': position.theta,
                                         'room': room.room_name,
                                         'distance': 'None',
                                         'place_name': position.place_name,
                                         'penalty': 1})
        ud.plan = None
        return 'succeeded'


class MoveBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted',
                                             'failure'],
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
        print bcolors.OKGREEN + 'Moving to %s in %s'\
            % (ud.robot_end_pose['place'], ud.robot_end_pose['room'])\
            + bcolors.ENDC
        self.pub_room.publish(String(ud.robot_end_pose['room']))
        rospy.sleep(1.0)
        self.pub_place.publish(String(ud.robot_end_pose['place']))
        try:
            ud.visited_places.append(ud.robot_end_pose)
        except:
            ud.visited_places = []
            ud.visited_places.append(ud.robot_end_pose)
        return 'succeeded'


def goal_reached_cb(msg, ud):
    print bcolors.WARNING + 'received message: /goal_status'
    + bcolors.ENDC
    #print msg.data
    if (msg.data == 'reached') or (msg.data == 'idle'):
        print bcolors.OKGREEN + 'position reached. Start rotation.'
        + bcolors.ENDC
        #rospy.sleep(2.0)
        return True
    else:
        return False


def main():
    rospy.init_node(NAME)

    eui_sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command'],
        output_keys=['result'])

    eui_sm.userdata.result = String('started')
    eui_sm.userdata.detection = False

    with eui_sm:
        smach.StateMachine.add('INIT', Init(),
                               transitions={'succeeded': 'CHECK_FOR_MUC',
                                            'canceled': 'CLEAN_UP'})
        smach.StateMachine.add('CHECK_FOR_MUC',
                               TimeDiffCheck(),
                               transitions={'yes': 'ASK_ABOUT_SOCIAL_ROLE',
                                            'no': 'MMUI_ASK_YES_NO',
                                            'failed': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add(
            'ASK_ABOUT_SOCIAL_ROLE',
            HobbitMMUI.AskYesNo(question='Am i too obstrusive in general?'),
            transitions={'yes': 'SOCIAL_ROLE_DOWN',
                         'no': 'SOCIAL_ROLE_UP'})
        smach.StateMachine.add(
            'SOCIAL_ROLE_UP',
            ChangeSocialRole(change=1),
            transitions={'succeeded': 'MMUI_ASK_YES_NO',
                         'failed': 'aborted',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'SOCIAL_ROLE_DOWN',
            ChangeSocialRole(change=-1),
            transitions={'succeeded': 'MMUI_ASK_YES_NO',
                         'failed': 'aborted',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'MMUI_ASK_YES_NO',
            HobbitMMUI.AskYesNo(question='Can I do something else for you?'),
            transitions={'yes': 'MMUI_ASK_FOR_CMD',
                         'no': 'GET_CURRENT_ROOM'})
        smach.StateMachine.add('GET_CURRENT_ROOM',
                               ServiceState('get_robots_current_room', GetName,
                                            response_key='robots_room_name'),
                               transitions={'succeeded': 'SELECT_END_POSE'})
        smach.StateMachine.add('SELECT_END_POSE', select_end_pose(),
                               transitions={'succeeded': 'MOVE_BASE_GO',
                                            'failed': 'CLEAN_UP'})
        smach.StateMachine.add('MOVE_BASE_GO', MoveBase(),
                               transitions={'succeeded': 'LOCATION_REACHED',
                                            'preempted': 'CLEAN_UP',
                                            'failure': 'CLEAN_UP'})
        smach.StateMachine.add('LOCATION_REACHED',
                               util.WaitForMsgState('/goal_status', String,
                                                    goal_reached_cb,
                                                    timeout=30),
                               transitions={'aborted': 'LOCATION_REACHED',
                                            'succeeded': 'SET_SUCCESS',
                                            'preempted': 'CLEAN_UP'})
        smach.StateMachine.add('MMUI_ASK_FOR_CMD', HobbitMMUI.AskForCmd(),
                               transitions={'succeeded': 'PROCESS_CMD',
                                            'failed': 'CLEAN_UP'})
        smach.StateMachine.add('PROCESS_CMD', process_cmd(),
                               transitions={'succeeded': 'SET_SUCCESS',
                                            'failed': 'CLEAN_UP'})
        smach.StateMachine.add('SET_SUCCESS', SetSuccess(),
                               transitions={'succeeded': 'succeeded',
                                            'preempted': 'CLEAN_UP'})
        smach.StateMachine.add('CLEAN_UP', CleanUp(),
                               transitions={'succeeded': 'preempted'})

    asw = ActionServerWrapper('end_user_interaction',
                              EndUserInteractionAction,
                              eui_sm, ['succeeded'], ['aborted'],
                              ['preempted'],
                              result_slots_map={'result': 'result'},
                              goal_slots_map={'command': 'command'})

    sis = smach_ros.IntrospectionServer('smach_server', eui_sm,
                                        '/HOBBIT/EUI_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

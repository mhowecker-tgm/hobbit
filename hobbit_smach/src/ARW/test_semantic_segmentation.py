#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_msgs'
NAME = 'ARW_BringObject'

FILE_ALL_POSES = "/home/hobbit/ACIN/DEBUG_get_search_positions.log"
FILE_FILTERED_POSES = "/home/hobbit/ACIN/DEBUG_get_filtered_positions.log"

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import uashh_smach.util as util
import tf
from rospy.service import ServiceException

from std_msgs.msg import String
from hobbit_msgs.msg import LocateUserAction
from hobbit_msgs.srv import *
from hobbit_msgs.msg import *
from smach_ros import ServiceState
from actionlib import SimpleActionServer
#from navfn.srv import MakeNavPlan, MakeNavPlanRequest
from geometry_msgs.msg import Pose2D, PoseStamped, Point, \
    Quaternion, Vector3Stamped, PoseWithCovarianceStamped, Pose, PoseArray
from sensor_msgs.msg import PointCloud2
from semantic_segmentation.srv import get_search_positions, \
    get_search_positionsRequest


def point_cloud_cb(msg, ud):
    ud.cloud = msg
    return True


class Check(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['finished', 'progress'],
            input_keys=['rotation_counter', 'search_poses'],
            output_keys=['rotation_counter'])
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)
        self.pub_poses = rospy.Publisher('/all_search_poses',
                                         PoseArray, latch=True)
        self.poses = PoseArray()
        self.poses.poses = []

    def execute(self, ud):
        print('length of ud.search_poses: ' + str(len(ud.search_poses)))
        for position in ud.search_poses:
            # poses are only generated and published to show them in rviz
            self.poses.header.frame_id = 'map'
            pose = Pose()
            pose.position = Point(position.x, position.y, 0.0)
            pose.orientation = Quaternion(
                *tf.transformations.quaternion_from_euler(
                    0, 0, position.theta))
            self.poses.poses.append(pose)
            self.pub_poses.publish(self.poses)
        #pose = Pose()
        #pose.position = Point(0.0, 0.0, 0.0)
        #pose.orientation = Quaternion(
        #    *tf.transformations.quaternion_from_euler(
        #        0, 0, 0))
        #self.poses.poses.append(pose)
        #pose1 = Pose()
        #pose1.position = Point(0.0, 0.0, 0.0)
        #pose1.orientation = Quaternion(
        #    *tf.transformations.quaternion_from_euler(
        #        0, 0, 1.57))
        #self.poses.poses.append(pose1)
        #self.pub_poses.publish(self.poses)
        print('length of ud.search_poses: ' + str(len(self.poses.poses)))
        return 'finished'


def get_search_positions_cb(ud, response):
    if not response:
        print(bcolors.FAIL + 'NO POSE RECEIEVED' + bcolors.ENDC)
        return 'aborted'
    else:
        print(bcolors.FAIL + 'POSE RECEIEVED' + bcolors.ENDC)
        for pose in response.positions:
            print(type(pose))
            print(bcolors.OKBLUE + str(pose) + bcolors.ENDC)
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
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'canceled'],
            input_keys=['command'], output_keys=['social_role'])
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)
        self.pub_face.publish('EMO_NEUTRAL')

    def execute(self, ud):
        self.pub_face.publish('EMO_NEUTRAL')
        self.pub_head.publish('middle')
        self.pub_obstacle.publish('active')
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        if ud.command.data == 'cancel':
            return 'canceled'
        return 'succeeded'


class PlanPath2(smach.State):
    """
    Class to get the next pose from search_poses
    and converts it to the needed format
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['movement', 'preempted', 'failure'],
            input_keys=['robot_current_pose',
                        'robot_end_pose',
                        'pose',
                        'search_poses',
                        'verified_poses'],
            output_keys=['detection', 'robot_end_pose'])
        self.pub_search_poses = rospy.Publisher('/search_poses',
                                                PoseArray, latch=True)
        self.search_poses = PoseArray()
        self.search_poses.poses = []

    def execute(self, ud):
        if self.preempt_requested():
            print 'Preempt requested: '
            self.service_preempt()
            return 'preempted'
        if len(ud.verified_poses) == 0:
            print(bcolors.OKGREEN +
                  'All poses in this room have been visited. \
                  Moving to the next'
                  + bcolors.ENDC)
            return 'failure'

        for position in ud.verified_poses:
            print(bcolors.FAIL + 'VISITED_PLACES + position' + bcolors.ENDC)
            print position
            end_pose = Pose2DStamped()
            end_pose.x = position.x
            end_pose.y = position.y
            end_pose.theta = position.theta
            ud.robot_end_pose = end_pose
            print('length of ud.verified_poses: '
                  + str(len(ud.verified_poses)))
            ud.verified_poses.remove(position)
            print('length of ud.verified_poses: '
                  + str(len(ud.verified_poses)))
            # search_poses are only generated
            # and published to show them in rviz
            self.search_poses.header.frame_id = 'map'
            search_pose = Pose()
            search_pose.position = Point(end_pose.x, end_pose.y, 0.0)
            search_pose.orientation = Quaternion(
                *tf.transformations.quaternion_from_euler(
                    0, 0, end_pose.theta))
            self.search_poses.poses.append(search_pose)
            self.pub_search_poses.publish(self.search_poses)
        rospy.sleep(2.0)
        return 'movement'


class MoveBase2(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'failure'],
            input_keys=['robot_end_pose'],
            output_keys=['robot_end_pose'])
        self.pub_goal = rospy.Publisher('goal_pose', Pose2DStamped)
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)

    def execute(self, ud):
        self.pub_head.publish(String('middle'))
        rospy.sleep(1.0)
        #goal = Pose2DStamped()
        #goal.x = ud.robot_end_pose.x
        #goal.y = ud.robot_end_pose.y
        #goal.theta = ud.robot_end_pose.theta
        self.pub_goal.publish(ud.robot_end_pose)
        rospy.sleep(2.0)
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
        print bcolors.FAIL + 'no robot pose message received.' + bcolors.ENDC
        return False


def scene_cb(msg, ud):
    ud.scene = msg
    return True


def main():
    rospy.init_node(NAME)

    lu_sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['command', 'search_poses'],
        output_keys=['result'])

    lu_sm.userdata.result = String('started')
    lu_sm.userdata.detection = False

    with lu_sm:

        lu_sm.userdata.result = String('started')
        lu_sm.userdata.rotation_counter = 0
        lu_sm.userdata.search_poses = []

        smach.StateMachine.add(
            'INIT',
            Init(),
            transitions={
                'succeeded': 'GET_SCENE',
                'canceled': 'aborted'
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
            transitions={'succeeded': 'GET_ROBOT_POSE',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'GET_ROBOT_POSE', util.WaitForMsgState(
                '/amcl_pose',
                PoseWithCovarianceStamped,
                get_robot_pose_cb,
                output_keys=['robot_current_pose'],
                timeout=5
            ),
            transitions={'succeeded': 'GET_SEARCH_POSITIONS',
                         'aborted': 'GET_ROBOT_POSE',
                         'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'GET_SEARCH_POSITIONS',
            ServiceState(
                'get_search_positions',
                get_search_positions,
                request_cb=search_poses_cb,
                response_cb=get_search_positions_cb,
                input_keys=['scene',
                            'robot_current_pose',
                            'positions',
                            'search_poses'],
                output_keys=['positions', 'search_poses']
            ),
            transitions={'succeeded': '360_CHECK',
                         'aborted': 'aborted'}
        )
        smach.StateMachine.add(
            '360_CHECK',
            Check(),
            transitions={'finished': 'succeeded',
                         'progress': 'aborted'}
        )
        smach.StateMachine.add(
            'PLAN_PATH_2',
            PlanPath2(),
            transitions={'movement': 'succeeded',
                         'preempted': 'aborted',
                         'failure': 'aborted'})

    asw = smach_ros.ActionServerWrapper(
        'ARW',
        LocateUserAction,
        lu_sm,
        ['succeeded'], ['aborted'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command'})

    sis = smach_ros.IntrospectionServer(
        'smach_server',
        lu_sm,
        '/HOBBIT/ARW_BO_SM_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

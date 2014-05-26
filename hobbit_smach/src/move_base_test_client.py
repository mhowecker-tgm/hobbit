#!/usr/bin/env python

import roslib
roslib.load_manifest('hobbit_smach')
import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from interfaces_mira.msg import MiraSendingGoalsAction, MiraSendingGoalsGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion


def test_action():
    global pose
    print('Trying to connect to action server')
    #print(pose)
    #client = actionlib.SimpleActionClient('mira_sending_goals', MoveBaseAction)
    client = actionlib.SimpleActionClient('mira_sending_goals', MiraSendingGoalsAction)
    client.wait_for_server()
    print('Connected to mira_sending_goals Action server')
    print(pose)
    goal_pose = PoseStamped()
    goal_pose.pose = pose
    #goal_pose.pose.position.x -=0.5
    goal_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,3))
    goal = MoveBaseGoal(target_pose = goal_pose)
    print('Sending goal:' + str(goal))
    client.send_goal(goal)
    client.wait_for_result()
    return(client.get_result())


def callback(msg):
    global subscriber
    global pose
    print(msg)
    pose = msg.pose.pose
    subscriber.unregister()
    #result = test_action(pose)
    #print result


def listener():
    global subscriber
    print('waiting for message on topic: amcl_pose')
    subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callback)
    result = test_action()
    print result
    rospy.sleep(1)
    

if __name__ == '__main__':
    rospy.init_node('move_base_test_client')
    listener()

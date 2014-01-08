#!/usr/bin/env python

## Simple demo of a rospy service that return all rooms in of a map

PKG = 'hobbit_msgs' # this package name
NAME = 'FakeGetPlanService'

import roslib; roslib.load_manifest(PKG) 

from hobbit_msgs.srv import *
from random import choice
from nav_msgs.srv import * 
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import rospy 


pose1 = PoseStamped()
pose1.pose.position = Point(1.5, 2.3, 0)
pose1.pose.orientation = Quaternion(0, 0, 0, 0)

pose2 = PoseStamped()
pose2.pose.position = Point(5.7, 2.3, 0)
pose2.pose.orientation = Quaternion(0, 0, 0, 0)

pose3 = PoseStamped()
pose3.pose.position = Point(4.5, 2.3, 0)
pose3.pose.orientation = Quaternion(0, 0, 0, 0)

poses=[pose1, pose2, pose3]

def getPlan(req):
    print 'Returning path plan'
    global poses
    plan = Path()
    plan.poses.append(req.start)
    plan.poses.append(choice(poses))
    plan.poses.append(req.goal)
    print plan
    return GetPlanResponse(plan)

def main():
    rospy.init_node(NAME)
    print 'FakeGetPlanService'
    #s = rospy.Service('make_plan', GetPlan, getPlan)
    s = rospy.Service('/move_base/NavfnROS/make_plan', GetPlan, getPlan)


    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    main()

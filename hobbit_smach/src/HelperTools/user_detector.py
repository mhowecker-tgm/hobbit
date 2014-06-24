#!/usr/bin/env python
import rospy
from rgbd_acquisition.msg import Person

def callback(msg):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",str(msg))
    if msg.confidence > 0.5:
        print('USER detected')
    else:
        print('NO DETECTION!')
    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('smach_user_detection_helper', anonymous=True)

    rospy.Subscriber("/persons", Person, callback)

    # spin() simply keeps python from exiting uihentil this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()

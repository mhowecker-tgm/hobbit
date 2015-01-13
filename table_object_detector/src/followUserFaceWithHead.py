#!/usr/bin/python

## David Fischinger
## Date: 13.01.2015
# 
# Node listens to tf of detected head. If head is detected it tries to follow the human head
#
# subscribes to tf frame of head
# calculates the needed (robot) head direction and
# publishes the movement for the head to the topiv /head/move


PKG = 'hobbit_smach'
import roslib; roslib.load_manifest(PKG)
import rospy
#import subprocess
import time, sys
import tf
from std_msgs.msg import String


class CFollowUserHead():
    def __init__(self, parent=None):

        listener = tf.TransformListener()
        #Subscriber => change to tf subscription
        #ss_sub = rospy.Subscriber("/SS/doSingleShotTestCFS", String, self.start_shot, queue_size=1)
        #Publisher for (robot)head movement
        #self.pc_pub = rospy.Publisher("/head/move", String )
    
    #triggers the process for publishing 
    def follow_head_loop(self):
            
        #turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('/frame', '/head', rospy.Time(0))
                print "trans: ", trans
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            #check left/right head position
            if trans[0] > 0.3:
                print "head should move right"
            elif trans[0] < -0.3:
                print "head should move left"
            #check up/down head position    
            if trans[1] > 0.2:
                print "head should move up"
            elif trans[1] < -0.2:
                print "head should move down"
            
            rate.sleep()


def main(args):       
    print "node started for following users head (with Hobbits head)" 
    rospy.init_node('follow_user_head_node', anonymous=False)

    followHead = CFollowUserHead()
    followHead.follow_head_loop()
    #rospy.spin()
    
if __name__ == "__main__":        
    main(sys.argv)



   

    

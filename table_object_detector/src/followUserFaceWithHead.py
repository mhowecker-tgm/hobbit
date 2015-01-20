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
        self.node_active = True
        self.listener = tf.TransformListener()
        time.sleep(0.5)
        #Subscriber => activate/deactivate node
        self.followUserFaceActivate_sub = rospy.Subscriber("/headFollowUserHead", String, self.activate, queue_size=1)
        #Publisher for (robot)head movement
        self.headMove_pub = rospy.Publisher("/head/move/incremental", String ,queue_size=1)
    
    #activate/deacativate node function
    def activate(self, msg):
        print "followUserFaceWithHead:  msg in: ", msg.data
        if (msg.data in ("an", "TRUE", "true", "True", "1", "on", "ON", "On")):
            print "node was activated"
            self.node_active = True
        elif (msg.data in ("aus", "FALSE", "false", "False", "0", "off", "OFF", "Off")):
            print "node was deactivated"
            self.node_active = False
        
    
    #triggers the process for publishing 
    def follow_head_loop(self):
            
        #headMove_pub = rospy.Publisher('head/move', std_msgs.msg.String,queue_size=1)

        rate = rospy.Rate(1.0)
        try:
            self.listener.waitForTransform("/frame", "/head", rospy.Time(), rospy.Duration(4.0))
        except:
            print "initially no tf transform (/frame - /head) found"
            
        while not rospy.is_shutdown():
            if self.node_active:
                try:
                    now = rospy.Time.now()
                    self.listener.waitForTransform("/frame", "/head", now, rospy.Duration(4.0))
                    (trans,rot) = self.listener.lookupTransform('/frame', '/head', now)#rospy.Time.now())#Time(0))
                    print "trans: ", trans
                except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    print "no tf transform (/frame - /head) found"
                    continue
    
                strHeadMoveTmp = ""
                #check up/down head position    
                if trans[1] > 0.25:
                    print "head should move down"
                    strHeadMoveTmp = "d9 "
                elif trans[1] < -0.25:
                    print "head should move up"
                    strHeadMoveTmp = "u9 "              #!!!!!!!!!! ignores this up/down movement if it should also go right or left!!!!!!!!!!!!!!
                
                #check left/right head position
                if trans[0] > 0.3:
                    print "head should move right"
                    strHeadMoveTmp = strHeadMoveTmp + "r9"
                elif trans[0] < -0.3:
                    print "head should move left"
                    strHeadMoveTmp = strHeadMoveTmp + "l9"
    
                if (len(strHeadMoveTmp) > 0):       #head move should be executed
                    self.headMove_pub.publish(String(strHeadMoveTmp))
            
            rate.sleep()


def main(args):       
    print "node started for following users head (with Hobbits head)" 
    rospy.init_node('follow_user_head_node', anonymous=False)

    followHead = CFollowUserHead()
    followHead.follow_head_loop()
    rospy.spin()
    
if __name__ == "__main__":        
    main(sys.argv)



   

    

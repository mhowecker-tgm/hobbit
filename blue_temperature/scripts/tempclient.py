#!/usr/bin/env python
import roslib; roslib.load_manifest('blue_temperature')

import sys
import rospy
from blue_temperature.srv import *

def ir_temperature(average):
    print "averaging over " + str(average) + " sample(s)..."
    rospy.wait_for_service('blue_temperature')
    try:
        blue_temperature = rospy.ServiceProxy('blue_temperature', Temperature)
        resp = blue_temperature(average)
        print "ambient: " + str(resp.air)
        print "obj: " + str(resp.object)
	print "elapsed time: " + str(resp.elapsedtime)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    if len(sys.argv) == 2:
        average = int(sys.argv[1])
    else:
        average = 1
    ir_temperature(average)

#!/usr/bin/env python
import rospy
from hobbit_msgs.msg import Event
from std_msgs.msg import Header
LIMIT=10

def talker():
    rospy.init_node('CRON_SR', anonymous=True)
    data = Event()
    data.header = Header()
    data.header.stamp = rospy.Time.now()
    data.event = 'E_SOCIALROLE'
    pub = rospy.Publisher('Event', Event, queue_size=10)
    r = rospy.Rate(10)
    i = 0
    while i < LIMIT:
        rospy.loginfo(str(data))
        pub.publish(data)
        r.sleep()
        i += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

#!/usr/bin/env python
import rospy
from hobbit_msgs.msg import Event
from std_msgs.msg import Header

def talker():
    rospy.init_node('CRON_SF', anonymous=True)
    data = Event()
    data.header = Header()
    data.header.stamp = rospy.Time.now()
    data.event = 'E_SAFETYCHECK'
    pub = rospy.Publisher('Event', Event, queue_size=10)
    pub.publish(data)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

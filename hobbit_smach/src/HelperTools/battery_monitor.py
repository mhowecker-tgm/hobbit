#!/usr/bin/env python

import rospy
from hobbit_msgs.msg import Event
from std_msgs.msg import Header
from mira_msgs.msg import BatteryState

LIMIT = 5
NAME = 'BATTERY_CHECK'
UPPER_LIMIT = 95 # High values to test the master node
LOWER_LIMIT = 90


def talker(level):
    data = Event()
    data.header = Header()
    data.header.stamp = rospy.Time.now()
    data.event = level
    pub = rospy.Publisher('/Event', Event, queue_size=10)
    r = rospy.Rate(10)
    i = 0
    while i < LIMIT:
        rospy.loginfo(str(data))
        pub.publish(data)
        r.sleep()
        i += 1


def battery_cb(msg):
    print('received msg on topic /battery_state')
    print(msg)
    if msg.lifePercent < UPPER_LIMIT:
        rospy.loginfo('Battery level is low')
        talker('E_RECHARGE')
    if msg.lifePercent < LOWER_LIMIT:
        rospy.loginfo('Battery level is low')
        talker('E_RECHARGE_LOW')
    else:
        pass


def main():
    rospy.init_node(NAME, anonymous=False)
    rospy.Subscriber('battery_state', BatteryState, battery_cb)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

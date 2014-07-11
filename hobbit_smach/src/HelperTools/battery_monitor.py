#!/usr/bin/env python

import rospy
from hobbit_msgs.msg import Event
from std_msgs.msg import Header
from mira_msgs.msg import BatteryState

LIMIT = 5
NAME = 'BATTERY_CHECK'
VOLT_LIMIT = 26.0


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
    # print('received msg on topic /battery_state')
    # print(msg)
    if msg.voltage < VOLT_LIMIT:
        rospy.loginfo('Battery level is low')
        talker('E_RECHARGE')
    else:
        pass


def main():
    rospy.init_node(NAME, anonymous=False)
    rospy.Subscriber('battery_state', BatteryState, battery_cb)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from hobbit_msgs.msg import Event
from std_msgs.msg import Header
from mira_msgs.msg import BatteryState
from hobbit_msgs.srv import HeadSleep

NAME = 'battery_monitor'
VOLT_LIMIT = 24.5
LIMIT = False

def talker(level):
    global LIMIT
    data = Event()
    data.header = Header()
    data.header.stamp = rospy.Time.now()
    data.event = level
    pub = rospy.Publisher('/Event', Event, queue_size=10)
    if LIMIT:
        #rospy.loginfo('do not publish E_RECHARGE')
        return
    LIMIT = True
    rospy.sleep(2)
    pub.publish(data)
    rospy.loginfo('publish E_RECHARGE')

def call_service():
    rospy.wait_for_service('/head_sleep')
    head_sleep = rospy.ServiceProxy('/head_sleep', HeadSleep)
    try:
        resp = head_sleep()
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

def battery_cb(msg):
    global LIMIT
    if msg.charging:
        LIMIT = False
        call_service()
    if msg.voltage < VOLT_LIMIT and msg.charging is False:
        rospy.loginfo('Battery level is low')
        talker('E_RECHARGE')


def main():
    rospy.init_node(NAME, anonymous=False)
    rospy.Subscriber('battery_state', BatteryState, battery_cb)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import roslib
roslib.load_manifest('hobbit_smach')
import rospy
from hobbit_msgs.msg import Event
from hobbit_msgs import MMUIInterface as MMUI

def callback(data):
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    print('Event')
    print(data)
    try:
        if data.event == 'G_YES':
            msg = 'YES gesture'
        elif data.event == 'G_NO':
            msg = 'NO Gesture'
        elif data.event == 'G_CANCEL':
            msg = 'CANCEL Gesture'
        elif data.event == 'G_REWARD':
            msg = 'REWARD Gesture'
        elif data.event == 'G_HELP':
            msg = 'HELP Gesture'
    except AttributeError as e:
        print('EVENT: No Gesture.')
        print(e)

    print(msg)
    mmui = MMUI.MMUIInterface()
    resp = mmui.showMMUI_OK(text=msg)
    print(resp)

def listener():

    rospy.init_node('gesture_testing', anonymous=True)

    rospy.Subscriber("Event", Event, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()

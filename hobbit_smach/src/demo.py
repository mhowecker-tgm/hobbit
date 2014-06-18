#!/usr/bin/env python

import roslib
roslib.load_manifest('hobbit_smach')
import rospy
import actionlib
from hobbit_msgs.msg import Command, Event
from hobbit_msgs.msg import GeneralHobbitAction, GeneralHobbitGoal
from std_msgs.msg import String


def callback(data):
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    print('Command')
    print(data.command)
    try:
        if data.command == 'C_LEARN':
	    print('START learn object:')
	    learn_object()
    except AttributeError as e:
        print('Command: Not suitable here.')
        print(e)


def callback(data):
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    print('Event')
    print(data)
    try: 
        if data.command[:-1] == 'C_AWAY':
            print('START User goes away:')
	    index = data.command[-1] -1 
	    away('away', index)
        elif data.command[:-1] == 'C_SLEEP':
	    print('START User goes to sleep:')
	    index = data.command[-1] - 1
	    away('sleep', index)
        elif data.command == 'C_GOTOPOINT':
	    print('START goto point:')
	    room, place = data.params[0].value.lower().split(' ')
	    print(room)
	    print(place)
	    goto(room, place)
    except AttributeError as e:
        print('EVENT: Not suitable here.')
        print(e)

    try: 
        if data.event == 'E_HELP':
	    print('START User initiated emergency:')
	    sos()
    except AttributeError as e:
        print('EVENT: Not suitable here.')
        print(e)
    
def listener():

    rospy.init_node('demo', anonymous=True)

    rospy.Subscriber("Command", Command, callback)
    rospy.Subscriber("Event", Event, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def learn_object():
    print "start client"
    client = actionlib.SimpleActionClient(
        'learn_object', GeneralHobbitAction)
    client.wait_for_server()
    print('connected to server')
    par = []
    goal = GeneralHobbitGoal(command=String('learn_object'),
                             previous_state=String('call_hobbit'),
                             parameters=par)
    print('learn_object')
    client.send_goal(goal)
    print('waiting for result')
    client.wait_for_result()
    return client.get_result()


def sos():
    print "start client"
    client = actionlib.SimpleActionClient(
        'emergency_user',
        GeneralHobbitAction)
    client.wait_for_server()
    print "connected to server"
    par = []
    par.append(String('user_initiated'))
    goal = GeneralHobbitGoal(
        command=String('Emergency'),
        parameters=par)
    print goal
    print "send goal"
    client.send_goal(goal)
    print "waiting for result"
    client.wait_for_result()

    return client.get_result()

def away(input_cmd, index):
    times = [1, 2, 4, 6, 12, 24]
    print "start client"
    client = actionlib.SimpleActionClient(
        'away', GeneralHobbitAction)
    client.wait_for_server()
    print('connected to server')
    par = []
    par.append(String(times[index]))
    goal = GeneralHobbitGoal(command=String(input_cmd),
                             previous_state=String('call_hobbit'),
                             parameters=par)
    print(goal)
    print(type(goal))
    print(input_cmd)
    client.send_goal(goal)
    print('waiting for result')
    client.wait_for_result()

    return client.get_result()

def goto(room, place):
    print "start client"
    client = actionlib.SimpleActionClient(
        'goto', GeneralHobbitAction)
    client.wait_for_server()
    print('connected to server')
    par = []
    par.append(String(room))
    par.append(String(place))
    goal = GeneralHobbitGoal(command=String('goto'),
                             previous_state=String('call_hobbit'),
                             parameters=par)
    print(goal)
    print(type(goal))
    print('goto')
    client.send_goal(goal)
    print('waiting for result')
    client.wait_for_result()

    return client.get_result()
        
if __name__ == '__main__':
    listener()

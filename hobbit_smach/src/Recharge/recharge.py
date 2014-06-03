#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'recharge'

DEBUG = True

import roslib
roslib.load_manifest(PKG)
import rospy
# import smach
# import uashh_smach.util as util

from std_msgs.msg import String
import hobbit_smach.hobbit_move_import as hobbit_move
from hobbit_msgs.msg import GeneralHobbitAction,\
    LocateUserAction, LocateUserGoal, ApproachUserAction, ApproachUserGoal,\
    Event
from hobbit_msgs.srv import GetCoordinates
from smach_ros import ActionServerWrapper, \
    SimpleActionState, IntrospectionServer
from smach import StateMachine, State, Sequence, cb_interface
#from smach_ros import ServiceState
from hobbit_user_interaction import HobbitMMUI, HobbitEmotions
import hobbit_smach.speech_output_import as speech_output
#import uashh_smach.platform.move_base as move_base


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

    def disable(self):
        self.HEADER = ''
        self.OKBLUE = ''
        self.OKGREEN = ''
        self.WARNING = ''
        self.FAIL = ''
        self.ENDC = ''


class Init(State):
    """Class to initialize certain parameters"""
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'canceled'],
            input_keys=['command'], output_keys=['social_role'])
        self.pub_head = rospy.Publisher('HeadMove', String)
        self.pub_obstacle = rospy.Publisher('/headcam/active', String)
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)
        self.pub_face.publish('EMO_NEUTRAL')

    def execute(self, ud):
        self.pub_face.publish('EMO_NEUTRAL')
        self.pub_head.publish('down')
        self.pub_obstacle.publish('active')
        if rospy.has_param('/hobbit/social_role'):
            ud.social_role = rospy.get_param('/hobbit/social_role')
        return 'succeeded'


class BatteryLevelCheck(State):
    """
    Class which checks from where the recharge scenario was called.
    """
    def __init__(self):
        State.__init__(self,
                       outcomes=['tired',
                                 'vtired',
                                 'preempted',
                                 'failure'],
                       input_keys=['command', 'parameters']
                       )

    def execute(self, ud):
        if self.preempt_requested():
            ud.result = String('preempted')
            return 'preempted'
        print(ud.parameters)
        if ud.parameters[0] == 'TIRED':
            return 'tired'
        else:
            return 'vtired'


class SetYes(State):
    """
    Set the correct values for HobbitEmotions and the corresponding questions.
    """
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'preempted', 'failed'],
                       output_keys=['emotion', 'emo_time', 'text']
                       )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.emotion = String('HAPPY')
        ud.emo_time = 4
        ud.text = 'T_CH_MovingToChargingStation'
        return 'succeeded'


class SetNo(State):
    """
    Set the correct values for HobbitEmotions and the corresponding questions.
    """
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'preempted', 'failed'],
                       output_keys=['emotion', 'emo_time', 'text']
                       )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        ud.text = 'T_CH_MovingToWaitingPosition'
        return 'succeeded'


class Dummy(State):
    """
    Class for setting the result message and clean up persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['command'],
            output_keys=['result', 'command']
        )
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
        #ud.result = String('')
        #rospy.sleep(2.0)
        return 'succeeded'


class SetSuccess(State):
    """
    Class for setting the success message in the actionlib result \
        and clean up of persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            output_keys=['result']
        )
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_HAPPY')
        self.pub.publish('Stop')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('succeeded')
        return 'succeeded'


class SetFailure(State):
    """
    Class for setting the failure message in the actionlib result \
        and clean up of persistent variables
    """
    def __init__(self):
        State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            output_keys=['result']
        )
        self.pub = rospy.Publisher('/DiscreteMotionCmd', String)
        self.pub_face = rospy.Publisher('/Hobbit/Emoticon', String)

    def execute(self, ud):
        self.pub_face.publish('EMO_SAD')
        self.pub.publish('Stop')
        if self.preempt_requested():
            ud.result = String('preempted')
            self.service_preempt()
            return 'preempted'
        ud.result = String('failure')
        return 'succeeded'


@cb_interface(input_keys=['current_robot_room'])
def set_nav_goal_cb(userdata, request):
    if rospy.has_param('Hobbit/docking_room'):
        docking_room = rospy.get_param('Hobbit/docking_room')
    if rospy.has_param('Hobbit/social_role'):
        social_role = rospy.get_param('Hobbit/social_role')
    else:
        social_role = 0
    if social_role == 3:
        # companion
        goal_room = String(userdata.current_robot_room)
        goal_location = String('dock')
    else:
        # tool, butler
        goal_room = String(docking_room)
        goal_location = String('default')
    nav_request = GetCoordinates().Request
    nav_request.room_name = goal_room
    nav_request.location_name = goal_location
    return nav_request


def main():
    rospy.init_node(NAME)

    rech_sm = StateMachine(
        outcomes=['succeeded', 'failure', 'preempted'],
        input_keys=['command', 'previous_state', 'parameters'],
        output_keys=['result'])

    rech_sm.userdata.result = String('started')

    seq = Sequence(
        outcomes=['succeeded', 'failed', 'preempted'],
        connector_outcome='succeeded'
    )

    with rech_sm:
        StateMachine.add(
            'INIT',
            Init(),
            transitions={'succeeded': 'BATTERY_LEVEL_CHECK',
                         'canceled': 'SET_FAILURE'})
        StateMachine.add(
            'BATTERY_LEVEL_CHECK',
            BatteryLevelCheck(),
            transitions={'preempted': 'SET_FAILURE',
                         'failure': 'SET_FAILURE',
                         'vtired': 'EMO_VTIRED',
                         'tired': 'EMO_TIRED'}
        )
        StateMachine.add(
            'EMO_TIRED',
            HobbitEmotions.ShowEmotions(emotion='TIRED', emo_time=0),
            transitions={'succeeded': 'LOCATE_USER',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'EMO_VTIRED',
            HobbitEmotions.ShowEmotions(emotion='VTIRED', emo_time=0),
            transitions={'succeeded': 'LOCATE_USER',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        if not DEBUG:
            StateMachine.add(
                'LOCATE_USER',
                SimpleActionState(
                    'locate_user',
                    LocateUserAction,
                    goal=LocateUserGoal(command=String('locateUser'))),
                {'succeeded': 'APPROACH_USER',
                    'failed': 'SET_FAILURE',
                    'preempted': 'preempted'}
            )
            StateMachine.add(
                'APPROACH_USER',
                SimpleActionState(
                    'approach_user',
                    ApproachUserAction,
                    goal=ApproachUserGoal(command=String('approachUser'))),
                {'succeeded': 'MMUI_CONFIRM',
                    'failed': 'SET_FAILURE',
                    'preempted': 'preempted'}
            )
        else:
            StateMachine.add(
                'LOCATE_USER',
                Dummy(),
                transitions={'succeeded': 'APPROACH_USER'}
            )
            StateMachine.add(
                'APPROACH_USER',
                Dummy(),
                transitions={'succeeded': 'MMUI_CONFIRM'}
            )
        rech_sm.userdata.question = String()
        StateMachine.add(
            'MMUI_CONFIRM',
            HobbitMMUI.AskYesNo(question='T_CH_TiredRelax'),
            transitions={'yes': 'MMUI_SAY_MovingToChargingStation',
                         'no': 'MMUI_SAY_MovingToWaitingPosition',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted',
                         'timeout': 'MMUI_CONFIRM',
                         '3times': 'SET_FAILURE'}
        )
        StateMachine.add(
            'MMUI_SAY_MovingToChargingStation',
            #HobbitMMUI.ShowInfo(info='T_CH_MovingToChargingStation'),
            speech_output.sayText(info='T_CH_MovingToChargingStation'),
            transitions={'succeeded': 'EMO_HAPPY',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'MMUI_SAY_MovingToWaitingPosition',
            #HobbitMMUI.ShowInfo(info='T_CH_MovingToWaitingPosition'),
            speech_output.sayText(info='T_CH_MovingToWaitingPosition'),
            transitions={'succeeded': 'SEQ',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'EMO_HAPPY',
            HobbitEmotions.ShowEmotions(emotion='HAPPY', emo_time=4),
            transitions={'succeeded': 'SEQ',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        StateMachine.add(
            'SEQ',
            seq,
            transitions={'succeeded': 'SET_SUCCESS',
                         'failed': 'SET_FAILURE',
                         'preempted': 'preempted'}
        )
        with seq:
            #Sequence.add(
            #    'WAIT_FOR_MMUI',
            #    HobbitMMUI.WaitforSoundEnd('/Event', Event),
            #    transitions={'aborted': 'WAIT_FOR_MMUI'})
            if not DEBUG:
                Sequence.add(
                    'MOVE_TO_DOCK',
                    hobbit_move.goToPosition(frame='/map', place='dock'))
                Sequence.add(
                    'DOCKING',
                    hobbit_move.dock())
            else:
                Sequence.add('MOVE_TO_DOCK', Dummy())
            # DOCKING has to execute the actual docking procedure within mira
            Sequence.add('DOCKING', Dummy())
            Sequence.add('MMUI_MAIN_MENU', HobbitMMUI.ShowMenu(menu='MAIN'))
        StateMachine.add(
            'SET_SUCCESS',
            SetSuccess(),
            transitions={'succeeded': 'succeeded',
                         'preempted': 'SET_FAILURE'}
        )
        StateMachine.add(
            'SET_FAILURE',
            SetFailure(),
            transitions={'succeeded': 'failure',
                         'preempted': 'preempted'}
        )
        #StateMachine.add(
        #    'CLEAN_UP',
        #    CleanUp(),
        #    transitions={'succeeded': 'preempted'})

    asw = ActionServerWrapper(
        'recharge',
        GeneralHobbitAction,
        rech_sm,
        ['succeeded'], ['failure'], ['preempted'],
        result_slots_map={'result': 'result'},
        goal_slots_map={'command': 'command',
                        'previous_state': 'previous_state',
                        'parameters': 'parameters'}
    )

    sis = IntrospectionServer('smach_server', rech_sm, '/HOBBIT/rech_sm_ROOT')
    sis.start()
    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_user_interaction'
NAME = 'hobbit_mmui'

import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import threading

from std_msgs.msg import String
from hobbit_msgs import MMUIInterface as MMUI
import uashh_smach.util as util


class WaitforSoundEnd(util.WaitForMsgState):
    """
    Inherit from util.WaitForMsgState to wait
    for S_SAPIEND
    """

    def execute(self, ud):
        """
            Override execute() to check for S_SAPIEND
        """
        msg = self.waitForMsg()
        if msg is not None:
            #rospy.loginfo(str(msg))
            if msg == 'preempted':
                return 'preempted'
            # call callback if there is one
            if self.msg_cb is not None:
                cb_result = self.msg_cb(msg, ud)
                # check if callback wants to dictate output
                if cb_result is not None:
                    print('self.msg_cb is not None')
                    if cb_result:
                        return 'succeeded'
                    else:
                        return 'failed'
            if msg.event == 'S_SAPIEND':
                return 'succeeded'
            else:
                return 'aborted'
        else:
            return 'aborted'


class CallDecision(smach.State):

    """
    Inherit from util.WaitForMsgState to wait
    for S_SAPIEND
    """

    def __init__(self):
        smach.State.__init__(
            self,
            input_keys=['call_state'],
            outcomes=['succeeded', 'failed', 'preempted',
                      'stop', 'ended', 'established', 'confirmed']
        )

    def execute(self, ud):
        if ud.call_state == 'stop':
            return 'stop'
        elif ud.call_state == 'ended':
            return 'ended'
        elif ud.call_state == 'established':
            return 'established'
        elif ud.call_state == 'confirmed':
            return 'confirmed'
        else:
            return 'succeeded'


class WaitforEndedCall(util.WaitForMsgState):
    """
    Inherit from util.WaitForMsgState to wait
    for S_SAPIEND
    """

    def execute(self, ud):
        """
            Override execute() to check for S_SAPIEND
        """
        msg = self.waitForMsg()
        if msg is not None:
            print(msg)
            if msg == 'preempted':
                return 'preempted'
            # call callback if there is one
            if self.msg_cb is not None:
                cb_result = self.msg_cb(msg, ud)
                # check if callback wants to dictate output
                if cb_result is not None:
                    print('self.msg_cb is not None')
                    if cb_result:
                        return 'succeeded'
                    else:
                        return 'failed'
            if msg.event == 'E_CALLENDED' or msg.event == 'B_ENDSOS':
                return 'succeeded'
            else:
                return 'aborted'
        else:
            return 'aborted'


class WaitforConfirmedCall(util.WaitForMsgState):
    """
    Inherit from util.WaitForMsgState to wait
    for some Events
    """

    def execute(self, ud):
        """
            Override execute() to check for some Events
        """
        msg = self.waitForMsg()
        if msg is not None:
            print(msg)
            if msg == 'preempted':
                return 'preempted'
            # call callback if there is one
            if self.msg_cb is not None:
                cb_result = self.msg_cb(msg, ud)
                # check if callback wants to dictate output
                if cb_result is not None:
                    print('self.msg_cb is not None')
                    if cb_result:
                        return 'succeeded'
                    else:
                        return 'failed'
            if msg.event == 'E_CALLCONFIRMED':
                ud.call_state = 'confirmed'
                return 'succeeded'
            elif msg.event == 'E_CALLESTABLISHED':
                ud.call_state = 'established'
                return 'succeeded'
            elif msg.event == 'B_STOPSOS':
                ud.call_state = 'stop'
                return 'succeeded'
            elif msg.event == 'E_CALLENDED' or msg.event == 'B_CALLENDED' or\
                msg.event == 'B_ENDSOS':
                ud.call_state = 'ended'
                return 'succeeded'
            else:
                return 'aborted'
        else:
            return 'aborted'


class ShowMenu(smach.State):
    """
    Class to interact with the MMUI
    """
    def __init__(self, menu=None):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'preempted'],
                             input_keys=['menu'],
                             )
        self.menu = menu

    def execute(self, ud):
        if self.preempt_requested():
            ud.answer = String('preempted')
            self.service_preempt()
        menu = 'M_' + self.menu if self.menu is not None else None
        if not menu:
            return 'failed'
        mmui = MMUI.MMUIInterface()
        resp = mmui.GoToMenu(menu=menu)
        print(resp)
        # TODO: needs checkfor resp content
        if resp:
            return 'succeeded'
        else:
            return 'failed'


class AskYesNo(smach.State):
    """
    Class to interact with the MMUI
    """
    def __init__(self, question):
        smach.State.__init__(self,
                             outcomes=['yes', 'no', 'failed', 'preempted',
                                       'timeout', '3times'],
                             input_keys=['question'],
                             output_keys=['answer'])
        self.question = question
        self.timeout_count = 0

    def execute(self, ud):
        print(self.timeout_count)
        if self.timeout_count > 2:
            self.timeout_count = 0
            return '3times'
        mmui = MMUI.MMUIInterface()
        resp = mmui.showMMUI_YESNO(text=self.question)
        if self.preempt_requested():
            ud.answer = String('preempted')
            self.service_preempt()
            return 'preempted'
        if resp:
            for i, v in enumerate(resp.params):
                print i, v
                if v.name == 'result' and v.value == 'D_YES':
                    return 'yes'
                elif v.name == 'result' and v.value == 'D_NO':
                    return 'no'
                elif v.name == 'result' and v.value == 'D_CANCEL':
                    return 'no'
                elif v.name == 'result' and v.value == 'D_TIMEOUT':
                    self.timeout_count += 1
                    return 'timeout'
                else:
                    pass
            return 'failed'
        else:
            return 'failed'


class ConfirmInfo(smach.State):
    """
    Class to interact with the MMUI
    """
    def __init__(self, info):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'preempted']
                             )
        self.info = info

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.showMMUI_OK(text=self.info)
        print(resp)
        # TODO: needs check for resp content
        if resp:
            return 'succeeded'
        else:
            return 'failed'


class ConfirmOk(smach.State):
    """
    Class to interact with the MMUI
    """
    def __init__(self, text):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted', 'timeout', '3times']
        )
        self.text = text
        self.timeout_count = 0

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        print(self.timeout_count)
        if self.timeout_count > 2:
            self.timeout_count = 0
            return '3times'
        mmui = MMUI.MMUIInterface()
        resp = mmui.showMMUI_OK(text=self.text)
        if resp:
            rospy.loginfo(str(resp))
            for i, v in enumerate(resp.params):
                print i, v
                if v.name == 'result' and v.value == 'D_OK':
                    return 'succeeded'
                elif v.name == 'result' and v.value == 'D_CANCEL':
                    return 'aborted'
                elif v.name == 'result' and v.value == 'D_TIMEOUT':
                    self.timeout_count += 1
                    return 'timeout'
                else:
                    self.timeout_count += 1
                    return 'timeout'
        else:
            self.timeout_count += 1
            return 'timeout'


class ShowInfo(smach.State):

    """
    Class to interact with the MMUI
    """

    def __init__(self, info, place='roomname', object_name='object_name'):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'preempted']
                             )
        self.info = info
        self.place = place
        self.object_name = object_name

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.showMMUI_Info(
	    text=self.info)
        return 'succeeded'
        if not self.place == 'roomname':
            mmui.showMMUI_Info(
                text=self.info, prm=self.place)
        elif not self.object_name == 'object_name':
            mmui.showMMUI_Info(
                text=self.info, prm=self.object_name)
        else:
            mmui.showMMUI_Info(
                text=self.info)
            return 'succeeded'


class CancelState(smach.State):
    """
    Class to interact with the MMUI
    This smach State will try to cancel all ongoing actions
    on the MMUI
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'preempted'],
                             input_keys=['text'],
                             )

    def execute(self, ud):
        if self.preempt_requested():
            ud.answer = String('preempted')
            self.service_preempt()
        mmui = MMUI.MMUIInterface()
        resp = mmui.showMMUI_Info(text=ud.text)
        print(resp)
        # TODO: needs check for resp content
        if resp:
            return 'succeeded'
        else:
            return 'failed'


class ShowCalendar(smach.State):
    """
    Smach State class to show calendar entries based on the given timeframe
    and categories.
    """
    def __init__(self, timeframe='03:00', categories=['meeting']):
        smach.State.__init__(
            self,
            outcomes=['failed', 'succeeded', 'preempted']
            # input_keys=['timeframe', 'categories']
        )
        self._categories = categories
        self._timeframe = timeframe

    def execute(self, ud):
        if self.preempt_requested():
            ud.answer = String('preempted')
            self.service_preempt()
        mmui = MMUI.MMUIInterface()
        resp = mmui.showMMUI_Calendar(timespan=self._timeframe, cat=self._categories[0])
        print(resp)
        if resp.params[0].value == 'D_OK':
            return 'succeeded'
        elif resp.params[0].value == 'D_CANCEL':
            return 'failed'
        elif resp.params[0].value == 'D_TIMEOUT':
            return 'failed'
        else:
            return 'failed'


class AskForCmd(smach.State):
    """
    Class to interact with the MMUI
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['question'])

    def execute(self, ud):
        mmui = MMUI.MMUIInterface()
        resp = mmui.showMMUI_NAME(ud.question)
        if resp:
            return 'succeeded'
        else:
            return 'failed'


class CallEmergency(smach.State):
    """
    Class to start the Voip-call to emergency personal
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'failed']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.StartSOSCall()
        print('response from StartSOSCall was: ')
        print(resp)
        return 'succeeded'


class CallEmergencySimple(smach.State):
    """
    Class to start the Voip-call to emergency personal
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.StartSOSCall()
        print('response from StartSOSCall was: ')
        print(resp)
        if resp:
            for i, v in enumerate(resp.params):
                print i, v
                if v.name == 'result' and v.value == 'S_OK':
                    return 'succeeded'
                else:
                    pass
            return 'aborted'
        else:
            return 'aborted'


class CallEnded(smach.State):
    """
    Call has ended, check if we have to hang up.
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'failed'],
            input_keys=['call_state']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.sendMMUI_Function('F_ENDCALL')
        print('response from StartSOSCall was: ')
        print(resp)
        return 'succeeded'


class SetAbsVolume(smach.State):
    """
    """
    def __init__(self, volume):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['call_state']
        )
        self.volume = volume

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.set_abs_volume(volume=self.volume)
        rospy.loginfo('F_ABSVOLUME: '+str(resp))
        return 'succeeded'


class SetVolumeHigher(smach.State):
    """
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['call_state']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.set_volume_louder()
        rospy.loginfo('F_LOUDER: '+str(resp))
        return 'succeeded'


class SetVolumeLower(smach.State):
    """
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['call_state']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.set_volume_quieter()
        rospy.loginfo('F_QUIETER: '+str(resp))
        return 'succeeded'


class CheckSOSCall(util.WaitForMsgState):
    """
    Inherit from util.WaitForMsgState to wait
    for the end of an emergency call.
    """
    def __init__(self, topic, msg_type, msg_cb=None, output_keys=None, latch=False, timeout=None):
        if output_keys is None:
            output_keys = []
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted', 'failed'],
            output_keys=output_keys)
        self.latch = latch
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.msg = None
        self.msg_cb = msg_cb
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)

    def execute(self, ud):
        """
            Override execute() to check for some Events
        """
        msg = self.waitForMsg()
        if msg is not None:
            print(msg)
            if msg == 'preempted':
                return 'preempted'
            # call callback if there is one
            if self.msg_cb is not None:
                cb_result = self.msg_cb(msg, ud)
                # check if callback wants to dictate output
                if cb_result is not None:
                    print('self.msg_cb is not None')
                    if cb_result:
                        return 'succeeded'
                    else:
                        return 'failed'
            if msg.event == 'B_ENDSOS':
                return 'failed'
            elif msg.event == 'B_STOPSOS':
                return 'succeeded'
            elif msg.event == 'E_CALLENDED':
                return 'succeeded'
            else:
                return 'aborted'
        else:
            return 'aborted'

class RemoveLastPrompt(smach.State):
    """
    Class to remove the last shown prompt on the MMUI
    This helps keeping everything preemptable
    """
    def __init__(self, menu=None):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )
        self.menu = menu

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
        mmui = MMUI.MMUIInterface()
        mmui.remove_last_prompt()
        return 'succeeded'


if __name__ == '__main__':
    print('You should not call this directly. Import the needed classes')
    mmui = MMUI.MMUIInterface()

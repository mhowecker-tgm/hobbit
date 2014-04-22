#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_user_interaction'
NAME = 'hobbit_mmui'

import roslib
roslib.load_manifest(PKG)
import smach

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
                        return 'aborted'
            if msg.event == 'S_SAPIEND':
                return 'succeeded'
            else:
                return 'aborted'
        else:
            return 'aborted'


class ShowMenu(smach.State):
    """
    Class to interact with the MMUI
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['menu'],
                             )

    def execute(self, ud):
        if self.preempt_requested():
            ud.answer = String('preempted')
            self.service_preempt()
        menu = 'M_' + ud.menu.data
        print(menu)
        mmui = MMUI.MMUIInterface()
        resp = mmui.GoToMenu(menu=menu)
        print(resp)
        # TODO: needs checkfor resp content
        if resp:
            return 'succeeded'
        else:
            return 'aborted'


class AskYesNo(smach.State):
    """
    Class to interact with the MMUI
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['yes', 'no', 'aborted', 'preempted'],
                             input_keys=['question'],
                             output_keys=['answer'])

    def execute(self, ud):
        if self.preempt_requested():
            ud.answer = String('preempted')
            self.service_preempt()
        mmui = MMUI.MMUIInterface()
        resp = mmui.showMMUI_YESNO(text=ud.question.data)
        if resp:
            for i, v in enumerate(resp.params):
                print i, v
                if v.name == 'result' and v.value == 'D_YES':
                    return 'yes'
                elif v.name == 'result' and v.value == 'D_NO':
                    return 'no'
                elif v.name == 'result' and v.value == 'D_CANCEL':
                    #return 'aborted'
                    return 'yes'
                else:
                    return 'aborted'
        else:
            return 'aborted'


class ShowInfo(smach.State):
    """
    Class to interact with the MMUI
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['text']
                             )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        mmui = MMUI.MMUIInterface()
        resp = mmui.showMMUI_Info(text=ud.text)
        print(resp)
        # TODO: needs checkfor resp content
        if resp:
            return 'succeeded'
        else:
            return 'failed'


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
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['aborted', 'succeeded', 'preempted'],
            input_keys=['timeframe', 'categories']
        )

    def execute(self, ud):
        if self.preempt_requested():
            ud.answer = String('preempted')
            self.service_preempt()
        mmui = MMUI.MMUIInterface()
        resp = mmui.showMMUI_Calendar(timespan=ud.timeframe, cat=ud.categories)
        print(resp)
        if resp[0].value == 'D_OK':
            return 'succeeded'
        elif resp[0].value == 'D_CANCEL':
            return 'failed'
        elif resp[0].value == 'D_TIMEOUT':
            return 'aborted'
        else:
            return 'aborted'


class AskForCmd(smach.State):
    """
    Class to interact with the MMUI
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['question'])

    def execute(self, ud):
        resp = MMUI.showMMUI_NAME(ud.question)
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
            outcomes=['succeeded', 'preempted']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        MMUI.sendMMUI_Function('F_CALLSOS')
        return 'succeeded'


class CallEnded(smach.State):
    """
    Call has ended, check if we have to hang up.
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            input_keys=['call_state']
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if ud.call_state.upper() == 'B_STOPSOS'\
                or ud.call_state.upper() == 'B_ENDSOS':
            return 'succeeded'
        elif ud.call_state.upper() == 'E_CALLCONFIRMED':
            MMUI.sendMMUI_Function('F_ENDCALL')
            return 'succeeded'


if __name__ == '__main__':
    print('You should not call this directly. Import the needed classes')
    mmui = MMUI.MMUIInterface()

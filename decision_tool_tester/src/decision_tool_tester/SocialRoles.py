#!/usr/bin/env python

import roslib
import rospy
import string
from decision_service.decision_tool import DecisionTool
from random import randint

class HobbitTexts:
    def __init__(self, engineid):
        self.engineid = engineid
   
        path = roslib.packages.get_pkg_dir('decision_service') + '/testfiles/'
        self.dt = DecisionTool()
        
        if self.dt.EngineExists(engineid):
            self.dt.DisposeEngine(engineid)
        self.dt.CreateEngine(engineid, path + 'example4.txt')

    # only for testing, social role should change according to user interaction
    def change_role(self, role):
        self.dt.SetGlobalConditional('ROBOT.SocialRole', role)

    # get back the appropriate text id
    def get_text(self, textid):
        self.dt.SetGlobalConditional('ROBOT.CurrentText', textid)
        return self.dt.Evaluate(self.engineid, 'TextId')
    
def main():
    
    phrases = {
               # Device - choice 1
               "T_PU_ObjectNotDetected_Dev_0":"Object not detected. Point again.",
               # Device - choice 2 
               "T_PU_ObjectNotDetected_Dev_1":"Unable to detect object. Try again.",
               # Device - choice 3
               "T_PU_ObjectNotDetected_Dev_2":"Could not detect object. Point again.",
               # Butler - choice 1
               "T_PU_ObjectNotDetected_But_0":"I could not detect the object you are pointing at. Please point at the object again.",
               # Butler - choice 2
               "T_PU_ObjectNotDetected_But_1":"I can't find the object. Could you please help me and point at the object again?",
               # Butler - choice 3
               "T_PU_ObjectNotDetected_But_2":"I have to move to a better position to find the object. Please point again at the object.",
               # Companion - choice 1
               "T_PU_ObjectNotDetected_Comp_0":"Unfortunately, I can't find the object you are pointing at. I have to move to a better position. Please help me and point at the object again.",
               # Companion - choice 2
               "T_PU_ObjectNotDetected_Comp_1":"I'm sorry, I cannot detect the object you are pointing at. I have to move to a better position. Please point at the object again to help me.",
               # Companion - choice 3
               "T_PU_ObjectNotDetected_Comp_2":"I was not able to find the object you are pointing at. Can you help me move to a better position by pointing at the object again?"
    }
    
    ht = HobbitTexts('SocialRolesEngine')
    
    # ROBOT.SocialRole = Device (for testing only)
    ht.change_role(1)
    
    print "Text to display when social role = Device:"
    print phrases[ht.get_text('T_PU_ObjectNotDetected')]

    # ROBOT.SocialRole = Butler (for testing only)
    ht.change_role(2)
    
    print "\nText to display when social role = Butler:"
    print phrases[ht.get_text('T_PU_ObjectNotDetected')]
    
    # ROBOT.SocialRole = Companion (for testing only)
    ht.change_role(4)
    
    print "\nText to display when social role = Companion:"
    print phrases[ht.get_text('T_PU_ObjectNotDetected')]


if __name__ == "__main__":
    main()

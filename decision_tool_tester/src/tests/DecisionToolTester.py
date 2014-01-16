#!/usr/bin/env python

import roslib
import rospy
from decision_service.decision_tool import DecisionTool

def main():
    # id for first engine
    firstid = 'id1'
    
    # id for second engine
    secondid = 'id2'
    
    # Get path to testfiles (path is for DecisionService node)
    path = roslib.packages.get_pkg_dir('decision_service') + '/testfiles/'
    
    dt = DecisionTool()
    # Create a new engine for a particular component and load the respective ruleset
    if dt.EngineExists(firstid):
        dt.DisposeEngine(firstid)
    dt.CreateEngine(firstid, path + 'example1.txt')
    
    # Create a new engine for another component and load the respective ruleset
    if dt.EngineExists(secondid):
        dt.DisposeEngine(secondid)
    dt.CreateEngine(secondid, path + 'example2.txt')
    
    # ================================================================================
    # Engine 1 - Access to The Profile
    
    # Load profile values from a file for the first engine
    dt.LoadProfile(firstid, path + 'profile.txt')
    
    # Evaluate a rule that depends on user parameters
    print '\n== Eval using Engine 1 =='
    print 'ErrorReporting 1: ' + dt.Evaluate(firstid, 'ErrorReporting')
    
    # Set a different value for user vision
    dt.SetConditional(firstid, 'user.vision', True)
    
    # Evaluate a rule using the new value
    print 'ErrorReporting 2: ' + dt.Evaluate(firstid, 'ErrorReporting')
    
    # Print a profile attribute
    print 'User name: ' + dt.GetProfileStringAttribute(firstid, 'user.name')
    
    # Set another value for user name
    dt.SetConditional(firstid, 'user.name', 'John')
    
    # Store the profile to disk
    dt.SaveProfile(firstid, roslib.packages.get_pkg_dir('decision_tool_tester') + '/saved_profile.txt')
    
    # Dispose the first engine
    dt.DisposeEngine(firstid)
    
    # ================================================================================
    # Engine 2
    
    # Set conditional at the second engine
    dt.SetConditional(secondid, 'screen.brightness', 5)
    
    # Evaluate a rule that depends on the above conditional
    print '\n== Eval using Engine 2 =='
    print 'ScreenContrast: ' + dt.Evaluate(secondid, 'ScreenContrast')
    
    # Set another conditional at the second engine
    # These values cause an error (see example2.txt)
    dt.SetConditional(secondid, 'screen.width', 1920)
    dt.SetConditional(secondid, 'screen.height', 1080)
    print 'ScreenSize: ' + dt.Evaluate(secondid, 'ScreenSize')
    
    # Dispose the second engine
    dt.DisposeEngine(secondid)
    
if __name__ == "__main__":
    main()
#    print decision_service.decision_tool.DecisionTool

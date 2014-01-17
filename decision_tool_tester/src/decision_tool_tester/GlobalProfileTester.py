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
    dt.CreateEngine(firstid, path + 'example3.txt')
    
    # Create a new engine for another component and load the SAME ruleset
    if dt.EngineExists(secondid):
        dt.DisposeEngine(secondid)
    dt.CreateEngine(secondid, path + 'example3.txt')
    
    # Set global parameter (male/female)
    dt.SetGlobalConditional('user.male', True)
    
    # The first engine will use capital letters, while the second will not
    dt.SetConditional(firstid, 'response.capslock', True)
    dt.SetConditional(secondid, 'response.capslock', False)
    
    # Evaluate WhatAmI for each engine
    print 'What am I, first engine?  : ' + dt.Evaluate(firstid, 'WhatAmI')
    print 'What am I, second engine?  : ' + dt.Evaluate(secondid, 'WhatAmI')
    
    # Dispose the first engine
    dt.DisposeEngine(firstid)
    
    # Dispose the second engine
    dt.DisposeEngine(secondid)
    
    # Playing with global attributes (names do not follow conventions for demo purposes)
    dt.SetGlobalConditional('hobbit.voice.volume', 7)
    dt.SetGlobalConditional('hobbit.voice.male', False)
    dt.SetGlobalConditional('hobbit.voice.pitch', 'high')
    
    # Just test attributes to check if the prefix services work
    dt.SetGlobalConditional('hobbit.name', 'George')
    dt.SetGlobalConditional('house.temperature', 27.2)
    
    # Test engine
    if dt.EngineExists('test'):
        dt.DisposeEngine('test')
    dt.CreateEngine('test', path + 'example1.txt')
    dt.SetConditional('test', 'hobbit.nickname', 'Bob')
    dt.SetConditional('test', 'house.size', 'Big')
    
    # Print what each profile contains at this point
    print '\nGlobal profile:\nhobbit.voice.volume = 7\nhobbit.voice.male = false\nhobbit.voice.pitch = high'
    print 'hobbit.name = George\nhouse.temperature = 27.2\n'
    print 'Local parameters for engine \"test\":\nhobbit.nickname = Bob\nhouse.size = Big\n'
    
    # Get attributes based on prefix
    print 'Getting local attributes for \"test\" and global attributes beginning with \"house.\":'
    print dt.GetProfileAttributesWithPrefix('test', 'house.*')
    
    print 'Getting local attributes for \"test\" and global attributes beginning with \"ho\":'
    print dt.GetProfileAttributesWithPrefix('test', 'ho*')
    
    print 'Getting global attributes beginning with \"hobbit.\":'
    # alternative way to achieve the same result as before
    print dt.GetGlobalProfileAttributesWithPrefix('hobbit.*')
    
    dt.DisposeEngine('test')
    
if __name__ == "__main__":
    main()

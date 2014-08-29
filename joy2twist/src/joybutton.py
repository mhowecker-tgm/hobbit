#!/usr/bin/python

## walter wohlkinger
## first version: 27.04.2011

PKG = 'joy2twist'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import numpy
import serial
from std_msgs.msg import String, Header
from std_srvs.srv import Empty
from mira_msgs.srv import ResetMotorStop
from sensor_msgs.msg import Joy
from hobbit_msgs.msg import Command, Status, Event, Parameter
from hobbit_msgs import MMUIInterface as MMUI

pubC = rospy.Publisher("/Command", Command) 
pubE = rospy.Publisher("/Event", Event)    

# rospy.wait_for_service('/face_detection/pause')
# rospy.wait_for_service('/face_detection/resume')
# rospy.wait_for_service('/emergency_detector/pause')
# rospy.wait_for_service('/emergency_detector/resume')
# rospy.wait_for_service('/hand_gestures/pause')
# rospy.wait_for_service('/hand_gestures/resume')
# rospy.wait_for_service('/skeleton_detector/pause')
# rospy.wait_for_service('/skeleton_detector/resume')
# rospy.wait_for_service('/reset_motorstop')
    
srvFacePause = rospy.ServiceProxy('/face_detection/pause', Empty)
srvFaceResume = rospy.ServiceProxy('/face_detection/resume', Empty)
srvEmergencyPause = rospy.ServiceProxy('/emergency_detector/pause', Empty)
srvEmergencyResume = rospy.ServiceProxy('/emergency_detector/resume', Empty)
srvGesturesPause = rospy.ServiceProxy('/hand_gestures/pause', Empty)
srvGesturesResume = rospy.ServiceProxy('/hand_gestures/resume', Empty)
srvSkeletonPause = rospy.ServiceProxy('/skeleton_detector/pause', Empty)
srvSkeletonResume = rospy.ServiceProxy('/skeleton_detector/resume', Empty)
srvResetMotors = rospy.ServiceProxy('/reset_motorstop', ResetMotorStop)
 
blockEvent = False

mmui = MMUI.MMUIInterface()

# BUTTON MAPPING #
# button[0]: X
# button[1]: A
# button[2]: B
# button[3]: Y
# button[4]: LB
# button[5]: RB
# button[6]: LT
# button[7]: RT
# button[8]: BACK
# button[9]: START
# button[10]: L_JOYSTICK
# button[11]: R_JOYSTICK


def joyCallback(msg):
    global blockEvent,mmui,srvFacePause,srvFaceResume,srvEmergencyPause,srvEmergencyResume,srvGesturesPause,srvGesturesResume,srvSkeletonPause,srvSkeletonResume,srvResetMotors
   
    if numpy.sum(numpy.asarray(msg.buttons)) < 1.0:
        # BUTTON UP EVENT and no more button is pressed -> unblock event
#        rospy.loginfo('Button up, no more button pressed')
        blockEvent = False
        return
    else:
        if blockEvent == True:
	    # button still pressed, keep blocking event
#            rospy.loginfo('Other button still pressed, block new button click')
            return
   	else:
	    # first button has been pressed. block next event
            blockEvent = True
 
    if msg.buttons[0] == 1:    # X, not used right now
        rospy.loginfo("JOYSTICK: Blue X button pressed. Enable ASR.")     
        mmui.enable_asr()           

    if msg.buttons[1] == 1:    # A
        rospy.loginfo("JOYSTICK: Green A button pressed. Unblock motors.") 
        
        try:
            resp = srvResetMotors()
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % str(e))                     

    if msg.buttons[2] == 1:    # B
        rospy.loginfo("JOYSTICK: Red B button pressed. STOP (C_STOP).")       
        e = Command()
        e.header = Header()
        e.command = "C_STOP"
        e.sessionID = '0'
        pubC.publish(e)

    if msg.buttons[3] == 1:    # Y
        rospy.loginfo("JOYSTICK: Yellow Y button pressed. Disable ASR.")
        mmui.disable_asr()
        
    if msg.buttons[4] == 1:    # L TOP (labeled LB for Left Button)                
        rospy.loginfo("JOYSTICK: Left top button pressed. Manual driving.")

    if msg.buttons[5] == 1:    # R TOP (labeled RB for Right Button)
        rospy.loginfo("JOYSTICK: Right top button pressed. Enable gestures, face and emergency detector and skeleton tracker.")
    
        try:
            resp = srvFaceResume()
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % str(e))
            
        try:
            resp = srvEmergencyResume()
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % str(e))
        
        try:
            resp = srvGesturesResume()
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % str(e))
            
        try:
            resp = srvSkeletonResume()
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % str(e))

        rospy.loginfo("Done.")

    if msg.buttons[6] == 1:    # L BOTTOM (labeled LT for Left Trigger)
        rospy.loginfo("JOYSTICK: Left bottom button pressed. Do nothing.")

    if msg.buttons[7] == 1:    # R BOTTOM (labeled RT for Right Trigger)
        rospy.loginfo("JOYSTICK: Right bottom button pressed. Disable gestures, face and emergency detector and skeleton tracker.")
        
        try:
            resp = srvFacePause()
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % str(e))
            
        try:
            resp = srvEmergencyPause()
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % str(e))
        
        try:
            resp = srvGesturesPause()
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % str(e))
            
        try:
            resp = srvSkeletonPause()
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % str(e))

        rospy.loginfo("Done.")

    if msg.buttons[8] == 1:    # Back
        rospy.loginfo("JOYSTICK: Back button pressed. Master Reset (C_MASTER_RESET)")
        e = Command()
        e.header = Header()
        e.command = "C_MASTER_RESET"
        e.sessionID = '0'
        pubC.publish(e)

    if msg.buttons[9] == 1:    # Start
        rospy.loginfo("JOYSTICK: Start button pressed. EMERGENCY (E_SOSBUTTON).")
        e = Event()
        e.header = Header()
        e.event = "E_SOSBUTTON"
        e.sessionID = '0'
        e.confidence = 1
        pubE.publish(e)

    if msg.buttons[10] == 1:   # Left joystick
        rospy.loginfo("JOYSTICK: Left joystick pressed. Do nothing.")

    if msg.buttons[11] == 1:   # Right joystick
        rospy.loginfo("JOYSTICK: Right joystick pressed. Do nothing.")

def main(args):
    rospy.init_node('Joybutton', anonymous=False)
    
    sub = rospy.Subscriber("/joy", Joy, joyCallback )
    rospy.spin()
    
if __name__ == "__main__":        
    main(sys.argv)

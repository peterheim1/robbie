#!/usr/bin/env python


import rospy
import smach
import smach_ros
import tf
import time
from actionlib import SimpleActionClient
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Float32, String
from festival.srv import *
from robbie.util import *




def main():
    rospy.init_node('robbie_face_monitor')
    #start speech service
    rospy.wait_for_service('speak_text')
    try:
        Speak_text_service = rospy.ServiceProxy('speak_text', FestivalSpeech)
    except rospy.ServiceException, e:
        print "Failed to acquire Festival SpeakText service: %s"%e

    Speak_text_service("  Robbie is on line  ")
    

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Wait_Face', musicplayer(), transitions={'stop':'default', 'phone':'phoneserver', '':'exit'})
        smach.StateMachine.add('Store_face', phone(), transitions={'stop':'default', '':'exit'})
        smach.StateMachine.add('Train_Face', questions(), transitions={'stop':'default', '':'exit'})
    sis = smach_ros.IntrospectionServer('face_recognition', sm, '/SM_ROOT')
    sis.start()
 
    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()

#!/usr/bin/env python

import roslib; roslib.load_manifest('robbie')
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

# define state BatteryState
class BatteryState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['in_progress', 'failed'])
        self.subscriber = rospy.Subscriber('/battery', Float32, self.callback)
        self.counter = 0

    def callback(self, data):
        if data.data < 12.5:
            self.counter = 3

    def execute(self, userdata):
        #rospy.loginfo('Executing state BatteryState waiting for voltage')
        if self.counter < 3:
            #self.counter += 1
            return 'in_progress'
        else:
            #rospy.loginfo('voltage trigger')
            return 'failed'


class SleepState(smach.State):
    """Sleep for a time duration, given either on initialization or via userdata.

     duration: of type rospy Duration or float in seconds. If not given or None,
     duration is read from userdata key 'duration'.

     userdata input duration: of type rospy Duration or float in seconds (not
     registered as input key if given on initialization)
    """
    def __init__(self, duration=36):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['duration'] if duration is None else [])
        self.duration = duration

    def execute(self, userdata):
        duration = userdata.duration if self.duration is None else self.duration
        rospy.loginfo("SleepState sleeping for %d seconds" % duration)
        # sleep in steps to handle state preemption
        SLEEP_STEP = 2 # maximum to sleep per step
        while duration > 0:
            sleeptime = SLEEP_STEP if duration > SLEEP_STEP else duration
            duration -= sleeptime
            rospy.sleep(sleeptime)
            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('SleepState was preempted while sleeping!')
                return 'preempted'
        return 'succeeded'

class Auto_Dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['in_progress'])
        self._Auto_dock_Publisher = rospy.Publisher('auto_dock', String)

    def execute(self, userdata):
        #rospy.loginfo('Executing state Auto_Dock')
        
        self._Auto_dock_Publisher.publish("dock")
        time.sleep(15)
        return 'in_progress'
        




def main():
    rospy.init_node('robbie_battery_monitor')
    #start speech service
    rospy.wait_for_service('speak_text')
    try:
        Speak_text_service = rospy.ServiceProxy('speak_text', FestivalSpeech)
    except rospy.ServiceException, e:
        print "Failed to acquire Festival SpeakText service: %s"%e

    Speak_text_service("  Robbie is on line  ")
    

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted','outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('BatteryState', BatteryState(), 
                               transitions={'in_progress':'BatteryState', 'failed':'STAGEING'})

        _goal = MoveBaseGoal()
        _goal.target_pose.header.stamp = rospy.Time.now()
        _goal.target_pose.header.frame_id = '/map'
        _goal.target_pose.pose.position = Point(x=0.859, y=0.133)
        quat = tf.transformations.quaternion_from_euler(0, 0, -0.121)
        _goal.target_pose.pose.orientation = Quaternion(*quat)
        #_goal.vicinity_range = 0.1
        smach.StateMachine.add("STAGEING",
                          smach_ros.SimpleActionState("move_base",
                                            MoveBaseAction,
                                            goal=_goal),
                                {'succeeded':'Auto_Dock'})
                           
        smach.StateMachine.add('Auto_Dock', Auto_Dock(), 
                               transitions={'in_progress':'BatteryState'})

        smach.StateMachine.add('Sleep', SleepState(), 
                               transitions={'succeeded':'BatteryState'})

    sis = smach_ros.IntrospectionServer('Robbie_behaviour', sm, '/SM_ROOT')
    sis.start()
 
    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()

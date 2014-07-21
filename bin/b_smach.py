#!/usr/bin/env python
'''

http://wiki.ros.org/smach/Tutorials/State%20Machine%20Preemption%20with%20MonitorState
'''

import rospy
import smach
import smach_ros
import tf
from actionlib import SimpleActionClient
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Float32, String
from festival.srv import *

class setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['setup_done'])
    def execute(self, userdata):
        rospy.sleep(3.5)
        return 'setup_done'

class navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigation_succeeded', 'preempted'])
    def execute(self, userdata):
        for idx in range(5):
            if self.preempt_requested():
                print "state navigation is being preempted!!!"
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1.0)
        return 'navigation_succeeded'

class auto_dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self._Auto_dock_Publisher = rospy.Publisher('auto_dock', String)

    def execute(self, userdata):
        #rospy.loginfo('Executing state Auto_Dock')
        
        self._Auto_dock_Publisher.publish("dock")
        #Speak_text_service("  auto dock in progress")
        rospy.sleep(30)#30 SEC
        return 'succeeded'

def staging():
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = '/map'
    goal.target_pose.pose.position = Point(x=1, y=0)
    quat = tf.transformations.quaternion_from_euler(0, 0, -0.121)
    goal.target_pose.pose.orientation = Quaternion(*quat)
    return goal

def child_term_cb(outcome_map):
    if outcome_map['NAVIGATION'] == 'navigation_succeeded':
        return True
    elif outcome_map['BATTERY_LEVEL'] == 'invalid':
        return True
    elif outcome_map['FACE_RECOGNITION'] == 'invalid':
        return True
    else:
        return False

def out_cb(outcome_map):
    if outcome_map['BATTERY_LEVEL'] == 'invalid':
        return 'low_battery'
    elif outcome_map['NAVIGATION'] == 'navigation_succeeded':
        return 'navigation_done'
    elif outcome_map['FACE_RECOGNITION'] == 'invalid':
        return 'succeeded'
    else:
        return 'low_battery'

def monitor_cb(ud, msg):
    #rospy.logwarn(msg.data)
    if msg.data < 11.8:
    #Speak_text_service("  Robbie is hungry")
        return False    

def monitor_cb1(ud, msg):
    rospy.logwarn(msg.data)
    #if msg.data < 11.8:
    #Speak_text_service("  Robbie is hungry")
    return False    
def main():
    rospy.init_node("preemption_example")

    navigation_concurrence = smach.Concurrence(outcomes=['navigation_done', 'low_battery','succeeded'],
                                        default_outcome='navigation_done',
                                        child_termination_cb=child_term_cb,
                                        outcome_cb=out_cb)

    with navigation_concurrence:
        smach.Concurrence.add('NAVIGATION', navigation())
        smach.Concurrence.add('BATTERY_LEVEL', smach_ros.MonitorState("/battery", Float32, monitor_cb))
        smach.Concurrence.add('FACE_RECOGNITION', smach_ros.MonitorState("/face", String, monitor_cb1))

    sm = smach.StateMachine(outcomes=['DONE', 'succeeded','aborted','preempted'])
    with sm:
        smach.StateMachine.add('SETUP', setup(), transitions={'setup_done':'TASKS'})
        smach.StateMachine.add('TASKS', navigation_concurrence, transitions={'navigation_done':'BAR', 'low_battery':'HUNGRY'}) 
        smach.StateMachine.add('BAR', navigation(), transitions={'navigation_succeeded':'TASKS', 'preempted':'SETUP'})
        smach.StateMachine.add("HUNGRY", smach_ros.SimpleActionState("move_base", MoveBaseAction, goal= staging()),       {'succeeded':'AUTO_DOCK'})
        smach.StateMachine.add('AUTO_DOCK',auto_dock(), transitions={'succeeded':'TASKS'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()

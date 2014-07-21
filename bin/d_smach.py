#!/usr/bin/env python
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

class hungry(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hungry_succeeded'])
    def execute(self, userdata):
        rospy.sleep(3.0)
        return 'hungry_succeeded'

class auto_dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self._Auto_dock_Publisher = rospy.Publisher('auto_dock', String)

    def execute(self, userdata):
        #rospy.loginfo('Executing state Auto_Dock')
        
        self._Auto_dock_Publisher.publish("dock")
        #Speak_text_service("  auto dock in progress")
        rospy.sleep(600)#10min
        return 'succeeded'
def staging():
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = '/map'
    goal.target_pose.pose.position = Point(x=1.5, y=1.121)
    quat = tf.transformations.quaternion_from_euler(0, 0, -0.027)
    goal.target_pose.pose.orientation = Quaternion(*quat)
    return goal

def staging1():
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = '/map'
    goal.target_pose.pose.position = Point(x=2.279, y=2.651)
    quat = tf.transformations.quaternion_from_euler(0, 0,  0.33)
    goal.target_pose.pose.orientation = Quaternion(*quat)
    return goal

def fridge():
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = '/map'
    goal.target_pose.pose.position = Point(x=0.859, y=0.133)
    quat = tf.transformations.quaternion_from_euler(0, 0, -0.121)
    goal.target_pose.pose.orientation = Quaternion(*quat)
    return goal

def monitor_cb(ud, msg):
    #rospy.logwarn(msg.data)
    if msg.data < 11.8:
        Speak_text_service("  Robbie is hungry")
        return False
    else:
        return True
    #return False

def main():
    rospy.init_node("monitor_example")
    rospy.wait_for_service('speak_text')
    try:
        Speak_text_service = rospy.ServiceProxy('speak_text', FestivalSpeech)
    except rospy.ServiceException, e:
        print "Failed to acquire Festival SpeakText service: %s"%e

    Speak_text_service("  Robbie battery monitor is on line")


    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    with sm:
        smach.StateMachine.add('healthy', smach_ros.MonitorState("/battery", Float32, monitor_cb), transitions={'invalid':'HUNGRY', 'valid':'healthy', 'preempted':'healthy'})
        smach.StateMachine.add("HUNGRY", smach_ros.SimpleActionState("move_base", MoveBaseAction, goal= staging()),       {'succeeded':'AUTO_DOCK'})
        #smach.StateMachine.add('HUNGRY',hungry(), transitions={'hungry_succeeded':'AUTO_DOCK'})#change to action state
        smach.StateMachine.add('AUTO_DOCK',auto_dock(), transitions={'succeeded':'healthy'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()


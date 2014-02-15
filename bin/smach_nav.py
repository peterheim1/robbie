#!/usr/bin/env python

import roslib; roslib.load_manifest('robbie')
import rospy
import smach
import smach_ros
import tf
from actionlib import SimpleActionClient
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
#from robbie.msg import *
from std_msgs.msg import String
from festival.srv import *

def get_current_robot_position(frame='/map'):
    '''Returns a (x,y,yaw) tuple. Frame defaults to /map.'''
    try:
        trans,rot = get_transform_listener().lookupTransform(frame, '/base_link', rospy.Time(0))
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(rot)
        return trans[0], trans[1], yaw
    except (tf.LookupException, tf.ConnectivityException) as e:
        print e
        return 0,0,0

def move_to(frame='/map', x=0, y=0, yaw=0):
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = frame
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation = Quaternion(*quat)
    goal.target_pose.pose.position = Point(x, y, 0)
    #return goal
    client = smach_ros.SimpleActionClient("move_base", MoveBaseAction)
    client.send_goal(goal)
    #client.wait_for_result()


# main
def main():
    rospy.init_node('smach_example_state_machine')
    #client = SimpleActionClient("robbie_base_action", RobbieBaseAction)
    #start speech service
    rospy.wait_for_service('speak_text')
    try:
        Speak_text_service = rospy.ServiceProxy('speak_text', FestivalSpeech)
    except rospy.ServiceException, e:
        print "Failed to acquire Festival SpeakText service: %s"%e

    Speak_text_service("  Robbie is on line   Navigation Test 2 way points")


    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    # Open the container
    with sm:

        _goal = MoveBaseGoal()
        _goal.target_pose.header.stamp = rospy.Time.now()
        _goal.target_pose.header.frame_id = '/map'
        _goal.target_pose.pose.position = Point(x=2.279, y=2.651)
        quat = tf.transformations.quaternion_from_euler(0, 0,  0.33)
        _goal.target_pose.pose.orientation = Quaternion(*quat)
        
        smach.StateMachine.add("STAGE_1",
                          smach_ros.SimpleActionState("move_base",
                                            MoveBaseAction,
                                            goal=_goal),
                                {'succeeded':'STAGE_2'})

        #smach.StateMachine.add("talk1",
                          #Speak_text_service("  Robbie is at way point one  "),
                                #{'succeeded':'STAGE_2'})
        
        _goal = MoveBaseGoal()
        _goal.target_pose.header.stamp = rospy.Time.now()
        _goal.target_pose.header.frame_id = '/map'
        _goal.target_pose.pose.position = Point(x=3, y=0.0)
        quat = tf.transformations.quaternion_from_euler(0, 0,  -1.52)
        _goal.target_pose.pose.orientation = Quaternion(*quat)
        
        smach.StateMachine.add("STAGE_2",
                          smach_ros.SimpleActionState("move_base",
                                            MoveBaseAction,
                                            goal=_goal),
                                {'succeeded':'STAGE_3'})
        _goal = MoveBaseGoal()
        _goal.target_pose.header.stamp = rospy.Time.now()
        _goal.target_pose.header.frame_id = '/map'
        _goal.target_pose.pose.position = Point(x=0.859, y=0.133)
        quat = tf.transformations.quaternion_from_euler(0, 0, -0.121)
        _goal.target_pose.pose.orientation = Quaternion(*quat)
        #_goal.vicinity_range = 0.1
        smach.StateMachine.add("STAGE_3",
                          smach_ros.SimpleActionState("move_base",
                                            MoveBaseAction,
                                            goal=_goal))
        
        
    
    # Execute SMACH plan
    outcome = sm.execute()

    


if __name__ == '__main__':
    main()

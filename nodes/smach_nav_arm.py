#!/usr/bin/env python

import roslib; roslib.load_manifest('robbie')
import rospy
import smach
import smach_ros
#import tf
from actionlib import SimpleActionClient
#from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
#from robbie.msg import *
from std_msgs.msg import String
from festival.srv import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
#from pr2_controllers_msgs.msg import *



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
        action_goal = FollowJointTrajectoryActionGoal()
        action_goal.goal_id.id = "wave"
        action_goal.goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        action_goal.goal.trajectory.joint_names = ['right_arm_tilt_joint_controller', 'right_arm_lift_joint_controller', 'right_arm_rotate_joint_controller',    'right_arm_elbow_joint_controller', 'right_arm_wrist_tilt_joint_controller']
        action_goal.goal.trajectory.points.append(
            JointTrajectoryPoint(positions = [0.1, 0.2, 1.7, 0.5, -1.06], 
                         velocities = [0, 0, 0, 0, 0],))
   
        smach.StateMachine.add("STAGE_1",
                          smach_ros.SimpleActionState("/arm_controller/follow_joint_trajectory",
                                            FollowJointTrajectoryAction,
                                            action_goal.goal))
        
        
    
    # Execute SMACH plan
    outcome = sm.execute()

    


if __name__ == '__main__':
    main()

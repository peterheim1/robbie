#!/usr/bin/env python
"""

"""

import roslib; roslib.load_manifest('robbie')
import rospy
import smach
import smach_ros

from actionlib import *
from actionlib.msg import *
from robbie.msg import *
from geometry_msgs.msg import *




    


def main():
    rospy.init_node('smach_example_actionlib')

    # Start an action server
    #server = TestServer('Robbie_Base_action')

    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'failed'])

    # Open the container
    with sm0:
        # Add states to the container
        def goal_tg(frame_id, position,orientation, vicinity=0.0):
            goal = RobbieBaseGoal()
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = frame_id
            goal.target_pose.pose.position = position
            goal.target_pose.pose.orientation = orientation
            goal.vicinity_range = 0.0
            return goal

        # Move to the fish tank
        
        smach.StateMachine.add('GOAL_FISH_TANK',
                smach_ros.SimpleActionState('robbie_base_action', RobbieBaseAction,
                    goal = goal_tg('/map',Point(x=2, y=1), 1, 0.0)), 
                {'succeeded':'GOAL_OPEN_DOOR'})

        # This will be open the door
        smach.StateMachine.add('GOAL_OPEN_DOOR',
                smach_ros.SimpleActionState('Robbie_base_action', RobbieBaseAction,
                    goal = TestGoal(goal=1)),
                {'aborted':'failed'})

        # For more examples on how to set goals and process results, see 
        # executive_python/smach/tests/smach_actionlib.py

    
     # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm0, '/SM_ROOT')
    sis.start()

    #rospy.signal_shutdown('All done.')
     # Execute the state machine
    outcome = sm0.execute()

    # Wait for ctrl-c to stop the application
    #rospy.spin()
    #sis.stop()


if __name__ == '__main__':
    main()

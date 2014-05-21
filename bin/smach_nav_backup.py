#!/usr/bin/env python

import roslib; roslib.load_manifest('robbie')
import rospy
import smach
import smach_ros
from actionlib import SimpleActionClient
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from robbie.msg import *

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'
        



# main
def main():
    rospy.init_node('smach_example_state_machine')
    client = SimpleActionClient("robbie_base_action", RobbieBaseAction)

    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    # Open the container
    with sm:
        with sm:
            _goal = RobbieBaseGoal()
            _goal.target_pose.header.stamp = rospy.Time.now()
            _goal.target_pose.header.frame_id = '/map'
            _goal.target_pose.pose.position = Point(x=2.279, y=2.651)
            _goal.target_pose.pose.orientation = Quaternion(z=-0.673918593079,w=0.73880561036)
            #_goal.vicinity_range = 0.1
            goal=_goal
            smach.StateMachine.add("STAGE_1",
                          smach_ros.SimpleActionState('robbie_base_action',
                                            RobbieBaseAction,
                                            goal=_goal))
                          #client.send_goal(goal))
                          #transitions={'succeeded':'BAR'})
            #smach.smach.StateMachine.add('BAR', Bar(), 
                                   #transitions={'outcome2':'FOO'})
    #sis = smach.IntrospectionServer('server_name', sm0, '/SM_ROOT')
    #sis.start()
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()

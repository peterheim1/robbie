#!/usr/bin/env python


import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint


#if name == '__main__':
if __name__=='__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    right_arm = MoveGroupCommander("right_arm")
    rospy.sleep(1)

  
    # move to a random target
    #right_arm.set_named_target("start1")
    #right_arm.go()
    #rospy.sleep(1)

    #right_arm.set_random_target()
    #right_arm.go()
    #rospy.sleep(1)

   
    right_arm.set_position_target([.75,-0.3, 0.9])
    right_arm.go()
    rospy.sleep(1)


    

    rospy.spin()
    roscpp_shutdown()

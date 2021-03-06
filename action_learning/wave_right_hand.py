#! /usr/bin/python
import roslib
roslib.load_manifest('robbie')

import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from math import pi
from dynamixel_controllers.srv import TorqueEnable
from control_msgs.msg import *
import os

rospy.init_node('wave_right_hand', anonymous=True)
side = rospy.get_param("~side", "right")

dynamixel_namespace = '/dynamixel_controller'

client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory',
                                      FollowJointTrajectoryAction)


dynamixels = rospy.get_param('right_arm/arm_controller', '')#['right_arm_tilt', 'right_arm_lift', 'right_arm_rotate', 'right_arm_elbow', 'right_arm_wrist_tilt']#rospy.get_param(dynamixel_namespace+'/dynamixels', dict())
traj_speed = rospy.get_param('~traj_speed', 1.0)
traj_speed = 1.0 / traj_speed

#servo_torque_enable = list()

#for name in sorted(dynamixels):
    #rospy.wait_for_service(dynamixel_namespace+'/'+name+'_controller/torque_enable')  
    #servo_torque_enable.append(rospy.ServiceProxy(dynamixel_namespace+'/'+name+'_controller/torque_enable', TorqueEnable))

client.wait_for_server()

action_goal = FollowJointTrajectoryActionGoal()
action_goal.goal_id.id = "wave"
action_goal.goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
action_goal.goal.trajectory.joint_names = ['right_arm_tilt_joint_controller', 'right_arm_lift_joint_controller', 'right_arm_rotate_joint_controller', 'right_arm_elbow_joint_controller', 'right_arm_wrist_tilt_joint_controller']

action_goal.goal.trajectory.points.append(
    JointTrajectoryPoint(positions = [0.1, 0.2, 1.7, 0.5, -1.06], 
                         velocities = [0, 0, 0, 0, 0],
                         time_from_start = rospy.Duration(0.0 * traj_speed)))



client.send_goal(action_goal.goal)
client.wait_for_result()

if client.get_state() == GoalStatus.SUCCEEDED:
    print "Success"

# Relax all servos to give them a rest.
#for torque_enable in servo_torque_enable:
    #torque_enable(False)

os._exit(0)

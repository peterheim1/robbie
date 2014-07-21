#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown, MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    rospy.init_node('moveit_py_place', anonymous=True)
    #right_arm.set_planner_id("KPIECEkConfigDefault");
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    #group = MoveGroupCommander("head")
    right_arm = MoveGroupCommander("right_arm")
    #right_arm.set_planner_id("KPIECEkConfigDefault");
    rospy.logwarn("cleaning world")
    GRIPPER_FRAME = 'gripper_bracket_f2'
    #scene.remove_world_object("table")
    scene.remove_world_object("part")
    scene.remove_attached_object(GRIPPER_FRAME, "part")
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    p.pose.position.x = 0.67
    p.pose.position.y = -0.
    p.pose.position.z = 0.75
    scene.add_box("part", p, (0.07, 0.01, 0.2))

    # move to a random target
    #group.set_named_target("ahead")
    #group.go()
    #rospy.sleep(1)

    result = False
    n_attempts = 0
       
    # repeat until will succeed
    while result == False:
        result = robot.right_arm.pick("part")      
        n_attempts += 1
        print "Attempts pickup: ", n_attempts
        rospy.sleep(0.2)
    
    #robot.right_arm.pick("part")
    #right_arm.go()
    rospy.sleep(5)


if __name__=='__main__':
    main()

  

  

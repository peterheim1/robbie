#! /usr/bin/python
import roslib

import rospy
import actionlib
from actionlib_msgs.msg import *
from robbie.srv import *
rospy.init_node('recharge_auto_dock', anonymous=True)

client = actionlib.SimpleActionClient('/recharge_action', AutoDock)
client.wait_for_server()

g = PointHeadGoal()
g.target.header.frame_id = 'head_pan_link'
g.target.point.x = 1.0
g.target.point.y = 0.0
g.target.point.z = 0.3
g.min_duration = rospy.Duration(1.0)

client.send_goal(g)
client.wait_for_result()

if client.get_state() == GoalStatus.SUCCEEDED:
    print "Succeeded"
else:
    print "Failed"

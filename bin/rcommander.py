#!/usr/bin/python
import roslib; roslib.load_manifest('robbie')
import rcommander.rcommander as rc
import rospy

rospy.init_node('my_rcommander', anonymous=True)
rc.run_rcommander(['default'])


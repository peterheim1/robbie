#!/usr/bin/env python

"""
    head_track_node.py - Version 1.0 2010-12-28
    
    Move the head to track a target given by (x,y) coordinates
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import roslib; roslib.load_manifest('robbie')
import rospy
from sensor_msgs.msg import RegionOfInterest, CameraInfo
from dynamixel_msgs.msg import JointState
from math import radians

class head_track():
    def __init__(self):
        rospy.init_node("head_track")
        rospy.on_shutdown(self.shutdown)
        
        """ Publish the movement commands on the /cmd_joints topic using the 
            JointState message type. """
        #self.head_pub = rospy.Publisher("/cmd_joints", JointState)
        self.tilt_pub = rospy.Publisher("/head_tilt_joint/command", JointState)
        self.pan_pub = rospy.Publisher("/head_pan_joint/command", JointState)
        
        self.rate = rospy.get_param("~rate", 10)
        
        """ The pan/tilt thresholds indicate how many pixels the ROI needs to be off-center
            before we make a movement. """
        self.pan_threshold = int(rospy.get_param("~pan_threshold", 5))
        self.tilt_threshold = int(rospy.get_param("~tilt_threshold", 5))
        
        """ The k_pan and k_tilt parameter determine how responsive the servo movements are.
            If these are set too high, oscillation can result. """
        self.k_pan = rospy.get_param("~k_pan", 7.0)
        self.k_tilt = rospy.get_param("~k_tilt", 5.0)
        
        self.max_pan = rospy.get_param("~max_pan", radians(145))
        self.min_pan = rospy.get_param("~min_pan", radians(-145))
        self.max_tilt = rospy.get_param("~max_tilt", radians(90))
        self.min_tilt = rospy.get_param("~min_tilt", radians(-90))
        
        r = rospy.Rate(self.rate) 
        
        self.head_cmd_pan = JointState()
        #self.joints = ["head_pan_joint", "]
        self.head_cmd_pan.name = "head_pan_joint"
        self.head_cmd_pan.goal_pos = 0
        self.head_cmd_pan.velocity = 0
        self.head_cmd_pan.header.stamp = rospy.Time.now()
        self.head_cmd_pan.header.frame_id = 'head_pan_joint'

        self.head_cmd_tilt = JointState()
        #self.joints = ["head_pan_joint", "]
        self.head_cmd_tilt.name = "head_tilt_joint"
        self.head_cmd_tilt.goal_pos = 0
        self.head_cmd_tilt.velocity = 0
        self.head_cmd_tilt.header.stamp = rospy.Time.now()
        self.head_cmd_tilt.header.frame_id = 'head_tilt_joint'
    
        """ Center the head and pan servos at the start. """
        for i in range(3):
            #self.head_pub.publish(self.head_cmd)
            self.pan_pub.publish(self.head_cmd_pan)
            self.tilt_pub.publish(self.head_cmd_tilt)
            rospy.sleep(1)
        
        self.tracking_seq = 0
        self.last_tracking_seq = -1
        
        rospy.Subscriber('roi', RegionOfInterest, self.setPanTiltSpeeds)
        rospy.Subscriber('/camera/camera_info', CameraInfo, self.getCameraInfo)
        
        while not rospy.is_shutdown():
            """ Publish the pan/tilt movement commands. """
            self.head_cmd_pan.header.stamp = rospy.Time.now()
            self.head_cmd_tilt.header.stamp = rospy.Time.now()
            self.head_cmd_pan.header.frame_id = 'head_pan_joint'
            self.head_cmd_tilt.header.frame_id = 'head_tilt_joint'
            if self.last_tracking_seq == self.tracking_seq:
                self.head_cmd_pan.velocity = 0
                self.head_cmd_tilt.velocity = 0
            else:
                self.last_tracking_seq = self.tracking_seq
            self.head_pub_pan.publish(self.head_cmd_pan)
            self.head_pub_tit.publish(self.head_cmd_tilt)
            r.sleep()
    
    def setPanTiltSpeeds(self, msg):
        """ When OpenCV loses the ROI, the message stops updating.  Use this counter to
            determine when it stops. """
        self.tracking_seq += 1
        
        """ Check to see if we have lost the ROI. """
        if msg.width == 0 or msg.height == 0 or msg.width > self.image_width / 2 or \
                msg.height > self.image_height / 2:
            self.head_cmd_pan.velocity = 0
            self.head_cmd_tilt.velocity = 0
            return

        """ Compute the center of the ROI """
        COG_x = msg.x_offset + msg.width / 2 - self.image_width / 2
        COG_y = msg.y_offset + msg.height / 2 - self.image_height / 2
          
        """ Pan the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_x) > self.pan_threshold:
            """ Set the pan speed proportion to the displacement of the horizontal displacement
                of the target. """
            self.head_cmd_pan.velocity = self.k_pan * abs(COG_x) / float(self.image_width)#pub on pan topic
               
            """ Set the target position to one of the min or max positions--we'll never
                get there since we are tracking using speed. """
            if COG_x > 0:
                self.head_cmd_pan.goal_pos = self.min_pan
            else:
                self.head_cmd_pan.goal_pos = self.max_pan
        else:
            self.head_cmd_pan.velocity = 0.0001#pub on pan topic
        
        """ Tilt the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_y) > self.tilt_threshold:
            """ Set the tilt speed proportion to the displacement of the vertical displacement
                of the target. """
            self.head_cmd_tilt.velocity = self.k_tilt * abs(COG_y) / float(self.image_height)
            
            """ Set the target position to one of the min or max positions--we'll never
                get there since we are tracking using speed. """
            if COG_y < 0:
                self.head_cmd_tilt.goal_pos = self.min_tilt
            else:
                self.head_cmd_tilt.goal_pos = self.max_tilt
        else:
            self.head_cmd_tilt.velocity = 0.0001
            
    def getCameraInfo(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        
    def shutdown(self):
        rospy.loginfo("Shutting down head tracking node...")         
                   
if __name__ == '__main__':
    try:
        head_track()
    except rospy.ROSInterruptException:
        rospy.loginfo("Head tracking node is shut down.")





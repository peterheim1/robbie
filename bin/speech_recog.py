#!/usr/bin/env python
# -*- coding: utf-8 -*-
#part of the speech recognition nodes of chippu

#reads a string from stdin( provided by speechrecog_init.py), strips it out of unwanted characters and informations,
#and publishes the recognised speech as a message named 'speech'
#
#written by achuwilson
#achu_wilson@rediffmail.com
#achuwilson.wordpress.com


import rospy
from std_msgs.msg import String
import os
import subprocess
import sys

def Control(file_object):
       pub = rospy.Publisher('speech', String)
       rospy.init_node('talker')
       startstring = 'sentence1: <s> '
       endstring = ' </s>'

       while not rospy.is_shutdown():
	 line = file_object.readline()
	 if not line:
				break
	   
         end = len(line)-6
         if line[0:9] == 'sentence1':
	    word = line[15:end]
	    print word
            rospy.loginfo(word)
            pub.publish(String(word))
         rospy.sleep(0.001)
if __name__ == '__main__':
       try:
           Control(sys.stdin)
       except rospy.ROSInterruptException: pass
       except KeyboardInterrupt:
		sys.exit(1)
   
   

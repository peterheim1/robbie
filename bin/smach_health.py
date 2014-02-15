#!/usr/bin/env python
import roslib; roslib.load_manifest('robbie')
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32
import time
import threading

class WaitForTwo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'in_progress', 'failed'])

        self.mutex = threading.Lock()
        self.two_received = False

        self.subscriber = rospy.Subscriber('/battery', Float32, self.callback)

    def callback(self, data):
        self.mutex.acquire()
        if data.data < 13.8:
            self.two_received = True
        self.mutex.release()

    def execute(self):
        #wait for a maximum of 30 seconds 
        for i in range(0, 3000):
            self.mutex.acquire()
            if self.two_received:
                #ok we received 2
                return 'success'

            self.mutex.release()

            time.sleep(.5)
            #still waiting
            return 'in_progress'
        #we didn't get 2 in the 30 sec
        return 'failed'

#!/usr/bin/env python
'''




'''

import roslib; roslib.load_manifest('robbie')
import rospy
import time
import actionlib
from face_recognition.msg import *
from std_msgs.msg import String
from festival.srv import *
from datetime import datetime, timedelta
from time import localtime, strftime


class Greeting():
    def __init__(self):
        #need to use action client
        #self.pub = rospy.Publisher("/fr_order", FRClientGoal, self._recog_once)
        self.client = actionlib.SimpleActionClient('face_recognition', face_recognition.msg.FaceRecognitionAction)
        self._NamePublisher = rospy.Publisher("name", String)
        rospy.Subscriber("/speech_text", String, self.speech_callback)
        rospy.Subscriber("/face_recognition/feedback", FaceRecognitionActionFeedback, self._feedback)
        #self.client.wait_for_server()
        
        #define afternoon and morning
        self.noon = strftime("%p:", localtime())
        if self.noon == "AM:":
            self.noon1 = "Goood Morning  "
        else:
            self.noon1 ="Good After Noon   "
        self.local = strftime("%H:%M:", localtime())
        #local time
        self.local = strftime("%H:%M:", localtime())


        #start speech service
        rospy.wait_for_service('speak_text')
        try:
            self.speak_text_service = rospy.ServiceProxy('speak_text', FestivalSpeech)
        except rospy.ServiceException, e:
            print "Failed to acquire Festival SpeakText service: %s"%e

        self.speak_text_service(self.noon1 + "  Robbie is on line" + " the time is   " + self.local)

    def speech_callback(self,text):
        self.hold = text.data
        if self.hold == "good morning":
            goal = face_recognition.msg.FaceRecognitionGoal(order_id=1, order_argument="none")
            self.client.send_goal(goal)
            time.sleep(0.2)
            #self.hold = "good morning tim"
            goal = face_recognition.msg.FaceRecognitionGoal(order_id=0, order_argument="none")
            self.client.send_goal(goal)
        elif self.hold == "hello":
            self.hold = "hello tim"
        rospy.logwarn(str(self.hold))

    def _feedback(self, text):
            
            self.name = text.feedback.names 
            self.confidence = text.feedback.confidence
            #rospy.loginfo(self.confidence[0])
            if self.confidence[0] > 0.8:
                self.name1 = (self.name[0])
                rospy.loginfo(self.name1)
                self.speak_text_service("good morning  " + " " + str(self.name1))
            else:
                self.name1 = "unknown"
                self.speak_text_service("Hello my name is Robbie   what is your name please ")
            self._NamePublisher.publish(self.name1)

    def Start(self):
	rospy.logdebug("Starting")


    def Stop(self):
	rospy.logdebug("Stopping")
        self.speak_text_service("Robbie's brain is going to sleep")

   
        

if __name__ == '__main__':
    try:
        st = Greeting()
        rospy.init_node('Greeting_node')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    

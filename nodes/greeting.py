#!/usr/bin/env python
'''




'''

import roslib; roslib.load_manifest('robbie')
import rospy
import time
import re
import actionlib
from face_recognition.msg import *
from std_msgs.msg import String
from festival.srv import *
from datetime import datetime, timedelta
from time import localtime, strftime
#from nltk_interpret.srv import *
#from robbie.forex import *
import forex


class Greeting():
    def __init__(self):
        
        self.client = actionlib.SimpleActionClient('face_recognition', face_recognition.msg.FaceRecognitionAction)
        self._NamePublisher = rospy.Publisher("name", String)
        rospy.Subscriber("/speech_text", String, self.speech_callback)
        rospy.Subscriber("/face_recognition/feedback", FaceRecognitionActionFeedback, self._feedback)
        rospy.Subscriber("/nltk_interpret", String, self.nltk_callback)
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
            self.hold = forex.ForeX()
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
                self.new_name = self.person
                self._add_face(self.new_name)
            self._NamePublisher.publish(self.name1)

    def _add_face(self, name):
        goal = face_recognition.msg.FaceRecognitionGoal(order_id=2, order_argument=name)
        self.client.send_goal(goal)
        self.speak_text_service(name + " " +" plase wait while I add your name")
        time.sleep(1)
        self._train_database()

    def _train_database(self):
        goal = face_recognition.msg.FaceRecognitionGoal(order_id=3, order_argument="none")
        self.client.send_goal(goal)
        time.sleep(0.2)
        rospy.loginfo("training database")

    def nltk_callback(self,text):
            split = text.data.split(':')
            #print text.data
            self.command = split[0]
            self.quest = split[1]
            self.size = split[2]
            self.color = split[3]
            self.shape = split[4]
            self.topic = split[5]
            self.person = split[6]
            self.major = split[7]
            self.food = split[8]
            #rospy.loginfo(self.person)

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

    

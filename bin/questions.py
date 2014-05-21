#!/usr/bin/env python
#
#Copyright (C) 2012-2013 Thecorpora Inc.
#
#This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
#
#This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
#You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.


import rospy
from std_msgs.msg import String
from festival.srv import *

import random
import sys
import os

#Load plugins directory
path = '/home/robbie/catkin_hydro/src/robbie'#packages.get_pkg_dir("robbie")
sys.path.append(path+"/plugins")
global dialogue
global chatPublisher
speak_text_service = rospy.ServiceProxy('speak_text', FestivalSpeech)
chatPublisher = rospy.Publisher("talk_chat", String)

def speak_this(text):
    global speak_text_service
    speak_text_service(text)

def listen_callback(data):
    #global face_detected  
    global dialogue
    global chatPublisher
    
    text=""
    sentence = data.data
    rospy.loginfo("Listened: |"+sentence+"|")
   
    #if not face_detected:
       #rospy.loginfo("Ignoring last sentece because face was not detected")
       #return

    if sentence in dialogue:
        output = dialogue[sentence]
        choice=random.choice(output)
        
        if choice[0]=="$":
            choice=choice.replace("$","")
            choice=choice.lower()
            for plug in plugins:
                try:
                   text=getattr(plug,choice)(sentence)
                   break
                except AttributeError:
                    rospy.loginfo("Attibute "+choice +" could not be found:"+ str(dir(plug)))
        else:
            text=choice
        rospy.loginfo(text)
        speak_this(text)
    else:
        rospy.loginfo("off to chat    " +sentence)
        chatPublisher.publish(sentence)
        #rospy.Publisher("talk_chat", String ,output)

#def face_callback(data):
    #global face_detected
    #face_detected = data.face_detected	


def read_dialogues(filename):
    global dialogue
    dialogue = {}

    f = open(filename)    
    for line in f.readlines():
        try:
            line = line.replace("\n","")
            parts = line.split(">>>")

            dialogue_input = parts[0]
            dialogue_output = parts[1].upper().strip()
        
            # we check wheter the input line alreayd exists, if so, we add to its own list
            if dialogue_input in dialogue:                
                dialogue[dialogue_input].append(dialogue_output)
            else:
                #dialogue_input does not exist
                dialogue[dialogue_input] = [dialogue_output]
        except Exception:
            pass        

    f.close()

def set_language():

    

    global subscribe
    try:
        subscribe.unregister()
        rospy.loginfo("Unregistered from previous language")
    except NameError:
        rospy.loginfo("First language set")


    path = '/home/robbie/catkin_hydro/src/robbie'#roslib.packages.get_pkg_dir("qbo_questions")
    filename = path+'/config/dialogues_en'
    print "Dialogue filename loaded: "+filename
    read_dialogues(filename)
    #subscribe=rospy.Subscriber("/listen/"+lang+"_questions", Listened, listen_callback)

def loadPlugins():
    global plugins
    plugins=[]
    path = '/home/robbie/catkin_hydro/src/robbie'#roslib.packages.get_pkg_dir("robbie")
    for f in os.listdir(path+"/plugins/"):
        moduleName, ext = os.path.splitext(f) 
        if ext == '.py' and moduleName!="__init__":
            plugins.append(__import__(moduleName))


def main():
    global client_speak
    global chatPublisher
    

    #Init ROS
    rospy.init_node('robbie_questions')
    

    #Init variable to know if somebody is in front of qbo
    #face_detected = False

    #Load plugins
    loadPlugins()

    
    set_language()
    print "Dialog => "+str(dialogue)

    rospy.loginfo("Starting questions node")

    #start speech service
    rospy.wait_for_service('speak_text')
    try:
        speak_text_service = rospy.ServiceProxy('speak_text', FestivalSpeech)
    except rospy.ServiceException, e:
        print "Failed to acquire Festival SpeakText service: %s"%e
    #speak_text_service("  Robbie is on line")
    #listen for chatter
    #rospy.Subscriber("/system_lang", String, system_language)
    rospy.Subscriber("/quest_talk", String, listen_callback)
    chatPublisher = rospy.Publisher("talk_chat", String)

    #For face view
    #rospy.Subscriber("/qbo_face_tracking/face_pos_and_dist", FacePosAndDist, face_callback)


    rospy.spin()


if __name__ == '__main__':
    main()


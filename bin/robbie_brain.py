#!/usr/bin/env python
# coding: utf-8
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2011 Thecorpora, S.L.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
# MA 02110-1301, USA.
#


import rospy


from std_msgs.msg import String

import smach
import smach_ros
import signal
import subprocess
import time
from festival.srv import *

def run_process(command = ""):

    if command != "":
        return subprocess.Popen(command.split())
    else:
        return -1

def run_all_process(all_commands):
    proc=[]
    for command in all_commands:
        proc.append(subprocess.Popen(command.split()))
    return proc

def kill_all_process(processes):
    for process in processes:
        process.send_signal(signal.SIGINT)


def speak_this(text):
    global speak_text_service
    #speak_text_service(str(text))
    speak_text_service(text)

####################

class CommonRobbieState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["exit"])
        self.state="none"
        self.input_values={"STOP STATE MACHINE":"exit"}
        self.next_state=""
        self.launchers=[]
        self.subscribe = None

    def speech_callback(self, data):
        sentence = data.data
        #rospy.loginfo("Listened: |"+sentence+"|")
        speak_text_service(sentence)
        

        if self.state=="Default" and sentence == "HALT YOU ARE MOVE":
                
                if robot_model.random_move:
                        run_process("rosnode kill /qbo_random_move")
                        robot_model.random_move = False
                
                rospy.set_param("/qbo_face_following/move_base", False)
                rospy.follow_face = False
                speak_this("OK. I STOPPED MOVING")
                return
 
        

        try:
            self.next_state=self.input_values[data.msg]
            
            #self.next_state=self.input_values[lang_label]
        except:
            rospy.loginfo("Sentence not found")
###########################
#Define default state
class default(CommonRobbieState):
    def __init__(self):
        
        
        smach.State.__init__(self, outcomes=['mplayer','phone','questions','webi','battery',''])
        self.state="Default"

        self.input_values={"RUN MUSIC PLAYER":"mplayer", "RUN PHONE SERVICES":"phone","RUN WEB INTERFACE":"webi","LAUNCH CHAT MODE":"questions"}
        self.launchers=["roslaunch qbo_brain default_state.launch"]
        self.launchers=[]
        
    def execute(self, userdata):
        
        rospy.loginfo('Executing: State '+self.state)
        self.next_state=""
        pids=run_all_process(self.launchers)
        
        

#Check if robbie_listen is down
        rosnode_list = runCmdOutput("rosnode list")
        if rosnode_list.find("/robbie_listen") == -1:
             run_process("rosnode kill /robbie_listen")
             time.sleep(2)
             run_process("roslaunch robbie a_voice_rec.launch")

        #Subscribe to topics
        #Listeners
        
        self.subscrib=rospy.Subscriber("/speech_text", String, self.speech_callback)
        
        
        speak_this("DEFAULT MODE IS ACTIVE")

        while self.next_state=="" and not rospy.is_shutdown():
                time.sleep(0.2)
                rospy.loginfo("Waiting sentence")
        

        if not rospy.is_shutdown():
            speak_this("EXITING DEFAULT MODE")
            self.subscribe.unregister()
            rospy.loginfo("NextState: "+self.next_state)
  
        active_check_face_object = False
        kill_all_process(pids)
        return self.next_state


def main():
    
    rospy.init_node("phoenix_brain")
    rospy.loginfo("Starting Phoenix Brain")
    rospy.wait_for_service('speak_text')
    try:
        speak_text_service = rospy.ServiceProxy('speak_text', FestivalSpeech)
    except rospy.ServiceException, e:
        print "Failed to acquire Festival SpeakText service: %s"%e

    rospy.spin()


if __name__ == '__main__':
    main()

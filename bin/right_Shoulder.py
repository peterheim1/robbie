#!/usr/bin/env python
'''
Created March, 2012

@author: Peter Heim

  r_shoulder.py - gateway to Arduino based arm controller
  Copyright (c) 2011 Peter Heim.  All right reserved.
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.
 
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import roslib; roslib.load_manifest('robbie')
import rospy
import tf
import math
from math import sin, cos, pi
import sys
import time
from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import JointState
#from sensor_msgs.msg import JointState
from SerialDataGateway import SerialDataGateway

class R_shoulder(object):
        '''
        Helper class for communicating with an R_shoulder board over serial port
        '''

       

        def _HandleReceivedLine(self,  line):
                self._Counter = self._Counter + 1
                #rospy.logdebug(str(self._Counter) + " " + line)
                #if (self._Counter % 50 == 0):
                self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))

                if (len(line) > 0):
                        lineParts = line.split('\t')
                       
                        if (lineParts[0] == 'P1'):
                                self._BroadcastJointStateinfo_P1(lineParts)
                                return
                        if (lineParts[0] == 'P2'):
                                self._BroadcastJointStateinfo_P2(lineParts)
                                return
                        if (lineParts[0] == 'P3'):
                                self._BroadcastJointStateinfo_P3(lineParts)
                                return
                        if (lineParts[0] == 'P4'):
                                self._BroadcastJointStateinfo_P4(lineParts)
                                return
                       


        
        def _BroadcastJointStateinfo_P1(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 7):
                        pass
                try:
                        #P1 = 0-(118-(float(lineParts[1])* 0.001533203))# position
                        #P2 = 0-(118-(float(lineParts[2])* 0.001533203))#float(lineParts[2])# target
                        P1 = (float(lineParts[1])-118)* 0.001533203# position
                        P2 = (float(lineParts[1])-118)* 0.001533203# position#float(lineParts[2])# target
                        P3 = float(lineParts[3])# current
                        P4 = float(lineParts[4])# speed
                        val = [P1, P2, P3, P4]
                        Motor_State = MotorState()
                        Motor_State.id = 11
                        Motor_State.goal = P2
                        Motor_State.position = P1
                        Motor_State.speed = P4
                        Motor_State.load = P3
                        Motor_State.moving = 0
                        Motor_State.timestamp = time.time()
                        self.P2_MotorPublisher.publish(Motor_State)
                       
                        Joint_State = JointState()
                        Joint_State.name = "right_arm_tilt_joint"
                        Joint_State.goal_pos = P2
                        Joint_State.current_pos = P1
                        Joint_State.velocity = P4
                        Joint_State.load = P3
                        Joint_State.error = P1 - P2
                        Joint_State.is_moving = 0
                        Joint_State.header.stamp = rospy.Time.now()#.stamprospy.Time.from_sec(state.timestamp)
                        self._P1_JointPublisher.publish(Joint_State)
                        #rospy.logwarn(Joint_State)

                except:
                        rospy.logwarn("Unexpected error:2" + str(sys.exc_info()[0]))

        def _BroadcastJointStateinfo_P2(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 7):
                        pass
                try:
                        A1 = 200 -float(lineParts[1])
                        P1 = 0 - (A1* 0.003070961)#(3.8-((float(lineParts[1])* 0.004597701)))# position
                        A2 = 200 -float(lineParts[2])
                        P2 = 0 - (A2* 0.003070961)#float(lineParts[2])# target
                        P3 = float(lineParts[3])# current
                        P4 = float(lineParts[4])# speed
                        val = [P1, P2, P3, P4]
                        Motor_State = MotorState()
                        Motor_State.id = 11
                        Motor_State.goal = P2
                        Motor_State.position = P1
                        Motor_State.speed = P4
                        Motor_State.load = P3
                        Motor_State.moving = 0
                        Motor_State.timestamp = time.time()
                        self.P2_MotorPublisher.publish(Motor_State)
                       
                        Joint_State = JointState()
                        Joint_State.name = "right_arm_lift_joint"
                        Joint_State.goal_pos = P2
                        Joint_State.current_pos = P1
                        Joint_State.velocity = P4
                        Joint_State.load = P3
                        Joint_State.error = P1 - P2
                        Joint_State.is_moving = 0
                        Joint_State.header.stamp = rospy.Time.now()#.stamprospy.Time.from_sec(state.timestamp)
                        self._P2_JointPublisher.publish(Joint_State)
                        #rospy.logwarn(Joint_State)

                except:
                        rospy.logwarn("Unexpected error:2" + str(sys.exc_info()[0]))

        def _BroadcastJointStateinfo_P3(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 7):
                        pass
                try:
                        A1 = 110 +float(lineParts[1])
                        A2 = 110 +float(lineParts[2])
                        P1 = float(A1)* 0.003070961
                        P2 = float(A2)* 0.003070961
                        P3 = float(lineParts[3])
                        P4 = float(lineParts[4])
                        val = [P1, P2, P3, P4]
                        Motor_State = MotorState()
                        Motor_State.id = 11
                        Motor_State.goal = P2
                        Motor_State.position = P1
                        Motor_State.speed = P4
                        Motor_State.load = P3
                        Motor_State.moving = 0
                        Motor_State.timestamp = time.time()
                        self.P3_MotorPublisher.publish(Motor_State)
                        #rospy.logwarn(Motor_State)
                        #rospy.logwarn(val)

                        Joint_State = JointState()
                        Joint_State.name = "right_arm_rotate_joint"
                        Joint_State.goal_pos = P2
                        Joint_State.current_pos = P1
                        Joint_State.velocity = P4
                        Joint_State.load = P3
                        Joint_State.error = P1 - P2
                        Joint_State.is_moving = 0
                        Joint_State.header.stamp = rospy.Time.now()
                        self._P3_JointPublisher.publish(Joint_State)

                except:
                        rospy.logwarn("Unexpected error:3" + str(sys.exc_info()[0]))

        def _BroadcastJointStateinfo_P4(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 7):
                        pass
                try:
                        P1 = float(lineParts[1])* 0.003070961
                        P2 = float(lineParts[2])* 0.003070961
                        P3 = float(lineParts[3])
                        P4 = float(lineParts[4])
                        val = [P1, P2, P3, P4]
                        Motor_State = MotorState()
                        Motor_State.id = 11
                        Motor_State.goal = P2
                        Motor_State.position = P1
                        Motor_State.speed = P4
                        Motor_State.load = P3
                        Motor_State.moving = 0
                        Motor_State.timestamp = time.time()
                        self.P4_MotorPublisher.publish(Motor_State)
                        #rospy.logwarn(Motor_State)
                        #rospy.logwarn(val)

                        Joint_State = JointState()
                        Joint_State.name = "right_arm_elbow_joint"
                        Joint_State.goal_pos = P2
                        Joint_State.current_pos = P1
                        Joint_State.velocity = P4
                        Joint_State.load = P3
                        Joint_State.error = P1 - P2
                        Joint_State.is_moving = 0
                        Joint_State.header.stamp = rospy.Time.now()
                        self._P4_JointPublisher.publish(Joint_State)
                except:
                        rospy.logwarn("Unexpected error:4" + str(sys.exc_info()[0]))


        

       
        def _WriteSerial(self, message):
                self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
                self._SerialDataGateway.Write(message)

        def __init__(self,):
                '''
                Initializes the receiver class.
                port: The serial port to listen to.
                baudrate: Baud rate for the serial communication
                '''
               
                #port = rospy.get_param("~port", "/dev/ttyACM0")
                #baud = int(rospy.get_param("~baud", "115200"))
                #self.name = name
                self.rate = rospy.get_param("~rate", 100.0)
                self.fake = rospy.get_param("~sim", False)
               
                #name = rospy.get_param("~name")
                self._Counter = 0

                rospy.init_node('r_shoulder')

                port = rospy.get_param("~port", "/dev/ttyACM0")
                baudRate = int(rospy.get_param("~baudRate", 115200))

                rospy.logwarn("Starting Right shoulder with serial port: " + port + ", baud rate: " + str(baudRate))

                # subscriptions
                
                
                
                rospy.Subscriber('right_arm_tilt_joint/command',Float64, self._HandleJoint_1_Command)
                rospy.Subscriber('right_arm_lift_joint/command',Float64, self._HandleJoint_2_Command)
                rospy.Subscriber('right_arm_rotate_joint/command',Float64, self._HandleJoint_3_Command)
                rospy.Subscriber('right_arm_elbow_joint/command',Float64, self._HandleJoint_4_Command)
                
                self._SerialPublisher = rospy.Publisher('arm_serial', String)

               
                
                self.P1_MotorPublisher = rospy.Publisher("/right_arm_tilt/motor_state", MotorState)
                self.P2_MotorPublisher = rospy.Publisher("/right_arm_lift/motor_state", MotorState)
                self.P3_MotorPublisher = rospy.Publisher("/right_arm_rotate/motor_state", MotorState)
                self.P4_MotorPublisher = rospy.Publisher("/right_arm_elbow/motor_state", MotorState)
                
                self._P1_JointPublisher = rospy.Publisher("/right_arm_tilt_joint/state", JointState)#geterrors
                self._P2_JointPublisher = rospy.Publisher("/right_arm_lift_joint/state", JointState)
                self._P3_JointPublisher = rospy.Publisher("/right_arm_rotate_joint/state", JointState)
                self._P4_JointPublisher = rospy.Publisher("/right_arm_elbow_joint/state", JointState)#add joint
                

               
                #self.servo_state_Publisher = rospy.Publisher('servo_state', OmniControllerState)
               

               
               

                self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)

        def Start(self):
                rospy.loginfo("Starting start function")
                self._SerialDataGateway.Start()
                message = 'r \r'
                self._WriteSerial(message)

        def Stop(self):
                rospy.loginfo("Stopping")
                message = 'r \r'
                self._WriteSerial(message)
                sleep(5)
                self._SerialDataGateway.Stop()
               
        def _HandleJoint_1_Command(self, twistCommand):#tilt
                """ Handle movement requests. tilt"""
                v = twistCommand.data      # m/s
                #if v > 1.58: v = 1.58
                v1 =float(v * 647.468354435)+243#57.2957795)
                if v1 < 243: v1 = 243
                #y = twistCommand.linear.y        # m/s
                #omega = twistCommand.angular.z      # rad/s
                #rospy.logwarn("Handling tilt command: " + str(v1) + "," + str(v) + ","+ str(v1))

                #message = 's %.2f %.2f %.2f\r' % self._GetBaseAndExponents((v1))
                message = 'j2 %d \r' % (v1)#% self._GetBaseAndExponents((v1)
                rospy.logwarn("Sending arm tilt message: " + (message))
                self._WriteSerial(message)

        def _HandleJoint_2_Command(self, twistCommand):#lift
                """ Handle movement requests. """
                v = twistCommand.data      # m/s
                
                v1 =float(v * 325.95)+200
                if v1 < 200: v1 = 2000          
               
                message = 'j1 %d \r' % (v1)#% self._GetBaseAndExponents((v1)
                rospy.logwarn("Sending lift command message: " + (message))
                self._WriteSerial(message)
               
        def _HandleJoint_3_Command(self, twistCommand):
                """ rotate arm function"""
                v = twistCommand.data      # m/s
                v1 =float(v * 325.95)
                #v2 = 180 - v1
                #y = twistCommand.linear.y        # m/s
                #omega = twistCommand.angular.z      # rad/s
                #rospy.logwarn("Handling twist command: " + str(v1) + "," + str(v) + ","+ str(v1))

                #message = 's %.2f %.2f %.2f\r' % self._GetBaseAndExponents((v1))
                message = 'j3 %d \r' % (v1)#% self._GetBaseAndExponents((v1)
                rospy.logwarn("Sending rotate command message: " + (message))
                self._WriteSerial(message)
                #self._WriteSerial(message)
               
        def _HandleJoint_4_Command(self, twistCommand):
                """ Handle elbow lift movement requests. """
                v = twistCommand.data      # m/s
                #if v < 0.4: v = 0.4
                #if v > 1.65: v = 1.65
                v1 =float(v * 325.95)
                if v1 > 900: v1 = 900
                #y = twistCommand.linear.y        # m/s
                #omega = twistCommand.angular.z      # rad/s
                rospy.logwarn("Handling elbow command: " + str(v1) )

                #message = 's %.2f %.2f %.2f\r' % self._GetBaseAndExponents((v1))
                message = 'j4 %d \r' % (v1)#% self._GetBaseAndExponents((v1)
                #rospy.logwarn("Sending speed command message: " + (message))
                self._WriteSerial(message)
               

        def _HandleJoint_5_Command(self, twistCommand):
                """ Handle movement requests. """
                v = twistCommand.data     # m/s
                v1 =float((v * 57.2957795))
                v2 = 180 - v1
                
                message = 'j5 %d \r' % (v2)#% self._GetBaseAndExponents((v1)
                rospy.logwarn("Sending wrist tilt command message: " + (message))
                self._WriteSerial(message) 
                
        def _HandleJoint_6_Command(self, twistCommand):
                """ Handle movement requests. """
                v = twistCommand.data      # m/s
                v1 =float((v * 57.2957795) )
                v2 = 180 - v1
                
                message = 'j6 %d \r' % (v2)#% self._GetBaseAndExponents((v1)
                rospy.logwarn("Sending f1 command message: " + (message))
                self._WriteSerial(message)    
                
        def _HandleJoint_7_Command(self, twistCommand):
                """ Handle movement requests. """
                v = twistCommand.data      # m/s
                v1 =float((v * 57.2957795) )
                v2 = 180 - v1
                
                message = 'j7 %d \r' % (v2)#% self._GetBaseAndExponents((v1)
                #rospy.loginfo("Sending speed command message: " + (message))
                self._WriteSerial(message) 
                
        def _HandleJoint_8_Command(self, twistCommand):
                """ Handle movement requests. """
                v = twistCommand.data      # m/s
                v1 =float((v * 57.2957795) )
                v2 = 180 - v1
                
                message = 'j8 %d \r' % (v2)#% self._GetBaseAndExponents((v1)
                #rospy.loginfo("Sending speed command message: " + (message))
                self._WriteSerial(message)  
                
        def _HandleJoint_9_Command(self, twistCommand):
                """ Handle movement requests. """
                v = twistCommand.data      # m/s
                v1 =float((v * 57.2957795) )
                v2 = 180 - v1
                
                message = 'j9 %d \r' % (v2)#% self._GetBaseAndExponents((v1)
                #rospy.loginfo("Sending speed command message: " + (message))
                self._WriteSerial(message)
                
        def _HandleJoint_10_Command(self, twistCommand):
                """ Handle movement requests. """
                v = twistCommand.data      # m/s
                v1 =float((v * 57.2957795) )
                v2 = 180 - v1
                
                message = 'j10 %d \r' % (v2)#% self._GetBaseAndExponents((v1)
                #rospy.loginfo("Sending speed command message: " + (message))
                self._WriteSerial(message)                                     
 

        def _GetBaseAndExponent(self, floatValue, resolution=4):
                '''
                Converts a float into a tuple holding two integers:
                The base, an integer with the number of digits equaling resolution.
                The exponent indicating what the base needs to multiplied with to get
                back the original float value with the specified resolution.
                '''

                if (floatValue == 0.0):
                        return (0, 0)
                else:
                        exponent = int(1.0 + math.log10(abs(floatValue)))
                        multiplier = math.pow(10, resolution - exponent)
                        base = int(floatValue * multiplier)

                        return(base, exponent - resolution)

        def _GetBaseAndExponents(self, floatValues, resolution=4):
                '''
                Converts a list or tuple of floats into a tuple holding two integers for each float:
                The base, an integer with the number of digits equaling resolution.
                The exponent indicating what the base needs to multiplied with to get
                back the original float value with the specified resolution.
                '''

                baseAndExponents = []
                for floatValue in floatValues:
                        baseAndExponent = self._GetBaseAndExponent(floatValue)
                        baseAndExponents.append(baseAndExponent[0])
                        baseAndExponents.append(baseAndExponent[1])

                return tuple(baseAndExponents)


if __name__ == '__main__':
        r_shoulder = R_shoulder()
        try:
                r_shoulder.Start()
                rospy.spin()

        except rospy.ROSInterruptException:
                r_shoulder.Stop()


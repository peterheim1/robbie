#!/usr/bin/env python
'''
Created March, 2012

@author: Peter Heim

  Neck_Tilt.py - gateway to Arduino based arm controller
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

import rospy
import rospy
import socket
import sys,re
import rospy
from std_msgs.msg import String

class Julius():
        def __init__(self):
            self.host = "localhost"
            self.port = 10500
            self._chatPublisher = rospy.Publisher("quest_talk", String)
            self.line = ""

            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            print "Connection to Julius succeeded"
            #sys.stdout.flush()
            while True:
                # build a valid line
                tmpline = self.sock.recv(1024).replace(".\n", "").rstrip()
                self.line += tmpline
                print "WAITING"
                if not tmpline[-1] == ">":
                   continue
            print self.line

            def stop(self):
                pass

if __name__ == '__main__':
    try:
        st = Julius()
        rospy.init_node('julius')
        rospy.spin()
    except rospy.ROSInterruptException:
        st.stop()

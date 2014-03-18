#!/usr/bin/env python
# Author: Derek Green

PKG = 'pocketsphinx'

#import roslib; roslib.load_manifest(PKG)
import rospy
import re
import os

from std_msgs.msg import String
from subprocess import Popen, PIPE

class SpeechText():
    def __init__(self):
        self.pub = rospy.Publisher('speech_text', String)
        rospy.init_node('speech_text_node', anonymous=True)
        path = rospy.get_param("/speech_text/lm_path")

        # the language model directory found at path should have a .dic and a .lm, grab them:
        lm = None
        dic = None
        for filename in os.listdir(path):
            if re.match(".*\.dic", filename):
                dic = filename
            if re.match(".*\.lm", filename):
                lm = filename

        if lm and dic:
            args = ["pocketsphinx_continuous", "-hmm", "/usr/share/pocketsphinx/model/hmm/wsj1", "-lm", path + lm, "-dict", path + dic]
            self.ps = Popen(args, stdin=PIPE, stdout=PIPE, stderr=PIPE)
            rospy.on_shutdown( self.clean_up )
        else:
            print "ERROR:  pocketsphinx is missing language model file.  dic = " + dic + ", lm = " + lm

    def speech_to_text(self):
        print "ENTERING SPEECH_TEXT"
        while not rospy.is_shutdown():
            line = self.ps.stdout.readline()
            if re.match("READY.*",line):
                print "======= pocket sphinx is ready ======="
            heard = re.match("\d{9}[:](.*)( [(]-\d*[)])",line)
            if heard:
                out = heard.group(1).lower().strip()
                print "JUST HEARD: \"" + out + "\""
                rospy.loginfo(out)
                self.pub.publish(out)

    def clean_up(self):
        print "=============================== speech_txt is shutting down.  Killing pocketsphinx process #", self.ps.pid
        self.ps.kill()

if __name__ == '__main__':
    try:
        st = SpeechText()
        st.speech_to_text()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

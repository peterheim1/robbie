#!/usr/bin/env python
import roslib; roslib.load_manifest('robbie')
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String

def callback(data):
    #rospy.loginfo(data.data)
    voltage = data.data
    if voltage < 12.4:
        str = "I am hurgry please feed me %s"
        rospy.loginfo("feed me")
        pub = rospy.Publisher('chatter', String)
        pub.publish(String(str))


def main():

    
    
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("battery", Float32, callback)
    
    

    
    rospy.spin()

if __name__ == '__main__':
    main()

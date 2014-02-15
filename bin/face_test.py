#! /usr/bin/env python
'''


sends request to action server 
train works
add name works 
recognise continoulley works
recognise once is not very good and switching between the 2 is not goodseems to work if only used short
exit also works


'''

import roslib; roslib.load_manifest('robbie')
import rospy
import actionlib

from face_recognition.msg import *


def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('face_recognition', face_recognition.msg.FaceRecognitionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = face_recognition.msg.FaceRecognitionGoal(order_id=0, order_argument="none")

    # Sends the goal to the action server.
    client.send_goal(goal)


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('face_test_py')
        result = fibonacci_client()
        #print "Result:", ', '.join([str(n) for n in result.sequence])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"


#!/usr/bin/env python
import rospy
import smach
import smach_ros

from std_msgs.msg import Float32

class hungry(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hungry_succeeded'])
    def execute(self, userdata):
        rospy.sleep(3.0)
        return 'hungry_succeeded'

def monitor_cb(ud, msg):
    #rospy.logwarn(msg.data)
    if msg.data < 11.8:
        return False
    else:
        return True
    #return False

def main():
    rospy.init_node("monitor_example")

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('healthy', smach_ros.MonitorState("/battery", Float32, monitor_cb), transitions={'invalid':'HUNGRY', 'valid':'healthy', 'preempted':'healthy'})
        smach.StateMachine.add('HUNGRY',hungry(), transitions={'hungry_succeeded':'healthy'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()


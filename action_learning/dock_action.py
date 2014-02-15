#! /usr/bin/env python

import roslib; roslib.load_manifest('robbie')
import rospy

import actionlib

import robbie.msg

class AutoDockAction(object):
  
  def __init__(self, name):

    # Initialize Robbie base action server
    self.result = AutoDockActionResult()
    self.feedback = AutoDockActionFeedback()
    self.server = SimpleActionServer(NAME, AutoDockAction, self.execute_callback, auto_start=False)
    self.server.start()
    rospy.loginfo("%s: Ready to accept goals", NAME)
   
    
  def execute_cb(self, goal):
    # listen for dock signal
    r = rospy.Rate(1)
    success = True
    
    
    # start auto dock action
    for i in xrange(1, goal.order):
      # check that preempt has not been requested by the client
      if self.server.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self.server.set_preempted()
        success = False
        break
      self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
      # publish the feedback do we need this
      self.server.publish_feedback(self._feedback)
      # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep()
      
    if success:
      self._result.sequence = self._feedback.sequence
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self.server.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('AutoDock')
  AutoDockAction(rospy.get_name())
  rospy.spin()

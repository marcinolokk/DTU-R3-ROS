#!/usr/bin/env python
import rospy

import actionlib
from waypoint_nav.msg import DeliveryAction, DeliveryGoal, DeliveryResult, DeliveryFeedback
from std_msgs.msg import Bool

class delivery_client(object):
  def __init__(self):

    # Variables
    self.tasks = ["Go to corridor", "Navigate in corridor", "Enter logistic room", "Wait for load", "Exit logistic room", "Back in corridor", "Back to Office"]
    self.goal = DeliveryGoal()

    # Init ROS node
    rospy.init_node('delivery_action_client')
    self.client = actionlib.SimpleActionClient('delivery', DeliveryAction)
    self.client.wait_for_server()
    
    # Subscribers
    rospy.Subscriber('/delivery/stop', Bool, self.stopCB)    

  def Start(self):
    self.goal.task = len(self.tasks)
    self.client.send_goal(self.goal, feedback_cb = self.feedbackCB)
    self.client.wait_for_result()
    print self.client.get_result().task_status

  def feedbackCB(self, fb):
    print fb.current_task

  def stopCB(self, s):
    if s.data:
      client.cancel_goal() 

if __name__ == '__main__': 
  c = delivery_client() 
  c.Start()  

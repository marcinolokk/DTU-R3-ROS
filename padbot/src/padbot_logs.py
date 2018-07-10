#!/usr/bin/env python 
import rospy 
import sys
import csv 
import math

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
 
# Class 
class CSV_log(object): 
  def __init__(self): 
    rospy.init_node('padbot_log')
    self.rate = rospy.Rate(10)
    if len(sys.argv) > 1:
      self.file_name = "padbot_" + str(sys.argv[1]) + ".csv"
    else:
      self.file_name = 'log.csv'
    
    self.x = 0
    self.y = 0
    self.theta = 0
    self.v = 0
    self.w = 0
    self.left_count = 0
    self.right_count = 0
    self.last_left = 0
    self.last_right = 0
    self.time = rospy.Time.now()   
    
    # Padbot parameters
    self.track_width = 0.228
    self.distance_per_count = 0.0000439
    
    # Subcribers    
    rospy.Subscriber('padbot/left_count', Int32, self.leftCB)  
    rospy.Subscriber('padbot/right_count', Int32, self.rightCB)
    rospy.Subscriber('cmd_vel', Twist, self.velCB)
    
    self.csvfile = open(self.file_name, 'w') 
    fieldnames = ['timestamp', 'x', 'y', 'theta', 'vel', 'omega', 'left_count', 'right_count'] 
    self.writer = csv.DictWriter(self.csvfile, fieldnames=fieldnames) 
    self.writer.writeheader() 
  
  def Running(self):
    while not rospy.is_shutdown():
      deltaLeft = self.left_count - self.last_left
      deltaRight = self.right_count - self.last_right
      delta_time = float(str(rospy.Time.now() - self.time))/1000000000
      self.time = rospy.Time.now() 
      self.last_left = self.left_count
      self.last_right = self.right_count
      if deltaLeft > 10000 or deltaRight > 10000:
        continue
      deltaDistance = (deltaLeft + deltaRight) * self.distance_per_count / 2
      deltaAngle = (deltaRight - deltaLeft) * self.distance_per_count / self.track_width
      self.x += deltaDistance * math.cos(self.theta)
      self.y += deltaDistance * math.sin(self.theta)
      self.theta += deltaAngle
      self.writer.writerow({'timestamp': self.time, 'x': self.x, 'y': self.y, 'theta': self.theta, 'vel': self.v, 'omega': self.w, 'left_count': self.last_left, 'right_count': self.last_right}) 
      self.rate.sleep()
  
  def Stop(self):
    self.csvfile.close()
  
  def leftCB(self, l):
    self.left_count = l.data
    
  def rightCB(self, r):
    self.right_count = r.data
     
  def velCB(self, vel): 
    self.v = vel.linear.x
    self.w = vel.angular.z

if __name__ == '__main__':  
  log = CSV_log()
  rospy.on_shutdown(log.Stop)
  try:
    log.Running()
  except rospy.ROSInterruptException:
    log.Stop()
    

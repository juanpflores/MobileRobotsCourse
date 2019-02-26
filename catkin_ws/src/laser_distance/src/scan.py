#! /usr/bin/env python
 
"""
A simple Laser Scanner Package that allows the robot 
to know when an object is closer than one meter from
it.
"""

 
import rospy
from sensor_msgs.msg import LaserScan
 
def callback(msg):
  for value in msg.ranges:
    if value <= 1:
      print "OBJETO CERCA DETECTADO A {}".format(value)
      break
  
 
rospy.init_node('scan_values')
sub = rospy.Subscriber('/hardware/scan', LaserScan, callback)
rospy.spin()
 
 

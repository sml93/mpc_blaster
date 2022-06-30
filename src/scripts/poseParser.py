from __future__ import division
from turtle import pos

import rospy
import unittest
import numpy as np

from geometry_msgs.msg import PoseStamped

class poseParser():
  def __init__(self, *args):
    super(poseParser, self).__init__(*args)

  def setUp(self):
    self.local_position = PoseStamped()
    self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                          PoseStamped, self.local_position_callback)

  def local_position_callback(self, data):
    self.local_position = data

  def tearDown(self):
    self.log_topic_vars()

  def log_topic_vars(self):
    """ log the state of topic variables """
    rospy.loginfo("local_position:\n{}".format(self.local_position))
    rospy.loginfo("========================")

  def getPose(self):
    position_x = self.local_position.pose.position.x
    position_y = self.local_position.pose.position.y
    position_z = self.local_position.pose.position.z
    orient_x = self.local_position.pose.orientation.x
    orient_y = self.local_position.pose.orientation.y
    orient_z = self.local_position.pose.orientation.z
    return position_x, position_y, position_z, orient_x, orient_y, orient_z
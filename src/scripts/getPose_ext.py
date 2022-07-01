#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped

uav_pose = PoseStamped()

class pose():
  def uav_pose_cb(self, data):
    global uav_pose
    uav_pose = data

def main():
  global uav_pose
  rospy.init_node('getPose', anonymous=True)
  get = pose()
  rospy.Subscriber('mavros/local_position/pose', PoseStamped, get.uav_pose_cb)
  # px = uav_pose.pose.position.x
  # py = uav_pose.pose.position.y
  pz = uav_pose.pose.position.z
  # ox = uav_pose.pose.orientation.x
  # oy = uav_pose.pose.orientation.y
  # oz = uav_pose.pose.orientation.z
  print(pz)
  # pose_array = np.array((px, py, pz, ox, oy, oz))     # Slow will have a substantial lag behind the actual data. 
  # print(pose_array)


if __name__=='__main__':
  try:
    while not rospy.is_shutdown():
      main()
    rospy.Rate(1000).sleep()
  except rospy.ROSInterruptException:
    pass
  except KeyboardInterrupt:
    print("Shutting down")

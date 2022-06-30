#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped

uav_pose = PoseStamped()

def uav_pose_cb(data):
  global uav_pose
  uav_pose = data

# def getPose():
#   global uav_pose
#   px = uav_pose.pose.position.x
#   py = uav_pose.pose.position.y
#   pz = uav_pose.pose.position.z
#   ox = uav_pose.pose.orientation.x
#   oy = uav_pose.pose.orientation.y
#   oz = uav_pose.pose.orientation.z
#   pose_ = np.array((px, py, pz, ox, oy, oz))
#   return pose_

def main():
  global uav_pose
  rospy.Subscriber('mavros/local_position/pose', PoseStamped, uav_pose_cb)
  while not rospy.is_shutdown():
    px = uav_pose.pose.position.x
    py = uav_pose.pose.position.y
    pz = uav_pose.pose.position.z
    ox = uav_pose.pose.orientation.x
    oy = uav_pose.pose.orientation.y
    oz = uav_pose.pose.orientation.z
    pose = np.array((px, py, pz, ox, oy, oz))
    print(pose)
  rospy.Rate(20).sleep()

# def main():
#   global uav_pose, px
#   while not rospy.is_shutdown():
#     print(pose()[0])
#     print(pose()[1])
#     print(pose()[2])
#     print(pose()[3])
#     print(pose()[4])
#     print(pose()[5])


if __name__=='__main__':
  try: main()
  except rospy.ROSInterruptException:
    pass
  except KeyboardInterrupt:
    print("Shutting down")

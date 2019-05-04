import transform as tf
import scipy.linalg
import numpy as np
import rosbag
import rospy
import math
import sys

args = sys.argv

if len(args) < 2:
    print("Please insert RosVive data file")
    sys.exit(0)

bag = rosbag.Bag(args[1], "r")

# Dictionary of trackers with poses in vive frame
imu = np.empty((6,0))

for topic, msg, t in bag.read_messages(topics=["/loc/vive/imu"]):
  vec = np.matrix([msg.linear_acceleration.x,
    msg.linear_acceleration.y,
    msg.linear_acceleration.z,
    msg.angular_velocity.x,
    msg.angular_velocity.y,
    msg.angular_velocity.z]).transpose()
  # print(vec)
  # print(imu)
  imu = np.hstack((imu, vec))

print(np.cov(imu))
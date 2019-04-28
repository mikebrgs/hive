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
poses = dict()

vive_positions = (list(),list(),list())
optitrack_positions = (list(),list(),list())

for topic, msg, t in bag.read_messages(topics=["/tf"]):
  if (msg.header.frame_id == "vive"):
    vive_positions[0].append(msg.transform.translation.x)
    vive_positions[1].append(msg.transform.translation.y)
    vive_positions[2].append(msg.transform.translation.z)
  if (msg.header.frame_id == "optitrack"):
    optitrack_positions[0].append(msg.transform.translation.x)
    optitrack_positions[1].append(msg.transform.translation.y)
    optitrack_positions[2].append(msg.transform.translation.z)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(vive_positions[0],
  vive_positions[1],
  vive_positions[2])
ax.plot(optitrack_positions[0],
  optitrack_positions[1],
  optitrack_positions[2])
plt.show()
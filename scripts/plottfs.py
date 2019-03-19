'''Reads a bag and plots the poses'''

import sys
import rospy
import rosbag
import numpy as np

import matplotlib.pyplot as plt

bag = rosbag.Bag(sys.argv[1])
poses = np.empty((7,0))
for topic, msg, t in bag.read_messages(topics=['/tf', 'tf']):
  pose = np.array([[msg.transforms[0].transform.translation.x,
    msg.transforms[0].transform.translation.y,
    msg.transforms[0].transform.translation.z,
    msg.transforms[0].transform.rotation.w,
    msg.transforms[0].transform.rotation.x,
    msg.transforms[0].transform.rotation.y,
    msg.transforms[0].transform.rotation.z]]).transpose()
  poses = np.hstack((poses,pose))

fig2, axs = plt.subplots(4, 2, sharex = True, sharey= False)

axs[0,0].plot(poses[0,:])
axs[0,0].set_title("PX")
axs[1,0].plot(poses[1,:])
axs[0,0].set_title("PY")
axs[2,0].plot(poses[2,:])
axs[0,0].set_title("PZ")
axs[0,1].plot(poses[3,:])
axs[0,0].set_title("QW")
axs[1,1].plot(poses[4,:])
axs[0,0].set_title("QX")
axs[2,1].plot(poses[5,:])
axs[0,0].set_title("QY")
axs[3,1].plot(poses[6,:])
axs[0,0].set_title("QZ")
plt.show()
import transform as tf
import scipy.linalg
import numpy as np
import rosbag
import rospy
import math
import sys

if len(args) < 2:
    print("Please insert RosVive data file")
    sys.exit(0)

bag = rosbag.Bag(args[1], "r")

# Dictionary of trackers with poses in vive frame
poses = dict()

for topic, msg, t in bag.read_messages(topics=["/tf"]):

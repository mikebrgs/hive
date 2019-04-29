import sys
import rospy
import rosbag
import numpy as np

args = sys.argv

if len(args) < 2:
  print("Please insert RosVive data file")
  sys.exit(0)

bag = rosbag.Bag(args[1], "r")

thecounter = 0

for topic, msg, t in bag.read_messages(topics=["/loc/vive/light"]):
  print(msg.lighthouse + " - " + str(msg.axis))
  for sample in msg.samples:
    print("\t" + str(sample.sensor) + " " + str(sample.angle))
  thecounter += 1
  if thecounter == 2:
    sys.exit(0)

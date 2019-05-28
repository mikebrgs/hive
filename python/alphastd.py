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
alphaH = dict()
alphaV = dict()
for i in range(0,24):
  alphaH[i] = list()
  alphaV[i] = list()

for topic, msg, t in bag.read_messages(topics=["/loc/vive/light"]):
  if msg.axis == 1:
    for sample in msg.samples:
      if sample.angle > math.pi/3 or sample.angle < -math.pi / 3:
        continue
      # if (sample.sensor == 21):
        # print(str(msg.axis) + " - " + str(sample.angle))
      alphaV[sample.sensor].append(sample.angle)
  elif msg.axis == 0:
    for sample in msg.samples:
      if sample.angle > math.pi/3 or sample.angle < -math.pi / 3:
        continue
      # if (sample.sensor == 21):
        # print(str(msg.axis) + " - " + str(sample.angle))
      alphaH[sample.sensor].append(sample.angle)

counter = 0.0
stds = 0.0;
print("Horizontal")
for i in range(0,24):
  if len(alphaH[i]) < 10:
    continue
  # print("Sensor " + str(i) + " --- " + str(alphaH[i]))
  # print("\n\n\n\n\n")
  print("Sensor " + str(i) + " " + str(np.std(alphaH[i])))
  counter += 1
  stds += np.std(alphaH[i])

print("Vertical")
for i in range(0,24):
  if len(alphaV[i]) < 10:
    continue
  # print("Sensor " + str(i) + " --- " + str(alphaV[i]))
  # print("\n\n\n\n\n")
  print("Sensor " + str(i) + " " + str(np.std(alphaV[i])))
  counter += 1
  stds += np.std(alphaV[i])

print(stds/counter)

# import matplotlib.pyplot as plt
# for i in range(0, 24):
#   plt.plot(alphas[i])
#   break
# plt.show()
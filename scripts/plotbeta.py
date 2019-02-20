import sys
import rosbag

import matplotlib.pyplot as plt

args = sys.argv

if len(args) < 2:
  print("Please insert RosVive data file")
  sys.exit(0)

bag = rosbag.Bag(args[1], "r")

hdurations = dict()
vdurations = dict()

for topic, msg, t in bag.read_messages(topics=["/loc/vive/light"]):
  if topic == "/loc/vive/light":
    if not msg.lighthouse in hdurations:
      hdurations[msg.lighthouse] = dict()
    if not msg.lighthouse in vdurations:
      vdurations[msg.lighthouse] = dict()
    for sample in msg.samples:
      if msg.axis == 0:
        if not sample.sensor in hdurations[msg.lighthouse]:
          hdurations[msg.lighthouse][sample.sensor] = list()
        hdurations[msg.lighthouse][sample.sensor].append(sample.length)
      elif msg.axis == 1:
        if not sample.sensor in vdurations[msg.lighthouse]:
          vdurations[msg.lighthouse][sample.sensor] = list()
        vdurations[msg.lighthouse][sample.sensor].append(sample.length)

lh_1 = hdurations.keys()[0]
# lh_2 = durations.keys()[1]
# fig, axs = plt.subplots(4,2)
fig, axs = plt.subplots(2,1)
axs[0].plot(hdurations[lh_1][6])
axs[0].set_title("HOR")
axs[1].plot(vdurations[lh_1][6])
axs[1].set_title("VER")

# axs[0,0].plot(durations[lh_1][2])
# axs[0,0].set_title("3")
# axs[1,0].plot(durations[lh_1][7])
# axs[1,0].set_title("4")
# axs[2,0].plot(durations[lh_1][12])
# axs[2,0].set_title("7")
# axs[3,0].plot(durations[lh_1][21])
# axs[3,0].set_title("21")
# axs[0,1].plot(durations[lh_2][2])
# axs[0,1].set_title("3")
# axs[1,1].plot(durations[lh_2][7])
# axs[1,1].set_title("4")
# axs[2,1].plot(durations[lh_2][12])
# axs[2,1].set_title("7")
# axs[3,1].plot(durations[lh_2][21])
# axs[3,1].set_title("21")
plt.show()
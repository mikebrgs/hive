import sys
import rosbag

args = sys.argv

if len(args) < 2:
  print("Please insert RosVive data file")
  sys.exit(0)

bag = rosbag.Bag(args[1], "r")

for topic, msg, t in bag.read_messages(topics=["/tf"]):
  print(msg)
  # if topic == "/tf":
  #   for smp_msg in msg.transforms:
  #     print(str(smp_msg.header.stamp) + " - " + str(topic))
  # else: 
  #   print(str(msg.header.stamp) + " - " + str(topic))
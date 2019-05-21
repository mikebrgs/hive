import sys
import rosbag

args = sys.argv

if len(args) < 2:
  print("Please insert RosVive data file")
  sys.exit(0)

bag = rosbag.Bag(args[1], "r")
# "/loc/vive/imu", "loc/vive/imu/", 
for topic, msg, t in bag.read_messages(topics=["/loc/vive/trackers"]):
  # print(str(msg.linear_acceleration.x) + " " +
  #   str(msg.linear_acceleration.y) + " " +
  #   str(msg.linear_acceleration.z))
  print(msg)
  # if topic == "/tf":
  #   for smp_msg in msg.transforms:
  #     print(str(smp_msg.header.stamp) + " - " + str(topic))
  # else: 
  #   print(str(msg.header.stamp) + " - " + str(topic))
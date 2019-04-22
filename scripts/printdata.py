import sys
import rospy
import rosbag

args = sys.argv

if len(args) < 2:
    print("Please insert RosVive data file")
    sys.exit(0)

bag = rosbag.Bag(args[1], "r")

print("Starting")
for topic, msg, t in bag.read_messages(topics=["/trackers"]):
    print(msg)
print("Ending")

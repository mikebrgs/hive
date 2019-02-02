import sys
import cv2
import rospy
import rosbag
import numpy as np

args = sys.argv

if len(args) < 2:
    print("Please insert RosVive data file")
    sys.exit(0)

bag = rosbag.Bag(args[1], "r")

print("Starting")

trackers = dict()
light_horizontal = dict()
light_vertical = dict()


for topic, msg, t in bag.read_messages(topics=["/loc/vive/light", "/loc/vive/trackers"]):
    # Get tracker data
    if topic == "/loc/vive/trackers":
        for tracker in msg.trackers:
            trackers[tracker.serial] = dict()
            trackers[tracker.serial]["positions"] = np.zeros((len(tracker.extrinsics),3))
            trackers[tracker.serial]["normals"] = np.zeros((len(tracker.extrinsics),3))
            # Save positions
            for sensor in tracker.extrinsics:
                trackers[tracker.serial]["positions"][sensor.id,0] = sensor.position.x
                trackers[tracker.serial]["positions"][sensor.id,1] = sensor.position.y
                trackers[tracker.serial]["positions"][sensor.id,2] = sensor.position.z
                trackers[tracker.serial]["normals"][sensor.id,0] = sensor.normal.x
                trackers[tracker.serial]["normals"][sensor.id,1] = sensor.normal.y
                trackers[tracker.serial]["normals"][sensor.id,2] = sensor.normal.z
    # Get light data
    if topic == "/loc/vive/light":
        # Figure out the axis
        if msg.axis == 0:
            light_horizontal[msg.lighthouse] = dict()
            for sample in msg.samples:
                light_horizontal[msg.lighthouse][sample.sensor] = np.tan(sample.angle)
                # light_horizontal[sample.sensor] = sample.angle
        elif msg.axis == 1:
            light_vertical[msg.lighthouse] = dict()
            for sample in msg.samples:
                light_vertical[msg.lighthouse][sample.sensor] = np.tan(sample.angle)
                # light_vertical[sample.sensor] = sample.angle
        # See if there's enough data and solve the PnP problem
        if (msg.lighthouse in light_horizontal and
            msg.lighthouse in light_vertical and
            len(light_horizontal[msg.lighthouse]) >= 5 and
            len(light_vertical[msg.lighthouse]) >= 5):
            image_points = np.zeros((len(trackers[msg.header.frame_id]["positions"]),2))
            detected_sensors = list()
            for sensor in light_horizontal[msg.lighthouse]:
                if sensor in light_vertical[msg.lighthouse]:
                    image_points[sensor,0] = light_horizontal[msg.lighthouse][sensor]
                    image_points[sensor,1] = light_vertical[msg.lighthouse][sensor]
                    detected_sensors.append(sensor)
            if len(detected_sensors) > 3:
                print("********")
                ret, rvec, tvec = cv2.solvePnP(trackers[msg.header.frame_id]["positions"][detected_sensors],image_points[detected_sensors],np.eye(3),None)
                print(msg.header.frame_id + " - " + msg.lighthouse)
                print(rvec)
                print(tvec)
print("Ending")
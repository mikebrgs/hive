import sys
import cv2
import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
args = sys.argv

if len(args) < 2:
    print("Please insert RosVive data file")
    sys.exit(0)

bag = rosbag.Bag(args[1], "r")

trackers = dict()
light_horizontal = dict()
light_vertical = dict()
dot_product = dict()
durations = dict()
poses = dict()

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
        elif msg.axis == 1:
            light_vertical[msg.lighthouse] = dict()
            for sample in msg.samples:
                light_vertical[msg.lighthouse][sample.sensor] = np.tan(sample.angle)
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
                    # Parametrization used for the previous solver (rotate the x and y axis 90deg around z)
                    # image_points[sensor,1] = -light_horizontal[msg.lighthouse][sensor]
                    # image_points[sensor,0] = light_vertical[msg.lighthouse][sensor]
                    detected_sensors.append(sensor)
            if len(detected_sensors) > 3:
                ret, lAAt, lPt = cv2.solvePnP(trackers[msg.header.frame_id]["positions"][detected_sensors],image_points[detected_sensors],np.eye(3),None)
                print(msg.lighthouse + " - " +
                  str(lPt[0,0]) + ", " +
                  str(lPt[1,0]) + ", " +
                  str(lPt[2,0]) + ", " +
                  str(lAAt[0,0]) + ", " +
                  str(lAAt[1,0]) + ", " +
                  str(lAAt[2,0]))
                # print("****" + msg.lighthouse + "****")
                # print("P: " + str(lPt))
                # print("AA: " + str(lAAt))
                for sensor in detected_sensors:
                    tPp = np.asmatrix(trackers[msg.header.frame_id]["positions"][sensor]).reshape((3,1))
                    lRt = np.eye(3)
                    cv2.Rodrigues(lAAt, lRt)
                    lPt = np.asmatrix(lPt).reshape((3,1))
                    lPp = lRt * tPp + lPt
                    # print(str(sensor) + " - " + str(lPp[0,0]/lPp[2,0]) + "/" + str(image_points[sensor][0]) + " - " + str(lPp[1,0]/lPp[2,0]) + "/" + str(image_points[sensor][1]))
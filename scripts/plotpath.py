import sys
import cv2
import rospy
import rosbag
import random
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
                    detected_sensors.append(sensor)
            if len(detected_sensors) > 3:
                # print("********")
                # ret, wAAt, wPt = cv2.solvePnP(trackers[msg.header.frame_id]["positions"][detected_sensors],
                #   image_points[detected_sensors].reshape(image_points[detected_sensors].shape[0],1,2),
                #   np.eye(3),
                #   None,
                #   flags = cv2.SOLVEPNP_EPNP)
                # RANSAC style approach
                best_error = 9e99
                best_sensors = None
                for index in range(0,5):
                  saved_sensors = list()
                  for sensor in range(0,4):
                    sensor = random.randrange(0,len(detected_sensors))
                    while sensor in saved_sensors:
                      sensor = random.randrange(0,len(detected_sensors))
                    saved_sensors.append(sensor)
                  want_sensors = list( detected_sensors[i] for i in saved_sensors )
                  ret, lAt, lPt = cv2.solvePnP(trackers[msg.header.frame_id]["positions"][want_sensors],
                    image_points[want_sensors].reshape(image_points[want_sensors].shape[0],1,2),
                    np.eye(3),
                    None,
                    flags = cv2.SOLVEPNP_AP3P)
                  new_error = 0
                  lRt = np.asmatrix(np.eye(3))
                  cv2.Rodrigues(lAt,lRt)
                  for sensor in detected_sensors:
                    tPs = np.asmatrix(trackers[msg.header.frame_id]["positions"][sensor]).transpose()
                    lPs = lRt * tPs + lPt
                    new_error += abs(lPs[0]/lPs[2] - image_points[sensor,0])
                    new_error += abs(lPs[1]/lPs[2] - image_points[sensor,1])
                  if new_error < best_error:
                    best_error = new_error
                    best_sensors = want_sensors;
                if best_error / 2 * 4 > 0.01:
                  continue
                ret, wAAt, wPt = cv2.solvePnP(trackers[msg.header.frame_id]["positions"][best_sensors],
                  image_points[best_sensors].reshape(image_points[best_sensors].shape[0],1,2),
                  np.eye(3),
                  None,
                  flags = cv2.SOLVEPNP_P3P)
                if not msg.lighthouse in poses:
                    poses[msg.lighthouse] = (list(),list(),list(),list(),list(),list())
                print(str(lPt[0][0]) + ", " +
                  str(lPt[1][0]) + ", " +
                  str(lPt[2][0]) + " - " +
                  # str(lAt[0][0]) + ", " +
                  # str(lAt[1][0]) + ", " +
                  # str(lAt[2][0]) + " - " +
                  str(best_error))
                poses[msg.lighthouse][0].append(lPt[0])
                poses[msg.lighthouse][1].append(lPt[1])
                poses[msg.lighthouse][2].append(lPt[2])
                poses[msg.lighthouse][3].append(lAt[0])
                poses[msg.lighthouse][4].append(lAt[1])
                poses[msg.lighthouse][5].append(lAt[2])

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D

lh_1 = poses.keys()[0]
# lh_2 = poses.keys()[1]

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(xs = poses[lh_2][0], ys = poses[lh_2][1], zs = poses[lh_2][2])
# ax.plot(xs = poses[lh_1][0], ys = poses[lh_1][1], zs = poses[lh_1][2])
# ax.set_xlim(-1,1)
# ax.set_ylim(-1,1)
# ax.set_zlim(0,2)

fig, axs = plt.subplots(3, 2, sharex = False, sharey= False)
axs[0,0].plot(poses[lh_1][0])
axs[1,0].plot(poses[lh_1][1])
axs[2,0].plot(poses[lh_1][2])
# axs[0,0].plot(poses[lh_2][0])
# axs[1,0].plot(poses[lh_2][1])
# axs[2,0].plot(poses[lh_2][2])
# axs[0,0].set_xlim([-3,3])
axs[0,0].set_ylim([-5,5])
axs[1,0].set_ylim([-5,5])
axs[0,1].plot(poses[lh_1][3])
axs[1,1].plot(poses[lh_1][4])
axs[2,1].plot(poses[lh_1][5])
# axs[0,1].plot(poses[lh_2][3])
# axs[1,1].plot(poses[lh_2][4])
# axs[2,1].plot(poses[lh_2][5])
# axs[0,1].set_xlim([-3,3])
axs[0,1].set_ylim([-5,5])
axs[1,1].set_ylim([-5,5])
plt.show()
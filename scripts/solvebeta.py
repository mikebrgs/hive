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
                ret, wAAt, wPt = cv2.solvePnP(trackers[msg.header.frame_id]["positions"][detected_sensors],image_points[detected_sensors],np.eye(3),None)
                if not msg.lighthouse in dot_product:
                    dot_product[msg.lighthouse] = dict()
                    durations[msg.lighthouse] = dict()
                for sample in msg.samples:
                    if not sample.sensor in dot_product[msg.lighthouse]:
                        durations[msg.lighthouse][sample.sensor] = list()
                    durations[msg.lighthouse][sample.sensor].append(sample.length)
                for sensor in detected_sensors:
                    if not sensor in dot_product[msg.lighthouse]:
                        dot_product[msg.lighthouse][sensor] = list()
                    # print(trackers[msg.header.frame_id]["normals"][sensor])
                    # print((wPt / np.linalg.norm(wPt)).ravel())
                    # TODO apply transforms
                    wRt = np.eye(3)
                    cv2.Rodrigues(wAAt,wRt)
                    tN = trackers[msg.header.frame_id]["normals"][sensor].reshape((3,1))
                    tP = trackers[msg.header.frame_id]["positions"][sensor].reshape((3,1))
                    wN = wRt * tN
                    wP = wRt * tP + wPt
                    dot_product[msg.lighthouse][sensor].append(-np.dot((wP / np.linalg.norm(wP)).ravel(),wN.ravel()))
                # print(msg.header.frame_id + " - " + msg.lighthouse)
                # print(wAAt)
                # print(wPt)

lh_1 = dot_product.keys()[0]
lh_2 = dot_product.keys()[1]
# fig, axs = plt.subplots(4,2)
# axs[0,0].plot(dot_product[lh_1][3])
# axs[0,0].set_title("3")
# axs[1,0].plot(dot_product[lh_1][4])
# axs[1,0].set_title("4")
# axs[2,0].plot(dot_product[lh_1][7])
# axs[2,0].set_title("7")
# axs[3,0].plot(dot_product[lh_1][21])
# axs[3,0].set_title("21")
# axs[0,1].plot(dot_product[lh_2][3])
# axs[0,1].set_title("3")
# axs[1,1].plot(dot_product[lh_2][4])
# axs[1,1].set_title("4")
# axs[2,1].plot(dot_product[lh_2][7])
# axs[2,1].set_title("7")
# axs[3,1].plot(dot_product[lh_2][21])
# axs[3,1].set_title("21")
sensor = 7
# factor = -(np.mean(dot_product[lh_1][sensor])) / (np.mean(durations[lh_1][sensor]))
f_dotproduct = np.array(dot_product[lh_1][sensor]) / factor
f_durations = np.array(durations[lh_1][sensor])

f_dotproduct += np.mean(f_durations) - np.mean(f_dotproduct)

print(factor)
# offset = np.mean(dot_product[lh_1][sensor]) - np.mean(durations[lh_1][sensor])
# print(offset)
# print(max(dot_product[lh_1][sensor]))
# print(max(durations[lh_1][sensor]))
# print(np.mean(dot_product[lh_1][sensor]))
# print(np.mean(durations[lh_1][sensor]))
# ndurations = np.array(durations[lh_1][sensor])
plt.plot(f_durations)
plt.plot(f_dotproduct)
plt.show()
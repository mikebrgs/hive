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
duration_horizontal = dict()
duration_vertical = dict()

thesensor = 7
beta_vertical = dict()
alpha_vertical_1 = dict()
alpha_vertical_2 = dict()
alpha_vertical_3 = dict()
alpha_vertical_4 = dict()
beta_horizontal = dict()
alpha_horizontal_1 = dict()
alpha_horizontal_2 = dict()
alpha_horizontal_3 = dict()
alpha_horizontal_4 = dict()
pose_x_horizontal = dict()
pose_x = dict()
pose_y = dict()
pose_y_vertical = dict()
pose_z = dict()
pose_z_horizontal = dict()
pose_z_vertical = dict()

wC1_x = dict()
wC1_y = dict()
wC1_z = dict()

wC2_x = dict()
wC2_y = dict()
wC2_z = dict()

wC3_x = dict()
wC3_y = dict()
wC3_z = dict()

wC4_x = dict()
wC4_y = dict()
wC4_z = dict()

# photodiode
gamma = 5.21367
side = 0.00214663
thecounter = 0

for topic, msg, t in bag.read_messages(topics=["/loc/vive/light", "/loc/vive/trackers"]):
  # Get tracker data
  if topic == "/loc/vive/trackers":
    for tracker in msg.trackers:
      trackers[tracker.serial] = dict()
      trackers[tracker.serial]["positions"] = np.zeros((len(tracker.extrinsics),3))
      trackers[tracker.serial]["normals"] = np.zeros((len(tracker.extrinsics),3))
      trackers[tracker.serial]["references"] = np.zeros((len(tracker.extrinsics),3))
      # Save positions
      for sensor in tracker.extrinsics:
        trackers[tracker.serial]["positions"][sensor.id,0] = sensor.position.x
        trackers[tracker.serial]["positions"][sensor.id,1] = sensor.position.y
        trackers[tracker.serial]["positions"][sensor.id,2] = sensor.position.z
        trackers[tracker.serial]["normals"][sensor.id,0] = sensor.normal.x
        trackers[tracker.serial]["normals"][sensor.id,1] = sensor.normal.y
        trackers[tracker.serial]["normals"][sensor.id,2] = sensor.normal.z
        norm = np.sqrt( sensor.normal.y ** 2 + sensor.normal.x ** 2)
        trackers[tracker.serial]["references"][sensor.id,0] = sensor.normal.y / norm
        trackers[tracker.serial]["references"][sensor.id,1] = -sensor.normal.x / norm
        trackers[tracker.serial]["references"][sensor.id,2] = 0
        # print(np.dot(trackers[tracker.serial]["references"][sensor.id],trackers[tracker.serial]["normals"][sensor.id]))
  # Get light data
  if topic == "/loc/vive/light":
    # Figure out the axis
    # print(msg.lighthouse)
    if msg.axis == 0:
      light_horizontal[msg.lighthouse] = dict()
      for sample in msg.samples:
        light_horizontal[msg.lighthouse][sample.sensor] = np.tan(sample.angle)
        # print("Sensor " + str(sample.sensor))
    elif msg.axis == 1:
      light_vertical[msg.lighthouse] = dict()
      for sample in msg.samples:
        light_vertical[msg.lighthouse][sample.sensor] = np.tan(sample.angle)
        # print("Sensor " + str(sample.sensor))
    # See if there's enough data and solve the PnP problem
    if (msg.lighthouse in light_horizontal and
      msg.lighthouse in light_vertical):
      image_points = np.zeros((len(trackers[msg.header.frame_id]["positions"]),2))
      detected_sensors = list()
      for sensor in light_horizontal[msg.lighthouse]:
        if sensor in light_vertical[msg.lighthouse]:
          print(msg.lighthouse + " - H - " + str(sensor) + " : " + str(np.arctan(light_horizontal[msg.lighthouse][sensor])))
          print(msg.lighthouse + " - V - " + str(sensor) + " : " + str(np.arctan(light_vertical[msg.lighthouse][sensor])))
          image_points[sensor,0] = np.arctan(light_horizontal[msg.lighthouse][sensor])
          image_points[sensor,1] = np.arctan(light_vertical[msg.lighthouse][sensor])
          detected_sensors.append(sensor)
      if len(detected_sensors) > 3:
        # print("********")
        ret, wAAt, wPt = cv2.solvePnP(
          trackers[msg.header.frame_id]["positions"][detected_sensors],
          image_points[detected_sensors],
          np.eye(3),
          None)
        print(msg.lighthouse)
        print(wPt)
        print(wAAt)
        for sensor in detected_sensors:
          wRt = np.asmatrix(np.eye(3))
          cv2.Rodrigues(wAAt,wRt)
          tPs = np.matrix(trackers[msg.header.frame_id]["positions"][sensor,:]).transpose()
          wPs = wRt * tPs + wPt
          print(str(sensor) + " - " + str(np.arctan(light_horizontal[msg.lighthouse][sensor])) + ", " + str(np.arctan(light_vertical[msg.lighthouse][sensor])))
          print(str(sensor) + " - " + str(np.arctan(wPs[0]/wPs[2])) + ", " + str(np.arctan(wPs[1]/wPs[2])))
          print(wPs)
        print(image_points)
        print("\n")
        print(trackers[msg.header.frame_id]["positions"])
        if not msg.lighthouse in duration_horizontal:
          duration_horizontal[msg.lighthouse] = list()
          duration_vertical[msg.lighthouse] = list()
          beta_horizontal[msg.lighthouse] = list()
          alpha_horizontal_1[msg.lighthouse] = list()
          alpha_horizontal_2[msg.lighthouse] = list()
          alpha_horizontal_3[msg.lighthouse] = list()
          alpha_horizontal_4[msg.lighthouse] = list()
          beta_vertical[msg.lighthouse] = list()
          alpha_vertical_1[msg.lighthouse] = list()
          alpha_vertical_2[msg.lighthouse] = list()
          alpha_vertical_3[msg.lighthouse] = list()
          alpha_vertical_4[msg.lighthouse] = list()
          pose_x[msg.lighthouse] = list()
          pose_y[msg.lighthouse] = list()
          pose_z[msg.lighthouse] = list()
          pose_x_horizontal[msg.lighthouse] = list()
          pose_y_vertical[msg.lighthouse] = list()
          pose_z_horizontal[msg.lighthouse] = list()
          pose_z_vertical[msg.lighthouse] = list()

          wC1_x[msg.lighthouse] = list()
          wC1_y[msg.lighthouse] = list()
          wC1_z[msg.lighthouse] = list()

          wC2_x[msg.lighthouse] = list()
          wC2_y[msg.lighthouse] = list()
          wC2_z[msg.lighthouse] = list()

          wC3_x[msg.lighthouse] = list()
          wC3_y[msg.lighthouse] = list()
          wC3_z[msg.lighthouse] = list()

          wC4_x[msg.lighthouse] = list()
          wC4_y[msg.lighthouse] = list()
          wC4_z[msg.lighthouse] = list()
        # Saving stuff
        pose_x[msg.lighthouse].append(wPt[0,0])
        pose_y[msg.lighthouse].append(wPt[1,0])
        pose_z[msg.lighthouse].append(wPt[2,0])
        wRt = np.asmatrix(np.eye(3))
        cv2.Rodrigues(wAAt,wRt)
        tN = np.asmatrix(trackers[msg.header.frame_id]["normals"][thesensor].reshape((3,1)))
        tP = np.asmatrix(trackers[msg.header.frame_id]["positions"][thesensor].reshape((3,1)))
        tR = np.asmatrix(trackers[msg.header.frame_id]["references"][thesensor].reshape((3,1)))
        # Reference rotation
        tRr = np.asmatrix(np.eye(3))
        cv2.Rodrigues(trackers[msg.header.frame_id]["normals"][thesensor] * gamma, tRr)
        # Direction of the first reference
        td1 = tRr * tR
        td2 = np.asmatrix(np.cross(np.asarray(td1).reshape(3),  np.asarray(tN).reshape(3)).reshape((3,1)))
        # Corners of photodiode
        tC1 = (side / 2) * (td1 + td2) + tP
        wC1 = np.asarray(wRt * tC1 + wPt).reshape(3)
        wC1_x[msg.lighthouse].append(wC1[0])
        wC1_y[msg.lighthouse].append(wC1[1])
        wC1_z[msg.lighthouse].append(wC1[2])

        tC2 = (side / 2) * (-td1 - td2) + tP
        wC2 = np.asarray(wRt * tC2 + wPt).reshape(3)
        wC2_x[msg.lighthouse].append(wC2[0])
        wC2_y[msg.lighthouse].append(wC2[1])
        wC2_z[msg.lighthouse].append(wC2[2])

        tC3 = (side / 2) * (-td1 + td2) + tP
        wC3 = np.asarray(wRt * tC3 + wPt).reshape(3)
        wC3_x[msg.lighthouse].append(wC3[0])
        wC3_y[msg.lighthouse].append(wC3[1])
        wC3_z[msg.lighthouse].append(wC3[2])

        tC4 = (side / 2) * (td1 - td2) + tP
        wC4 = np.asarray(wRt * tC4 + wPt).reshape(3)
        wC4_x[msg.lighthouse].append(wC4[0])
        wC4_y[msg.lighthouse].append(wC4[1])
        wC4_z[msg.lighthouse].append(wC4[2])

        tC_list = [wC1, wC2, wC3, wC4]
        # print(msg.lighthouse)
        # print(tC_list)

        if thesensor in detected_sensors:
          thecounter += 1
          if msg.axis == 0:
            beta1 = np.arctan2(wC1[0], wC1[2])
            beta2 = np.arctan2(wC2[0], wC2[2])
            beta3 = np.arctan2(wC3[0], wC3[2])
            beta4 = np.arctan2(wC4[0], wC4[2])
            beta_list = [beta1, beta2, beta3, beta4]
            # print(beta_list)
            # sys.exit(0)
            beta = max(beta_list) - min(beta_list)
            beta_horizontal[msg.lighthouse].append(beta * 10e3)
            alpha_horizontal_1[msg.lighthouse].append(beta1)
            alpha_horizontal_2[msg.lighthouse].append(beta2)
            alpha_horizontal_3[msg.lighthouse].append(beta3)
            alpha_horizontal_4[msg.lighthouse].append(beta4)
            pose_x_horizontal[msg.lighthouse].append(wPt[0,0])
            pose_z_horizontal[msg.lighthouse].append(wPt[2,0])
            for sample in msg.samples:
              if sample.sensor == thesensor:
                duration_horizontal[msg.lighthouse].append(sample.length)
                break
          if msg.axis == 1:
            beta1 = np.arctan2(wC1[1], wC1[2])
            beta2 = np.arctan2(wC2[1], wC2[2])
            beta3 = np.arctan2(wC3[1], wC3[2])
            beta4 = np.arctan2(wC4[1], wC4[2])
            beta_list = [beta1, beta2, beta3, beta4]
            # print(beta_list)
            # sys.exit(0)
            beta = max(beta_list) - min(beta_list)
            beta_vertical[msg.lighthouse].append(beta * 10e3)
            alpha_vertical_1[msg.lighthouse].append(beta1)
            alpha_vertical_2[msg.lighthouse].append(beta2)
            alpha_vertical_3[msg.lighthouse].append(beta3)
            alpha_vertical_4[msg.lighthouse].append(beta4)
            pose_y_vertical[msg.lighthouse].append(wPt[1,0])
            pose_z_vertical[msg.lighthouse].append(wPt[2,0])
            for sample in msg.samples:
              if sample.sensor == thesensor:
                duration_vertical[msg.lighthouse].append(sample.length)
                break
        else:
          if msg.axis == 0:
            pose_x_horizontal[msg.lighthouse].append(wPt[0,0])
            pose_z_horizontal[msg.lighthouse].append(wPt[2,0])
            duration_horizontal[msg.lighthouse].append(0.0)
            beta_horizontal[msg.lighthouse].append(0.0)
            alpha_horizontal_1[msg.lighthouse].append(0.0)
            alpha_horizontal_2[msg.lighthouse].append(0.0)
            alpha_horizontal_3[msg.lighthouse].append(0.0)
            alpha_horizontal_4[msg.lighthouse].append(0.0)
          if msg.axis == 1:
            pose_y_vertical[msg.lighthouse].append(wPt[1,0])
            pose_z_vertical[msg.lighthouse].append(wPt[2,0])
            duration_vertical[msg.lighthouse].append(0.0)
            beta_vertical[msg.lighthouse].append(0.0)
            alpha_vertical_1[msg.lighthouse].append(0.0)
            alpha_vertical_2[msg.lighthouse].append(0.0)
            alpha_vertical_3[msg.lighthouse].append(0.0)
            alpha_vertical_4[msg.lighthouse].append(0.0)
        if thecounter >= 1:
          sys.exit(0)
          pass

lh_1 = beta_vertical.keys()[0]
lh_2 = beta_vertical.keys()[1]

# Plot all the alphas, betas and positions the axis.
# fig1, axs = plt.subplots(2, 2, sharex = True, sharey= False)
# axs[0,0].plot(wC1_x[lh_1])
# axs[0,0].plot(wC1_y[lh_1])
# axs[0,0].plot(wC1_z[lh_1])

# axs[0,1].plot(wC2_x[lh_1])
# axs[0,1].plot(wC2_y[lh_1])
# axs[0,1].plot(wC2_z[lh_1])

# axs[1,0].plot(wC3_x[lh_1])
# axs[1,0].plot(wC3_y[lh_1])
# axs[1,0].plot(wC3_z[lh_1])

# axs[1,1].plot(wC4_x[lh_1])
# axs[1,1].plot(wC4_y[lh_1])
# axs[1,1].plot(wC4_z[lh_1])

fig2, axs = plt.subplots(3, 2, sharex = True, sharey= False)
axs[0,0].plot(alpha_horizontal_1[lh_1])
axs[0,0].plot(alpha_horizontal_2[lh_1])
axs[0,0].plot(alpha_horizontal_3[lh_1])
axs[0,0].plot(alpha_horizontal_4[lh_1])
axs[0,0].legend(['1','2','3','4'])
axs[1,0].plot(pose_x_horizontal[lh_1])
axs[1,0].plot(pose_z_vertical[lh_1])
axs[1,0].legend(['x','z'])
axs[2,0].plot(duration_horizontal[lh_1])
axs[2,0].plot(beta_horizontal[lh_1])
axs[2,0].set_ylim([0,50])

axs[0,1].plot(alpha_vertical_1[lh_1])
axs[0,1].plot(alpha_vertical_2[lh_1])
axs[0,1].plot(alpha_vertical_3[lh_1])
axs[0,1].plot(alpha_vertical_4[lh_1])
axs[0,1].legend(['1','2','3','4'])
axs[1,1].plot(pose_y_vertical[lh_1])
axs[1,1].plot(pose_z_vertical[lh_1])
axs[1,1].legend(['y','z'])
axs[2,1].plot(duration_vertical[lh_1])
axs[2,1].plot(beta_vertical[lh_1])
axs[2,1].set_ylim([0,50])

# fig, axs = plt.subplots(3,2)
# axs[0,0].plot(beta_horizontal[lh_1])
# axs[0,0].plot(duration_horizontal[lh_1])
# axs[0,0].legend(['Predicted','Real'])
# axs[0,0].set_ylim([0,50])
# axs[0,0].set_title("Horizontal Sweep")
# axs[1,0].plot(beta_vertical[lh_1])
# axs[1,0].plot(duration_vertical[lh_1])
# axs[1,0].legend(['Predicted','Real'])
# axs[1,0].set_ylim([0,50])
# axs[1,0].set_title("Vertical Sweep")
# axs[2,0].plot(pose_x[lh_1])
# axs[2,0].plot(pose_y[lh_1])
# axs[2,0].plot(pose_z[lh_1])
# axs[2,0].legend(['x','y','z'])
# axs[2,0].set_title("Positions")
# axs[0,1].plot(alpha_horizontal_1[lh_1])
# axs[0,1].plot(alpha_horizontal_2[lh_1])
# axs[0,1].plot(alpha_horizontal_3[lh_1])
# axs[0,1].plot(alpha_horizontal_4[lh_1])
# axs[0,1].legend(['1','2','3','4'])
# axs[1,1].plot(alpha_vertical_1[lh_1])
# axs[1,1].plot(alpha_vertical_2[lh_1])
# axs[1,1].plot(alpha_vertical_3[lh_1])
# axs[1,1].plot(alpha_vertical_4[lh_1])
# axs[1,1].legend(['1','2','3','4'])

plt.show()

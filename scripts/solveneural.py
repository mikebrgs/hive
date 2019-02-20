# import sklearn

# P1 = [-0.080181,-0.084760,2.12597,0.092712,-2.49243,1.73064]
# BV1 = 9.9375
# BH1 = 11.1458
# P2 = [0.18537,0.055454,0.966796,1.05216,-1.58355,1.11018]
# BV2 = 12.8542
# BH2 = 10.2083
# P3 = [0.0417925,-0.0289449,1.15421,0.050341,2.41822,-1.50874]
# BV3 = 18.825
# BH3 = 19.625
# P4 = [0.050507,0.001338,0.961665,0.177544,2.32914,-1.81053]
# BV4 = 22.3542
# BH4 = 23.875

# Ps = [P1,P2,P3,P4]
# BV = [BV1,BV2,BV3,BV4]
# BH = [BH1,BH2,BH3,BH4]
from __future__ import absolute_import, division, print_function

# import pathlib

import pandas as pd
import seaborn as sns

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

import matplotlib.pyplot as plt

# ROS imports
import rosbag
import rospy
import cv2

import numpy as np
import sys

args = sys.argv

if len(args) < 2:
  print("Please insert RosVive data file")
  sys.exit(0)

bag = rosbag.Bag(args[1], "r")

SENSOR = 7

trackers = dict()
light_horizontal = dict()
light_vertical = dict()
input_hdata = np.empty([0,6])
input_vdata = np.empty([0,6])
output_hdata = np.empty([0,1])
output_vdata = np.empty([0,1])

counter = 0
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
          image_points[sensor,0] = np.arctan(light_horizontal[msg.lighthouse][sensor])
          image_points[sensor,1] = np.arctan(light_vertical[msg.lighthouse][sensor])
          detected_sensors.append(sensor)
      if len(detected_sensors) > 3:
        # print("********")
        ret, lAt, lPt = cv2.solvePnP(trackers[msg.header.frame_id]["positions"][detected_sensors],
          image_points[detected_sensors].reshape(image_points[detected_sensors].shape[0],1,2),
          np.eye(3),
          None,
          flags = cv2.SOLVEPNP_UPNP)
        lRt = np.asmatrix(np.eye(3))
        cv2.Rodrigues(lAt,lRt)

        for sample in msg.samples:
          if SENSOR == sample.sensor:
            if msg.axis == 1:
              pose = np.array([lPt[0][0], lPt[1][0], lPt[2][0], lAt[0][0], lAt[1][0], lAt[2][0]])
              length = np.array([sample.length])
              input_vdata = np.vstack((input_vdata, pose))
              output_vdata = np.vstack((output_vdata, length))
            elif msg.axis == 0:
              pose = np.array([lPt[0][0], lPt[1][0], lPt[2][0], lAt[0][0], lAt[1][0], lAt[2][0]])
              length = np.array([sample.length])
              input_hdata = np.vstack((input_hdata, pose))
              output_hdata = np.vstack((output_hdata, length))

def build_model():
  model = keras.Sequential([
    layers.Dense(6, activation=tf.nn.sigmoid, input_shape=(6,)),
    layers.Dense(6, activation=tf.nn.sigmoid),
    layers.Dense(1)
  ])

  optimizer = tf.keras.optimizers.RMSprop(0.001)

  model.compile(loss='mse',
                optimizer=optimizer,
                metrics=['mae', 'mse'])
  return model

horizontal_model = build_model()
vertical_model = build_model()

EPOCHS = 10000
history = horizontal_model.fit(
  input_hdata, output_hdata,
  epochs=EPOCHS, validation_split = 0.0,
  verbose=True, batch_size=100)
history = vertical_model.fit(
  input_vdata, output_vdata,
  epochs=EPOCHS, validation_split = 0.0,
  verbose=True, batch_size=100)

predicted_hdata = horizontal_model.predict(input_hdata)
predicted_vdata = vertical_model.predict(input_vdata)

fig, axs = plt.subplots(2, 1, sharex = False, sharey= False)
axs[0].scatter(predicted_hdata, output_hdata)
axs[1].scatter(predicted_vdata, output_vdata)
plt.show()

fig, axs = plt.subplots(2, 1, sharex = False, sharey= False)
axs[0].plot(predicted_hdata)
axs[0].plot(output_hdata)
axs[1].plot(predicted_vdata)
axs[1].plot(output_vdata)
plt.show()

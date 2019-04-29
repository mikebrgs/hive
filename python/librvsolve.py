import numpy as np
import libsolve
import sys
import rospy
import cv2
import random

def trackerhandler(msg, data):
  # data is dict
  for tracker in msg.trackers:
    data[tracker.serial] = dict()
    data[tracker.serial]["positions"] = np.zeros((len(tracker.extrinsics),3))
    data[tracker.serial]["normals"] = np.zeros((len(tracker.extrinsics),3))
    # Save positions
    for sensor in tracker.extrinsics:
      data[tracker.serial]["positions"][sensor.id,0] = sensor.position.x
      data[tracker.serial]["positions"][sensor.id,1] = sensor.position.y
      data[tracker.serial]["positions"][sensor.id,2] = sensor.position.z
      data[tracker.serial]["normals"][sensor.id,0] = sensor.normal.x
      data[tracker.serial]["normals"][sensor.id,1] = sensor.normal.y
      data[tracker.serial]["normals"][sensor.id,2] = sensor.normal.z
  return

def lighthousehandler(msg, data):
  for lighthouse in msg.lighthouses:
    data.append(lighthouse.serial)
  return

class RAP3P(libsolve.Solver):
  """docstring for Solver"""
  def __init__(self, trackers):
    super(RAP3P, self).__init__()
    self.trackers = trackers

  def handler(self, msg, data):
    if msg.axis == 0:
      self.light_horizontal[msg.lighthouse] = dict()
      for sample in msg.samples:
        self.light_horizontal[msg.lighthouse][sample.sensor] = np.tan(sample.angle)
    elif msg.axis == 1:
      self.light_vertical[msg.lighthouse] = dict()
      for sample in msg.samples:
        self.light_vertical[msg.lighthouse][sample.sensor] = np.tan(sample.angle)
    # See if there's enough data and solve the PnP problem
    if (msg.lighthouse in self.light_horizontal and
      msg.lighthouse in self.light_vertical):
      image_points = np.zeros((len(self.trackers[msg.header.frame_id]["positions"]),2))
      detected_sensors = list()
      for sensor in self.light_horizontal[msg.lighthouse]:
        if sensor in self.light_vertical[msg.lighthouse]:
          image_points[sensor,0] = self.light_horizontal[msg.lighthouse][sensor]
          image_points[sensor,1] = self.light_vertical[msg.lighthouse][sensor]
          detected_sensors.append(sensor)
      if len(detected_sensors) > 3:
        # RANSAC style approach
        best_error = 9e99
        best_sensors = None
        for index in range(0,3):
          saved_sensors = list()
          for sensor in range(0,4):
            sensor = random.randrange(0,len(detected_sensors))
            while sensor in saved_sensors:
              sensor = random.randrange(0,len(detected_sensors))
            saved_sensors.append(sensor)
          want_sensors = list( detected_sensors[i] for i in saved_sensors )
          ret, lAt, lPt = cv2.solvePnP(self.trackers[msg.header.frame_id]["positions"][want_sensors],
            image_points[want_sensors].reshape(image_points[want_sensors].shape[0],1,2),
            np.eye(3),
            None,
            flags = cv2.SOLVEPNP_AP3P)
          new_error = 0
          lRt = np.asmatrix(np.eye(3))
          cv2.Rodrigues(lAt,lRt)
          for sensor in detected_sensors:
            tPs = np.asmatrix(self.trackers[msg.header.frame_id]["positions"][sensor]).transpose()
            lPs = lRt * tPs + lPt
            new_error += abs(lPs[0]/lPs[2] - image_points[sensor,0])
            new_error += abs(lPs[1]/lPs[2] - image_points[sensor,1])
          if new_error < best_error:
            best_error = new_error
            best_sensors = want_sensors;
        if best_error / 2 * 4 > 0.01:
          return
        ret, wAAt, wPt = cv2.solvePnP(self.trackers[msg.header.frame_id]["positions"][best_sensors],
          image_points[best_sensors].reshape(image_points[best_sensors].shape[0],1,2),
          np.eye(3),
          None,
          flags = cv2.SOLVEPNP_P3P)

        SENSOR = 6
        for sample in msg.samples:
          if SENSOR == sample.sensor:
            if msg.axis == 1:
              # Normal
              tNs = self.trackers[msg.header.frame_id]["normals"][sensor].reshape((3,1))
              # Position
              tPs = self.trackers[msg.header.frame_id]["positions"][sensor].reshape((3,1))
              # Rerefence
              tH1 = np.matrix([tNs[1,0], - tNs[0,0], 0.0]).transpose()
              tH1 = tH1 / np.linalg.norm(tH1)
              tH2 = np.matrix([tNs[1,0] * tH1[2,0] - tNs[2,0] * tH1[1,0],
                tNs[2,0] * tH1[0,0] - tNs[0,0] * tH1[2,0],
                tNs[0,0] * tH1[1,0] - tNs[1,0] * tH1[0,0]]).transpose()
              lH1 = lRt * tH1
              f1 = lH1.transpose() * tH1
              f2 = lH1.transpose() * tH2
              lH1_corrected = f1[0,0] * tH1 + f2[0,0] * tH2
              lH1_corrected = lH1_corrected / np.linalg.norm(lH1_corrected)
              angle2 = np.arccos(lH1_corrected.transpose() * tH1)[0,0]
              lPs = lRt * tPs + lPt
              lNs = lRt * tNs
              lNs_normalized = lNs / np.linalg.norm(lNs)
              lPs_normalized = lPs / np.linalg.norm(lPs)
              distance = np.sqrt(lPs[0,0]**2 + lPs[1,0]**2 + lPs[2,0]**2)
              angle1 = np.arccos(lPs_normalized.transpose() * lNs_normalized)[0,0]
              features = np.array([distance, angle1, angle2])
              length = np.array([sample.length])
              data[2] = np.vstack((data[2], features))
              data[3] = np.vstack((data[3], length))
            elif msg.axis == 0:
              tNs = self.trackers[msg.header.frame_id]["normals"][sensor].reshape((3,1))
              tPs = self.trackers[msg.header.frame_id]["positions"][sensor].reshape((3,1))
              lPs = lRt * tPs + lPt
              tH1 = np.matrix([tNs[1,0], - tNs[0,0], 0.0]).transpose()
              tH1 = tH1 / np.linalg.norm(tH1)
              tH2 = np.matrix([tNs[1,0] * tH1[2,0] - tNs[2,0] * tH1[1,0],
                tNs[2,0] * tH1[0,0] - tNs[0,0] * tH1[2,0],
                tNs[0,0] * tH1[1,0] - tNs[1,0] * tH1[0,0]]).transpose()
              lH1 = lRt * tH1
              f1 = lH1.transpose() * tH1
              f2 = lH1.transpose() * tH2
              lH1_corrected = f1[0,0] * tH1 + f2[0,0] * tH2
              lH1_corrected = lH1_corrected / np.linalg.norm(lH1_corrected)
              angle2 = np.arccos(lH1_corrected.transpose() * tH1)[0,0]
              lNs = lRt * tNs
              lNs_normalized = lNs / np.linalg.norm(lNs)
              lPs_normalized = lPs / np.linalg.norm(lPs)
              distance = np.sqrt(lPs[0,0]**2 + lPs[1,0]**2 + lPs[2,0]**2)
              angle1 = np.arccos(lPs_normalized.transpose() * lNs_normalized)[0,0]
              features = np.array([distance, angle1, angle2])
              length = np.array([sample.length])
              data[0] = np.vstack((data[0], features))
              data[1] = np.vstack((data[1], length))
    return

class LightReader(libsolve.BagReader):
  """docstring for LightReader"""
  def __init__(self, filename, trackers):
    super(LightReader, self).__init__(filename)
    self.solver = RAP3P(trackers)

  def read(self, topic, data):
    for t, msg, _ in self.bag.read_messages(topics = topic):
      self.solver.handler(msg, data)

def main(args):
  trackers = dict()
  br = libsolve.BagReader(args[1])
  br.read("/loc/vive/trackers", trackerhandler, trackers)
  data = [np.empty([0,6]),
    np.empty([0,1]),
    np.empty([0,6]),
    np.empty([0,1])]
  lr = LightReader(args[1], trackers)
  lr.read("/loc/vive/light", data)
  pass

if __name__ == '__main__':
  main(sys.argv)
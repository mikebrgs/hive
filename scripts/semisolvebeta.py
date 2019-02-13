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

thesensor = 21
theangle = 6.05619
theside = 0.00198385

vpbeta = dict() # predicted
hpbeta = dict() # predicted
vrbeta = dict() # real
hrbeta = dict() # real

trackers = dict()
for topic, msg, t in bag.read_messages(topics=["/trackers"]):
  for tracker in msg.trackers:
    vpbeta[tracker.serial] = dict()
    hpbeta[tracker.serial] = dict()
    vrbeta[tracker.serial] = dict()
    hrbeta[tracker.serial] = dict()

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

for topic, msg, t in bag.read_messages(topics=["/tdata"]):
  if msg.lighthouse not in vpbeta[msg.tracker]:
    vpbeta[msg.tracker][msg.lighthouse] = list()
    vrbeta[msg.tracker][msg.lighthouse] = list()
  if msg.lighthouse not in hpbeta[msg.tracker]:
    hpbeta[msg.tracker][msg.lighthouse] = list()
    hrbeta[msg.tracker][msg.lighthouse] = list()
  lPt = np.matrix([msg.position.x,
    msg.position.y,
    msg.position.z]).transpose()
  lAt = np.matrix([msg.axisangle.x,
    msg.axisangle.y,
    msg.axisangle.z])
  lRt = np.asmatrix(np.eye(3))
  cv2.Rodrigues(lAt, lRt)
  for sample in msg.samples:
    if sample.sensor == thesensor:
      tN = np.asmatrix(trackers[msg.tracker]["normals"][thesensor].reshape((3,1)))
      tP = np.asmatrix(trackers[msg.tracker]["positions"][thesensor].reshape((3,1)))
      tR = np.asmatrix(trackers[msg.tracker]["references"][thesensor].reshape((3,1)))

      tRr = np.asmatrix(np.eye(3))
      cv2.Rodrigues(trackers[msg.tracker]["normals"][thesensor] * theangle, tRr)

      td1 = tRr * tR
      td2 = np.asmatrix(np.cross(np.asarray(td1).reshape(3),  np.asarray(tN).reshape(3)).reshape((3,1)))
      # Corners of photodiode
      tC1 = (theside / 2) * (td1 + td2) + tP
      lC1 = np.asarray(lRt * tC1 + lPt).reshape(3)
      tC2 = (theside / 2) * (-td1 - td2) + tP
      lC2 = np.asarray(lRt * tC2 + lPt).reshape(3)
      tC3 = (theside / 2) * (-td1 + td2) + tP
      lC3 = np.asarray(lRt * tC3 + lPt).reshape(3)
      tC4 = (theside / 2) * (td1 - td2) + tP
      lC4 = np.asarray(lRt * tC4 + lPt).reshape(3)
      tC_list = [lC1, lC2, lC3, lC4]
      if msg.axis == 0:
        beta1 = np.arctan2(lC1[0], lC1[2])
        beta2 = np.arctan2(lC2[0], lC2[2])
        beta3 = np.arctan2(lC3[0], lC3[2])
        beta4 = np.arctan2(lC4[0], lC4[2])
        beta_list = [beta1, beta2, beta3, beta4]
        beta = max(beta_list) - min(beta_list)
        hpbeta[msg.tracker][msg.lighthouse].append(beta * 10e3)
        hrbeta[msg.tracker][msg.lighthouse].append(sample.length)
      elif msg.axis == 1:
        beta1 = np.arctan2(lC1[1], lC1[2])
        beta2 = np.arctan2(lC2[1], lC2[2])
        beta3 = np.arctan2(lC3[1], lC3[2])
        beta4 = np.arctan2(lC4[1], lC4[2])
        beta_list = [beta1, beta2, beta3, beta4]
        beta = max(beta_list) - min(beta_list)
        vpbeta[msg.tracker][msg.lighthouse].append(beta * 10e3)
        vrbeta[msg.tracker][msg.lighthouse].append(sample.length)

tr_1 = vpbeta.keys()[0]
lh_1 = vpbeta[tr_1].keys()[0]
lh_2 = vpbeta[tr_1].keys()[1]

fig1, axs = plt.subplots(2, 2, sharex = True, sharey= False)
axs[0,0].plot(hpbeta[tr_1][lh_1])
axs[0,0].plot(hrbeta[tr_1][lh_1])
axs[0,0].legend(['LH1 - HP', 'LH1 - HR'])
axs[0,0].set_ylim([0,50])
axs[1,0].plot(vpbeta[tr_1][lh_1])
axs[1,0].plot(vrbeta[tr_1][lh_1])
axs[1,0].legend(['LH1 - VP', 'LH1 - VR'])
axs[1,0].set_ylim([0,50])
axs[0,1].plot(hpbeta[tr_1][lh_2])
axs[0,1].plot(hrbeta[tr_1][lh_2])
axs[0,1].legend(['LH2 - HP', 'LH2 - HR'])
axs[0,1].set_ylim([0,50])
axs[1,1].plot(vpbeta[tr_1][lh_2])
axs[1,1].plot(vrbeta[tr_1][lh_2])
axs[1,1].legend(['LH2 - VP', 'LH2 - VR'])
axs[1,1].set_ylim([0,50])


plt.show()
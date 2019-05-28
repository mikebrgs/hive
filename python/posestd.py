import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

args = sys.argv

if len(args) < 2:
  print("Please insert RosVive data file")
  sys.exit(0)


positions = np.empty((3,0))
attitudes = np.empty((3,0))
bag = rosbag.Bag(args[1], "r")
counter = 0
for topic, msg, t in bag.read_messages(topics=["tf", "/tf"]):
  # if msg != None:
  #   smsg = msg
  for smsg in msg.transforms:
    # print(smsg)
    if smsg.child_frame_id == "LHR-09DF88FD":
      positions = np.hstack((positions,np.array([[smsg.transform.translation.x,
        smsg.transform.translation.y,
        smsg.transform.translation.z]]).transpose()))
      qw = smsg.transform.rotation.w
      qx = smsg.transform.rotation.x
      qy = smsg.transform.rotation.y
      qz = smsg.transform.rotation.z
      angle = 2*np.arccos(qw)
      if angle < 0:
        angle = angle + 2*np.pi
      attitudes = np.hstack((attitudes,np.array([[angle * qx / np.sqrt(1-qw*qw),
        angle * qy / np.sqrt(1-qw*qw),
        angle * qz / np.sqrt(1-qw*qw)]]).transpose()))
      counter += 1
if counter >= 2:
  print(np.sqrt(max(np.linalg.eig(np.cov(positions))[0])))
# fig, axs = plt.subplots(3, 1, sharex = False, sharey= False)
# axs[0].plot(attitudes[0,:])
# axs[1].plot(attitudes[1,:])
# axs[2].plot(attitudes[2,:])
# plt.show()
# print(np.linalg.eig(np.cov(attitudes))[0])
  print(np.sqrt(max(np.linalg.eig(np.cov(attitudes))[0])))
  print(np.sqrt(max(np.linalg.eig(np.cov(attitudes))[0]))*180/np.pi)

positions = np.empty((3,0))
attitudes = np.empty((3,0))
bag = rosbag.Bag(args[1], "r")
counter = 0
for topic, msg, t in bag.read_messages(topics=["tf", "/tf"]):
  for smsg in msg.transforms:
    # print(smsg)
    if smsg.child_frame_id == "LHR-08DE340B":
      positions = np.hstack((positions,np.array([[smsg.transform.translation.x,
        smsg.transform.translation.y,
        smsg.transform.translation.z]]).transpose()))
      qw = smsg.transform.rotation.w
      qx = smsg.transform.rotation.x
      qy = smsg.transform.rotation.y
      qz = smsg.transform.rotation.z
      angle = 2*np.arccos(qw)
      attitudes = np.hstack((attitudes,np.array([[angle * qx / np.sqrt(1-qw*qw),
        angle * qy / np.sqrt(1-qw*qw),
        angle * qz / np.sqrt(1-qw*qw)]]).transpose()))
      counter += 1
if counter >= 2:
  print(np.sqrt(max(np.linalg.eig(np.cov(positions))[0])))
  print(np.sqrt(max(np.linalg.eig(np.cov(attitudes))[0]))*180/np.pi)

positions = np.empty((3,0))
attitudes = np.empty((3,0))
bag = rosbag.Bag(args[1], "r")
counter = 0
for topic, msg, t in bag.read_messages(topics=["tf", "/tf"]):
  for smsg in msg.transforms:
    # print(smsg)
    if smsg.child_frame_id == "LHR-08DDBC05":
      positions = np.hstack((positions,np.array([[smsg.transform.translation.x,
        smsg.transform.translation.y,
        smsg.transform.translation.z]]).transpose()))
      qw = smsg.transform.rotation.w
      qx = smsg.transform.rotation.x
      qy = smsg.transform.rotation.y
      qz = smsg.transform.rotation.z
      angle = 2*np.arccos(qw)
      attitudes = np.hstack((attitudes,np.array([[angle * qx / np.sqrt(1-qw*qw),
        angle * qy / np.sqrt(1-qw*qw),
        angle * qz / np.sqrt(1-qw*qw)]]).transpose()))
      counter += 1
if counter >= 2:
  print(np.sqrt(max(np.linalg.eig(np.cov(positions))[0])))
  print(np.sqrt(max(np.linalg.eig(np.cov(attitudes))[0]))*180/np.pi)


positions = np.empty((3,0))
attitudes = np.empty((3,0))
bag = rosbag.Bag(args[1], "r")
counter = 0
for topic, msg, t in bag.read_messages(topics=["tf", "/tf"]):
  for smsg in msg.transforms:
    # print(smsg)
    if smsg.child_frame_id == "tracker1":
      positions = np.hstack((positions,np.array([[smsg.transform.translation.x,
        smsg.transform.translation.y,
        smsg.transform.translation.z]]).transpose()))
      qw = smsg.transform.rotation.w
      qx = smsg.transform.rotation.x
      qy = smsg.transform.rotation.y
      qz = smsg.transform.rotation.z
      angle = 2*np.arccos(qw)
      attitudes = np.hstack((attitudes,np.array([[angle * qx / np.sqrt(1-qw*qw),
        angle * qy / np.sqrt(1-qw*qw),
        angle * qz / np.sqrt(1-qw*qw)]]).transpose()))
      counter += 1
if counter >= 2:
  print(np.sqrt(max(np.linalg.eig(np.cov(positions))[0])))
  print(np.sqrt(max(np.linalg.eig(np.cov(attitudes))[0]))*180/np.pi)

positions = np.empty((3,0))
attitudes = np.empty((3,0))
bag = rosbag.Bag(args[1], "r")
counter = 0
for topic, msg, t in bag.read_messages(topics=["tf", "/tf"]):
  for smsg in msg.transforms:
    # print(smsg)
    if smsg.child_frame_id == "controller1":
      positions = np.hstack((positions,np.array([[smsg.transform.translation.x,
        smsg.transform.translation.y,
        smsg.transform.translation.z]]).transpose()))
      qw = smsg.transform.rotation.w
      qx = smsg.transform.rotation.x
      qy = smsg.transform.rotation.y
      qz = smsg.transform.rotation.z
      angle = 2*np.arccos(qw)
      attitudes = np.hstack((attitudes,np.array([[angle * qx / np.sqrt(1-qw*qw),
        angle * qy / np.sqrt(1-qw*qw),
        angle * qz / np.sqrt(1-qw*qw)]]).transpose()))
      counter += 1
if counter >= 2:
  print(np.sqrt(max(np.linalg.eig(np.cov(positions))[0])))
  print(np.sqrt(max(np.linalg.eig(np.cov(attitudes))[0]))*180/np.pi)

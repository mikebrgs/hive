import tf
import sys
import rospy
import rosbag
import numpy as np
import tf.transformations
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

offset_bag = rosbag.Bag(sys.argv[1])
bag = rosbag.Bag(sys.argv[2])
vive_poses = (list(), list(), list())
optitrack_poses = (list(), list(), list())

vRt = None
vPt = None
oRa = None
oPa = None
oRv = None
oPv = None
aRt = None
aPt = None

for topic, msg, t in offset_bag.read_messages(topics=['/offset', 'offset']):
  if msg.header.frame_id == "optitrack":
    oPv = np.matrix([msg.transform.translation.x,
      msg.transform.translation.y,
      msg.transform.translation.z]).transpose()
    oRv = np.matrix(tf.transformations.quaternion_matrix([msg.transform.rotation.x,
      msg.transform.rotation.y,
      msg.transform.rotation.z,
      msg.transform.rotation.w])[0:3,0:3])
    # print(oRv)
  elif msg.header.frame_id == "arrow":
    aPt = np.matrix([msg.transform.translation.x,
      msg.transform.translation.y,
      msg.transform.translation.z]).transpose()
    aRt = np.matrix(tf.transformations.quaternion_matrix([msg.transform.rotation.x,
      msg.transform.rotation.y,
      msg.transform.rotation.z,
      msg.transform.rotation.w])[0:3,0:3])
    # print(aRt)

for topic, msg, t in bag.read_messages(topics=['/tf', 'tf']):
  if msg.header.frame_id == "vive":
    vPt = np.matrix([msg.transform.translation.x,
      msg.transform.translation.y,
      msg.transform.translation.z]).transpose()
    vRt = np.matrix(tf.transformations.quaternion_matrix([msg.transform.rotation.x,
      msg.transform.rotation.y,
      msg.transform.rotation.z,
      msg.transform.rotation.w])[0:3,0:3])
    vive_poses[0].append(float(vPt[0]))
    vive_poses[1].append(float(vPt[1]))
    vive_poses[2].append(float(vPt[2]))
    # print(vRt)
    # pose = np.array([[msg.transform.translation.x,
    #   msg.transform.translation.y,
    #   msg.transform.translation.z,
    #   msg.transform.rotation.w,
    #   msg.transform.rotation.x,
    #   msg.transform.rotation.y,
    #   msg.transform.rotation.z]]).transpose()
    # vive_poses = np.hstack((vive_poses,pose))
  else:
    oPa = np.matrix([msg.transform.translation.x,
      msg.transform.translation.y,
      msg.transform.translation.z]).transpose()
    oRa = np.matrix(tf.transformations.quaternion_matrix([msg.transform.rotation.x,
      msg.transform.rotation.y,
      msg.transform.rotation.z,
      msg.transform.rotation.w])[0:3,0:3])
    # opti_vPt = oRv.transpose() * oRa * aRt
    opti_vPt = oRv.transpose() * (oRa * aPt + oPa) + (-oRv.transpose() * oPv)
    optitrack_poses[0].append(float(opti_vPt[0]))
    optitrack_poses[1].append(float(opti_vPt[1]))
    optitrack_poses[2].append(float(opti_vPt[2]))
    # print(oRa)
    # pose = np.array([[msg.transform.translation.x,
    #   msg.transform.translation.y,
    #   msg.transform.translation.z,
    #   msg.transform.rotation.w,
    #   msg.transform.rotation.x,
    #   msg.transform.rotation.y,
    #   msg.transform.rotation.z]]).transpose()
    # optitrack_poses = np.hstack((optitrack_poses,pose))
  # if not (oRa is None) and not (aRt is None) and not (vRt is None) and not (oRv is None):
  #   test_oRv = oRa * aRt * vRt.transpose();
  #   test_oPv = oRa * (aRt * (-vRt.transpose() * vPt) + aPt) + oPa
  #   print(oRv)
  #   print(test_oRv)
  #   print(" ")


print(vive_poses[0])
print("***")
print(vive_poses[1])
print("***")
print(vive_poses[2])
print("***")

print(optitrack_poses[0][10:-10])
print("***")
print(optitrack_poses[1][10:-10])
print("***")
print(optitrack_poses[2][10:-10])
print("***")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(xs = vive_poses[0], ys = vive_poses[1], zs = vive_poses[2])
ax.plot(xs = optitrack_poses[0][10:-10], ys = optitrack_poses[1][10:-10], zs = optitrack_poses[2][10:-10])

# plt.show()

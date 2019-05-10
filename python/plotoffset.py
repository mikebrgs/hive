import tf
import sys
import rospy
import rosbag
import numpy as np
import tf.transformations
import matplotlib.pyplot as plt

offset_bag = rosbag.Bag(sys.argv[1])
bag = rosbag.Bag(sys.argv[2])
vive_poses = np.empty((7,0))
optitrack_poses = np.empty((7,0))

vRt = None
oRa = None
oRv = None
aRt = None
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
    # print(oRa)
    # pose = np.array([[msg.transform.translation.x,
    #   msg.transform.translation.y,
    #   msg.transform.translation.z,
    #   msg.transform.rotation.w,
    #   msg.transform.rotation.x,
    #   msg.transform.rotation.y,
    #   msg.transform.rotation.z]]).transpose()
    # optitrack_poses = np.hstack((optitrack_poses,pose))
  if not (oRa is None) and not (aRt is None) and not (vRt is None) and not (oRv is None):
    test_oRv = oRa * aRt * vRt.transpose();
    test_oPv = oRa * (aRt * (-vRt.transpose() * vPt) + aPt) + oPa
    print(oRv)
    print(test_oRv)
    print(" ")

# fig2, axs = plt.subplots(4, 2, sharex = True, sharey= False)

# axs[0,0].plot(poses[0,:])
# axs[0,0].set_title("PX")
# axs[1,0].plot(poses[1,:])
# axs[1,0].set_title("PY")
# axs[2,0].plot(poses[2,:])
# axs[2,0].set_title("PZ")
# axs[0,1].plot(poses[3,:])
# axs[0,1].set_title("QW")
# axs[1,1].plot(poses[4,:])
# axs[1,1].set_title("QX")
# axs[2,1].plot(poses[5,:])
# axs[2,1].set_title("QY")
# axs[3,1].plot(poses[6,:])
# axs[3,1].set_title("QZ")
# plt.show()
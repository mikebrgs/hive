import tf
import sys
import rospy
import rosbag
import numpy as np
import tf.transformations
import matplotlib.pyplot as plt

bag = rosbag.Bag(sys.argv[1])
vive_poses = np.empty((7,0))
optitrack_poses = np.empty((7,0))

aRt = np.matrix([[0.06756, -0.7676, 0.6374],
                 [0.9895, 0.1335, 0.05586],
                 [-0.128, 0.6269, 0.7685]])
vRt = None
oRa = None
for topic, msg, t in bag.read_messages(topics=['/tf', 'tf']):
  if msg.header.frame_id == "vive":
    vRt = tf.transformations.quaternion_matrix([msg.transform.rotation.x,
      msg.transform.rotation.y,
      msg.transform.rotation.z,
      msg.transform.rotation.w])[1:4,1:4]
    print(vRt)
    pose = np.array([[msg.transform.translation.x,
      msg.transform.translation.y,
      msg.transform.translation.z,
      msg.transform.rotation.w,
      msg.transform.rotation.x,
      msg.transform.rotation.y,
      msg.transform.rotation.z]]).transpose()
    vive_poses = np.hstack((vive_poses,pose))
  else:
    oRa = tf.transformations.quaternion_matrix([msg.transform.rotation.x,
      msg.transform.rotation.y,
      msg.transform.rotation.z,
      msg.transform.rotation.w])[1:4,1:4]
    # print(oRa)
    pose = np.array([[msg.transform.translation.x,
      msg.transform.translation.y,
      msg.transform.translation.z,
      msg.transform.rotation.w,
      msg.transform.rotation.x,
      msg.transform.rotation.y,
      msg.transform.rotation.z]]).transpose()
    optitrack_poses = np.hstack((optitrack_poses,pose))

oRv = oRa * aRt * vRt.transpose();
print(oRv)

fig2, axs = plt.subplots(4, 2, sharex = True, sharey= False)

axs[0,0].plot(poses[0,:])
axs[0,0].set_title("PX")
axs[1,0].plot(poses[1,:])
axs[1,0].set_title("PY")
axs[2,0].plot(poses[2,:])
axs[2,0].set_title("PZ")
axs[0,1].plot(poses[3,:])
axs[0,1].set_title("QW")
axs[1,1].plot(poses[4,:])
axs[1,1].set_title("QX")
axs[2,1].plot(poses[5,:])
axs[2,1].set_title("QY")
axs[3,1].plot(poses[6,:])
axs[3,1].set_title("QZ")
plt.show()
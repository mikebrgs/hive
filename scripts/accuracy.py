import transform as tf
import scipy.linalg
import numpy as np
import rosbag
import rospy
import math
import sys

'''Returns dictionary with lists of poses'''
def readbag(filename, mode = 0):
  bag = rosbag.Bag(filename)
  poses = dict()
  for topic, msg, t in bag.read_messages(topics=['/tf', 'tf']):
    if not poses.has_key(msg.transforms[0].child_frame_id):
      poses[msg.transforms[0].child_frame_id] = list()
    pose = tf.Transform(msg.transforms[0].transform.translation.x,
      msg.transforms[0].transform.translation.y,
      msg.transforms[0].transform.translation.z,
      msg.transforms[0].transform.rotation.w,
      msg.transforms[0].transform.rotation.x,
      msg.transforms[0].transform.rotation.y,
      msg.transforms[0].transform.rotation.z,
      float(msg.transforms[0].header.stamp.secs) + float(msg.transforms[0].header.stamp.nsecs) * 10e-9)
    # print(msg.transforms[0].header.stamp.secs)
    # print(msg.transforms[0].header.stamp.nsecs)
    # sys.exit(0)
    poses[msg.transforms[0].child_frame_id].append(pose)
  return poses

def getplane(poses):
  aux_matrix = np.empty([3,0])
  for i in range(0, poses.__len__()):
    aux_matrix = np.hstack((aux_matrix,
      np.matrix([poses[i].x,
        poses[i].y,
        poses[i].z]).transpose()))
  A = aux_matrix[0:2,:].transpose()
  A = np.hstack((A, np.ones((A.shape[0], 1))))
  b = aux_matrix[2,:].transpose()
  new_A = np.c_[aux_matrix[0,:].transpose(), aux_matrix[1,:].transpose(), np.ones(aux_matrix.shape[1])]
  aux_matrix_mean = aux_matrix.sum(axis=1) / aux_matrix.shape[1]
  u,s,vh = np.linalg.linalg.svd((aux_matrix-aux_matrix_mean).transpose())
  v = vh.conj().transpose()
  normal = v[:,-1]
  offset = -v[:,-1].transpose()*aux_matrix_mean
  return normal, offset


def getplanes(poses):
  trackers = poses.items()
  normals = dict()
  offsets = dict()
  for tracker, _ in trackers:
    aux_matrix = np.empty([3,0])
    print(tracker + ": " + str(poses[tracker].__len__()))
    normals[tracker], offsets[tracker] = getplane(poses[tracker])
  return normals, offsets

def getpositionerror(pose, normal, offset):
  error = float(normal.transpose() * np.matrix([pose.x,
    pose.y,
    pose.z]).transpose() + offset)
  return error

def getpositionerrors(poses, normal, offset):
  errors = list()
  # print(poses.__len__())
  for i in range(0, poses.__len__()):
    errors.append(getpositionerror(poses[i], normal, offset))
  return errors

def getpositionerrorsplanes(poses, normals, offsets):
  trackers = poses.items()
  errors = dict()
  for tracker, _ in trackers:
    # print(tracker)
    errors[tracker] = getpositionerrors(poses[tracker], normals[tracker], offsets[tracker])
  return errors

def liststatistics(thelist):
  theabsolutelist = list()
  for i in range(0, thelist.__len__()):
    theabsolutelist.append(abs(thelist[i]))
  mean_error = np.mean(theabsolutelist)
  std_error = np.std(thelist)
  max_error = max(theabsolutelist)
  return max_error, mean_error, std_error

def dictstatistics(thedict):
  theabsolutedict = dict()
  trackers = thedict.items()
  for tracker, _ in trackers:
    max_error, mean_error, std_error = liststatistics(thedict[tracker])
    print("***Statistics " + tracker + "***")
    print("MaxError: " + str(max_error))
    print("MeanError: " + str(mean_error))
    print("STDError: " + str(std_error))
  return

def getorientationerror(pose, rotated_normal, normal):
  rotated_rotated_normal = pose.inverse().rotatevector(rotated_normal)
  dot_product = (float(normal.transpose() * rotated_rotated_normal) /
    (np.linalg.norm(normal) * np.linalg.norm(rotated_rotated_normal)))
  if dot_product > 1:
    angle = 0
  elif dot_product < -1:
    angle = math.pi
  else:
    angle = math.acos(dot_product)
  if angle > math.pi:
    angle = angle - 2*math.pi
  elif angle < -math.pi:
    angle = angle + 2*math.pi
  return angle * 180 / math.pi, rotated_rotated_normal

def displaynormals(normals):
  plt.figure(1)
  plt.subplot(311)
  plt.plot(normals[0])
  plt.title('x')
  plt.subplot(312)
  plt.plot(normals[1])
  plt.title('y')
  plt.subplot(313)
  plt.plot(normals[2])
  plt.title('z')
  plt.show()
  return

def getorientationerrors(poses, normal):
  errors = list()
  normals = [list(), list(), list()]
  rotated_normal = poses[0].rotatevector(normal)
  for i in range(0, poses.__len__()):
    error, normal = getorientationerror(poses[i], rotated_normal, normal)
    normals[0].append(float(normal[0]))
    normals[1].append(float(normal[1]))
    normals[2].append(float(normal[2]))
    errors.append(error)
  return errors

def getorientationerrosplanes(poses, normals):
  trackers = poses.items()
  errors = dict()
  for tracker, _ in trackers:
    errors[tracker] = getorientationerrors(poses[tracker], normals[tracker])
  return errors

import matplotlib.pyplot as plt
def displayorientations(poses):
  qw = list()
  qx = list()
  qy = list()
  qz = list()
  for i in range(0, poses.__len__()):
    qw.append(poses[i].qw)
    qx.append(poses[i].qx)
    qy.append(poses[i].qy)
    qz.append(poses[i].qz)
  plt.figure(1)
  plt.subplot(411)
  plt.plot(qw)
  plt.title('qw')
  plt.subplot(412)
  plt.plot(qx)
  plt.title('qx')
  plt.subplot(413)
  plt.plot(qy)
  plt.title('qy')
  plt.subplot(414)
  plt.plot(qz)
  plt.title('qz')
  plt.show()
  return

def getvelocities(poses):
  velocities = dict()
  for tracker in poses:
    if poses[tracker].__len__() <= 1:
      continue
    velocities[tracker] = list()
    for i in range(0, poses[tracker].__len__()-1):
      current_pose = poses[tracker][i]
      # print(current_pose.stamp)
      # sys.exit(0)
      next_pose = poses[tracker][i+1]
      v = math.sqrt((next_pose.x-current_pose.x)**2 +
        (next_pose.y - current_pose.y)**2 +
        (next_pose.z - current_pose.z)**2) / (next_pose.stamp - current_pose.stamp)
      velocities[tracker].append(v)
  return velocities


def displayorientationsplanes(poses):
  trackers = poses.items()
  for tracker, _ in trackers:
    displayorientations(poses[tracker])
  return

def main(argv):
  poses = readbag(sys.argv[1])
  normals, offsets = getplanes(poses)
  # Position
  print("POSITION")
  errors = getpositionerrorsplanes(poses, normals, offsets)
  dictstatistics(errors)
  # Orientation
  print("ORIENTATION")
  errors = getorientationerrosplanes(poses, normals)
  dictstatistics(errors)

  print("VELOCITY")
  velocities = getvelocities(poses)
  dictstatistics(velocities)
  return

if __name__ == "__main__":
  main(sys.argv)

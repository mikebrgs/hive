import cv2
import math
import numpy as np

side = 0.00214663
gamma = 0

position = np.array([0.0, 0.0, 0.0])
normal = np.array([0.0, 0.0, 1.0])
reference = np.array([0.0, 0.0, 1.0])

def randompose():
  Z = np.random.rand() * 5
  XY_max = Z * np.tan(math.pi/3)
  X = (np.random.rand() - 0.5) * 2 * XY_max
  Y = (np.random.rand() - 0.5) * 2 * XY_max
  Ax = np.random.rand() * math.pi
  Ay = np.random.rand() * math.pi
  Az = np.random.rand() * math.pi
  pose = np.array([X,Y,Z,Ax,Ay,Az])
  return pose

def verticalbetafrompose(pose):
  lRt = np.asmatrix(cv2.Rodrigues(pose[3:6])[0])
  lPt = np.asmatrix(pose[0:3]).transpose()
  tN = np.asmatrix(normal).transpose()
  tP = np.asmatrix(position).transpose()
  tR = np.asmatrix(reference).transpose()
  td1 = tR
  td2 = np.asmatrix(np.cross(np.asarray(td1).reshape(3),
    np.asarray(tN).reshape(3)).reshape((3,1)))

  tC1 = (side / 2) * (td1 + td2) + tP
  wC1 = np.asarray(lRt * tC1 + lPt).reshape(3)

  tC2 = (side / 2) * (td1 - td2) + tP
  wC2 = np.asarray(lRt * tC2 + lPt).reshape(3)

  tC3 = (side / 2) * ( - td1 + td2) + tP
  wC3 = np.asarray(lRt * tC3 + lPt).reshape(3)

  tC4 = (side / 2) * ( - td1 - td2) + tP
  wC4 = np.asarray(lRt * tC4 + lPt).reshape(3)

  beta1 = np.arctan2(wC1[1], wC1[2])
  beta2 = np.arctan2(wC2[1], wC2[2])
  beta3 = np.arctan2(wC3[1], wC3[2])
  beta4 = np.arctan2(wC4[1], wC4[2])
  beta_list = [beta1, beta2, beta3, beta4]
  beta = max(beta_list) - min(beta_list)

  return beta * 10e3

def horizontalbetafrompose(pose):
  lRt = np.asmatrix(cv2.Rodrigues(pose[3:6])[0])
  lPt = np.asmatrix(pose[0:3]).transpose()
  tN = np.asmatrix(normal).transpose()
  tP = np.asmatrix(position).transpose()
  tR = np.asmatrix(reference).transpose()
  td1 = tR
  td2 = np.asmatrix(np.cross(np.asarray(td1).reshape(3),
    np.asarray(tN).reshape(3)).reshape((3,1)))

  tC1 = (side / 2) * (td1 + td2) + tP
  wC1 = np.asarray(lRt * tC1 + lPt).reshape(3)

  tC2 = (side / 2) * (td1 - td2) + tP
  wC2 = np.asarray(lRt * tC2 + lPt).reshape(3)

  tC3 = (side / 2) * ( - td1 + td2) + tP
  wC3 = np.asarray(lRt * tC3 + lPt).reshape(3)

  tC4 = (side / 2) * ( - td1 - td2) + tP
  wC4 = np.asarray(lRt * tC4 + lPt).reshape(3)

  beta1 = np.arctan2(wC1[0], wC1[2])
  beta2 = np.arctan2(wC2[0], wC2[2])
  beta3 = np.arctan2(wC3[0], wC3[2])
  beta4 = np.arctan2(wC4[0], wC4[2])
  beta_list = [beta1, beta2, beta3, beta4]
  beta = max(beta_list) - min(beta_list)

  return beta * 10e3

def reduceverticalpose(pose):
  distance = np.sqrt(pose[1]**2 + pose[2]**2)
  alpha_v = np.arctan2(pose[0],pose[2])
  height = pose[0]
  Rx = np.matrix([[1.0,0.0,0.0],
    [0.0,np.cos(-alpha_v),-np.sin(-alpha_v)],
    [0.0,np.sin(-alpha_v),np.cos(-alpha_v)]])
  R = cv2.Rodrigues(pose[3:6])[0]
  new_R = Rx * R
  trAl = cv2.Rodrigues(new_R)[0]
  orientation_x = trAl[0]
  orientation_y = trAl[1]
  orientation_z = trAl[2]
  features = np.array([distance, orientation_x, orientation_y, orientation_z])
  return features

def reducehorizontalpose(pose):
  distance = np.sqrt(pose[0]**2 + pose[2]**2)
  alpha_v = np.arctan2(pose[1], pose[2])
  height = pose[1]
  Ry = np.matrix([[np.cos(alpha_h),0.0,np.sin(alpha_h)],
    [0.0, 1.0, 0.0],
    [-np.sin(alpha_h),0.0,np.cos(alpha_h)]])
  R = cv2.Rodrigues(pose[3:6])[0]
  new_R = Ry * R
  trAl = cv2.Rodrigues(new_R)[0]
  orientation_x = trAl[0]
  orientation_y = trAl[1]
  orientation_z = trAl[2]
  features = np.array([distance, orientation_x, orientation_y, orientation_z])
  return features
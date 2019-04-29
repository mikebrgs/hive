import numpy as np
import math

# class Translation

"""This class handles 3D rigid body transforms
It saves the translation and rotation and simplifies the notation when writing
the script with it"""
class Transform:

# Initializes the transform to default or non default values
  def __init__(self,x=0.0,y=0.0,z=0.0,qw=1.0,qx=0.0,qy=0.0,qz=0.0,stamp=None,child=None,parent=None):
    self.x = x
    self.y = y
    self.z = z
    self.qx = qx
    self.qy = qy
    self.qz = qz
    self.qw = qw
    self.child = child
    self.parent = parent
    self.stamp = stamp

  # Initializes the translation to the values we want
  def SetTranslation(self,x,y,z):
    self.translation[0,0] = x
    self.translation[1,0] = y
    self.translation[2,0] = z

  #Initializes the orientation with a quaternion to the values we want
  def SetRotation(self,w,x,y,z):
    self.qx = x
    self.qy = y
    self.qz = z
    self.qw = w

  # Sets the parent of the frame
  def SetParentFrame(self, frame):
    self.parent = frame

  # Sets the child of the frame
  def SetChildFrame(self, frame):
    self.child = frame

  def SetStamp(self, stamp):
    self.stamp = stamp

  def getx(self):
    return float(self.x)

  def gety(self):
    return float(self.y)

  def getz(self):
    return float(self.z)

  def getqx(self):
    return qx

  def getqy(self):
    return qy

  def getqz(self):
    return qz

  def getqw(self):
    return qw

  def axisangle(self):
    aux = Transform(self.x, self.y, self.z, self.qw, self.qx, self.qy, self.qz)
    aux.qw = aux.qw / math.sqrt(aux.qx**2 + aux.qy**2 + aux.qz**2 + aux.qw**2)
    aux.qx = aux.qx / math.sqrt(aux.qx**2 + aux.qy**2 + aux.qz**2 + aux.qw**2)
    aux.qy = aux.qy / math.sqrt(aux.qx**2 + aux.qy**2 + aux.qz**2 + aux.qw**2)
    aux.qz = aux.qz / math.sqrt(aux.qx**2 + aux.qy**2 + aux.qz**2 + aux.qw**2)
    # print(aux)
    if self.qw < 0:
      aux.qw = -aux.qw
      aux.qx = -aux.qx
      aux.qy = -aux.qy
      aux.qz = -aux.qz
    # if (aux.qw == 1 or aux.qw == -1):
    #   return np.matrix([[0],[0],[0]])
    angle = 2*math.acos(aux.qw)
    if angle > math.pi:
      angle -= 2*math.pi
    elif angle < -math.pi:
      angle += 2*math.pi
    # aa_x = aux.qx / math.sqrt(1-aux.qw**2) * angle
    # aa_y = aux.qy / math.sqrt(1-aux.qw**2) * angle
    # aa_z = aux.qz / math.sqrt(1-aux.qw**2) * angle
    # aa = np.matrix([[aa_x],[aa_y],[aa_z]])
    rotm = np.matrix([
      [1 - 2*aux.qy**2 - 2*aux.qz**2,
        2*aux.qx*aux.qy - 2*aux.qz*aux.qw,
        2*aux.qx*aux.qz + 2*aux.qy*aux.qw],
      [2*aux.qx*aux.qy + 2*aux.qz*aux.qw,
        1 - 2*aux.qx**2 - 2*aux.qz**2,
        2*aux.qy*aux.qz - 2*aux.qx*aux.qw],
      [2*aux.qx*aux.qz - 2*aux.qy*aux.qw,
        2*aux.qx*aux.qz - 2*aux.qy*aux.qw,
        1 - 2*aux.qx**2 - 2*aux.qy**2]
    ])
    w, v = np.linalg.eig(rotm)
    # print("w: " + str(w))
    # print("v: " + str(v))
    axis = 0
    if abs(np.real(w[0]) - 1)<1e-2 and abs(np.real(w[1]) - 1)<1e-2 and abs(np.real(w[2]) - 1)<1e-2:
      return np.matrix([[0],[0],[0]]), 0
    if abs(np.real(w[1]) - 1)<1e-2 and abs(np.real(w[2]) - 1)<1e-2:
      return np.matrix([[0],[0],[0]]), 0
    if abs(np.real(w[0]) - 1)<1e-2 and abs(np.real(w[2]) - 1)<1e-2:
      return np.matrix([[0],[0],[0]]), 0
    if abs(np.real(w[0]) - 1)<1e-2 and abs(np.real(w[1]) - 1)<1e-2:
      return np.matrix([[0],[0],[0]]), 0
    if angle < 0.05:
      return np.matrix([[0],[0],[0]]), 0
    for i in range(0, 3):
      if abs(np.real(w[i]) - 1) <= abs(np.real(w[0]) - 1):
        axis = i
    if axis == 0:
      other_axis = 1
    else:
      other_axis = 0
    axis = np.matrix([[np.real(v[0,i])],[np.real(v[1,i])],[np.real(v[2,i])]])
    return axis, angle

  def inverse(self):
    inv = Transform()
    inv.qw = self.qw
    inv.qx = - self.qx
    inv.qy = - self.qy
    inv.qz = - self.qz
    return inv

  def rotatevector(self, vector):
    rotated_vector = np.matrix(np.zeros((3,1)))
    rotm = np.matrix([
      [1 - 2*self.qy**2 - 2*self.qz**2,
        2*self.qx*self.qy - 2*self.qz*self.qw,
        2*self.qx*self.qz + 2*self.qy*self.qw],
      [2*self.qx*self.qy + 2*self.qz*self.qw,
        1 - 2*self.qx**2 - 2*self.qz**2,
        2*self.qy*self.qz - 2*self.qx*self.qw],
      [2*self.qx*self.qz - 2*self.qy*self.qw,
        2*self.qy*self.qz + 2*self.qx*self.qw,
        1 - 2*self.qx**2 - 2*self.qy**2]
    ])
    # rotated_vector[0] = ((self.qx**2 - self.qy**2 - self.qz**2 + self.qw**2) * vector[0,0] +
    #     2*(self.qx*self.qy + self.qz*self.qw) * vector[1,0] +
    #     2*(self.qx*self.qz - self.qy*self.qw) * vector[2,0])
    # rotated_vector[1] = (2*(self.qx*self.qy - self.qz*self.qw) * vector[0,0] +
    #     (-self.qx**2 + self.qy**2 - self.qz**2 + self.qw**2) * vector[1,0] +
    #     2*(self.qy*self.qz + self.qx*self.qw) * vector[2,0])
    # rotated_vector[2] = (2*(self.qx*self.qz + self.qy*self.qw) * vector[0,0] +
    #     2*(self.qy*self.qz + self.qx*self.qw) * vector[1,0] +
    #     (-self.qx**2 - self.qy**2 + self.qz**2 + self.qw**2) * vector[2,0])
    return rotm.transpose()*vector

  # Multiplication of transforms
  def __mul__(self, other):
    product = Transform()
    product.x = ((self.qx**2 - self.qy**2 - self.qz**2 + self.qw**2) * other.x +
        2*(self.qx*self.qy + self.qz*self.qw) * other.y +
        2*(self.qx*self.qz - self.qy*self.qw) * other.z +
        self.x)
    product.y = (2*(self.qx*self.qy - self.qz*self.qw) * other.x +
        (-self.qx**2 + self.qy**2 - self.qz**2 + self.qw**2) * other.y +
        2*(self.qy*self.qz + self.qx*self.qw) * other.z +
        self.y)
    product.z = (2*(self.qx*self.qz + self.qy*self.qw) * other.x +
        2*(self.qy*self.qz + self.qx*self.qw) * other.y +
        (-self.qx**2 - self.qy**2 + self.qz**2 + self.qw**2) * other.z +
        self.z)
    product.qx =  (self.qx * other.qw +
        self.qy * other.qz -
        self.qz * other.qy +
        self.qw * other.qx)
    product.qy = (-self.qx * other.qz +
        self.qy * other.qw +
        self.qz * other.qx +
        self.qw * other.qy)
    product.qz =  (self.qx * other.qy -
        self.qy * other.qx +
        self.qz * other.qw +
        self.qw * other.qz)
    product.qw = (-self.qx * other.qx -
        self.qy * other.qy -
        self.qz * other.qz +
        self.qw * other.qw)
    return product

  # Multiplication of transforms
  def __rmul__(self, other):
    product = Transform()
    product.x = ((other.qx**2 - other.qy**2 - other.qz**2 + other.qw**2) * self.x +
        2*(other.qx*other.qy + other.qz*other.qw) * self.y +
        2*(other.qx*other.qz - other.qy*other.qw) * self.z +
        other.x)
    product.y = (2*(other.qx*other.qy - other.qz*other.qw) * self.x +
        (-other.qx**2 + other.qy**2 - other.qz**2 + other.qw**2) * self.y +
        2*(other.qy*other.qz + other.qx*other.qw) * self.z +
        other.y)
    product.z = (2*(other.qx*other.qz + other.qy*other.qw) * self.x +
        2*(other.qy*other.qz + other.qx*other.qw) * self.y +
        (-other.qx**2 - other.qy**2 + other.qz**2 + other.qw**2) * self.z +
        other.z)
    product.qx =  (other.qx * self.qw +
        other.qy * self.qz -
        other.qz * self.qy +
        other.qw * self.qx)
    product.qy = (-other.qx * self.qz +
        other.qy * self.qw +
        other.qz * self.qx +
        other.qw * self.qy)
    product.qz =  (other.qx * self.qy -
        other.qy * self.qx +
        other.qz * self.qw +
        other.qw * self.qz)
    product.qw = (-other.qx * self.qx -
        other.qy * self.qy -
        other.qz * self.qz +
        other.qw * self.qw)
    return product

  # Convertion to string of the translation only
  def __str__(self):
    aux_str = ("(p = [" + str(self.x) + ", " +
    str(self.y) + ", " +
    str(self.z) + "], q=[" +
    str(self.qw) + ", " +
    str(self.qx) + ", " +
    str(self.qy) + ", " +
    str(self.qz) + "])")
    return aux_str

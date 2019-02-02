import numpy as np
from scipy.optimize import minimize, rosen, rosen_der

def hModel(params, lPt, lRt, tPs, tNs, tO, obs):
  # params[0] - side size
  # params[1] - orientation of reference
  # lPt - position of the photodiode in the lighthouse frame
  # lNs - normal of the photodiode in the lighthouse frame
  # lOs - reference of the photodiode in the lighthouse frame
  # Rotating the reference vector to desired orientation
  tRs = np.asmatrix(np.eye(3))
  cv2.Rodrigues(lNs * params[1], tRs)

  td1 = tRs * tO
  td2 = np.asmatrix(np.cross(np.asarray(td1).reshape(3),  np.asarray(tNs).reshape(3)).reshape((3,1)))

  tC1 = (params[0] / 2) * (td1 + td2) + tPs
  tC2 = (params[0] / 2) * (- td1 - td2) + tPs
  tC3 = (params[0] / 2) * (td1 - td2) + tPs
  tC4 = (params[0] / 2) * (- td1 + td2) + tPs

  lC1 = np.asarray(lRt * tC1 + lPt).reshape(3)
  lC2 = np.asarray(lRt * tC2 + lPt).reshape(3)
  lC3 = np.asarray(lRt * tC3 + lPt).reshape(3)
  lC4 = np.asarray(lRt * tC4 + lPt).reshape(3)

  alpha1 = np.arctan2(lC1[0], lC1[2])
  alpha2 = np.arctan2(lC2[0], lC2[2])
  alpha3 = np.arctan2(lC3[0], lC3[2])
  alpha4 = np.arctan2(lC4[0], lC4[2])
  alpha_list = [alpha1, alpha2, alpha3, alpha4]

  ang = max(alpha_list) - min(alpha_list)

  return (obs - ang)**2

def vModel(params, lPs, lRt, tPs, tNs, tO, obs):
  # params[0] - side size
  # params[1] - orientation of reference
  # lPs - position of the photodiode in the lighthouse frame
  # lNs - normal of the photodiode in the lighthouse frame
  # lOs - reference of the photodiode in the lighthouse frame
  # Rotating the reference vector to desired orientation
  tRs = np.asmatrix(np.eye(3))
  cv2.Rodrigues(lNs * params[1], tRs)

  td1 = tRs * tO
  td2 = np.asmatrix(np.cross(np.asarray(td1).reshape(3),  np.asarray(tNs).reshape(3)).reshape((3,1)))

  tC1 = (params[0] / 2) * (td1 + td2) + tPs
  tC2 = (params[0] / 2) * (- td1 - td2) + tPs
  tC3 = (params[0] / 2) * (td1 - td2) + tPs
  tC4 = (params[0] / 2) * (- td1 + td2) + tPs

  lC1 = np.asarray(lRt * tC1 + lPt).reshape(3)
  lC2 = np.asarray(lRt * tC2 + lPt).reshape(3)
  lC3 = np.asarray(lRt * tC3 + lPt).reshape(3)
  lC4 = np.asarray(lRt * tC4 + lPt).reshape(3)

  alpha1 = np.arctan2(lC1[2], lC1[2])
  alpha2 = np.arctan2(lC2[2], lC2[2])
  alpha3 = np.arctan2(lC3[2], lC3[2])
  alpha4 = np.arctan2(lC4[2], lC4[2])
  alpha_list = [alpha1, alpha2, alpha3, alpha4]

  ang = max(alpha_list) - min(alpha_list)

  return (obs - ang)**2


x0 = [2 * 0.0016, 0]
res = minimize(hModel, x0, args = (2, ), method='Nelder-Mead', tol=1e-6)
print(res.x)
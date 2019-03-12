import sys
import cv2
import pickle
import libsolve
import libnrvsolve
import numpy as np
from sklearn.svm import SVR
import matplotlib.pyplot as plt

def main(args):
  trainfile = ""
  testfile = ""
  load = False
  save = False

  # Reading arguments
  if len(args) < 3:
    print("Usage:")
    print("python solvesvr.py <trainfile.bag> <testfile.bag>")
    print("python solvesvr.py -s <trainfile.bag> <testfile.bag>")
    print("python solvesvr.py -l <testfile.bag>")
    sys.exit(0)
  if len(args) >= 3 and args[1] == "-l":
    load = True
    testfile = args[2]
  elif len(args) > 3 and args[1] == "-s":
    save = True
    trainfile = args[2]
    testfile = args[3]
  else:
    load = False
    trainfile = args[1]
    testfile = args[2]

  # Reading tracker data
  hmodel = None
  vmodel = None
  if not load:
    trackers = dict()
    br = libsolve.BagReader(trainfile)
    br.read("/loc/vive/trackers", libnrvsolve.trackerhandler, trackers)
    # Reading light data
    data = [np.empty([0,4]),
      np.empty([0,1]),
      np.empty([0,4]),
      np.empty([0,1])]
    lr = libnrvsolve.LightReader(trainfile, trackers)
    lr.read("/loc/vive/light", data)

    # # Coordinate conversion
    # reduced_h = np.empty([0,4])
    # reduced_v = np.empty([0,4])
    # for i in range(0,data[0].shape[0]):
    #   distance = np.sqrt(data[0][i,0]**2 + data[0][i,1]**2)
    #   alpha_h = np.arctan2(data[0][i,0] , data[0][i,2])
    #   height = data[0][i,1]
    #   Ry = np.matrix([[np.cos(alpha_h),0.0,np.sin(alpha_h)],
    #     [0.0, 1.0, 0.0],
    #     [-np.sin(alpha_h),0.0,np.cos(alpha_h)]])
    #   R = cv2.Rodrigues(data[0][i,3:6])[0]
    #   new_R = Ry * R
    #   trAl = cv2.Rodrigues(new_R)[0]
    #   orientation_x = trAl[0]
    #   orientation_y = trAl[1]
    #   orientation_z = trAl[2]
    #   features = np.array([distance, orientation_x, orientation_y, orientation_z])
    #   reduced_h = np.vstack((reduced_h, features))

    # for i in range(0,data[2].shape[0]):
    #   distance = np.sqrt(data[2][i,0]**2 + data[2][i,1]**2)
    #   alpha_v = np.arctan2(data[2][i,1] , data[2][i,2])
    #   height = data[2][i,0]
    #   Ry = np.matrix([[np.cos(alpha_v),0.0,np.sin(alpha_v)],
    #     [0.0, 1.0, 0.0],
    #     [-np.sin(alpha_v),0.0,np.cos(alpha_v)]])
    #   R = cv2.Rodrigues(data[2][i,3:6])[0]
    #   new_R = Ry * R
    #   trAl = cv2.Rodrigues(new_R)[0]
    #   orientation_x = trAl[0]
    #   orientation_y = trAl[1]
    #   orientation_z = trAl[2]
    #   features = np.array([distance, orientation_x, orientation_y, orientation_z])
    #   reduced_v = np.vstack((reduced_v, features))

    # # Plot
    # fig, axs = plt.subplots(6, 3, sharex = False, sharey= False)
    # axs[0,0].plot(data[0][:,0])
    # axs[1,0].plot(data[0][:,1])
    # axs[2,0].plot(data[0][:,2])
    # axs[3,0].plot(data[0][:,3])
    # axs[4,0].plot(data[0][:,4])
    # axs[5,0].plot(data[0][:,5])

    # axs[0,1].plot(reduced_h[:,0])
    # axs[3,1].plot(reduced_h[:,1])
    # axs[4,1].plot(reduced_h[:,2])
    # axs[5,1].plot(reduced_h[:,3])

    # axs[0,2].plot(reduced_v[:,0])
    # axs[3,2].plot(reduced_v[:,1])
    # axs[4,2].plot(reduced_v[:,2])
    # axs[5,2].plot(reduced_v[:,3])
    # plt.show()


    # Undersampling
    SPLIT = 0.5
    h_idx = np.random.choice(range(data[0].shape[0]),
      int(data[0].shape[0] * SPLIT),
      replace=False)
    v_idx = np.random.choice(range(data[2].shape[0]),
      int(data[2].shape[0] * SPLIT),
      replace=False)

    # Training SVR
    hmodel = SVR(kernel = "rbf",
      gamma=1e-1, # Controls the degree of the RBF - change this one for more detail
      C=1e5,
      epsilon=0.005,
      verbose = True)
    hmodel.fit(data[0][h_idx], data[1][h_idx])
    vmodel = SVR(kernel = "rbf",
      gamma=1e-1, # Controls the degree of the RBF - change this one for more detail
      C=1e5,
      epsilon=0.005,
      verbose = True)
    vmodel.fit(data[2][v_idx], data[3][v_idx]) 
    models = (hmodel,vmodel)

    # Saving the data
    if save:
      dumpfile = open("tmp/svr_models4.bin", "wb")
      pickle.dump(models, dumpfile)
      dumpfile.close()
  else:
    # Saving the data
    loadfile = open("tmp/svr_models4.bin", "rb")
    models = pickle.load(loadfile)
    loadfile.close()

    hmodel = models[0]
    vmodel = models[1]

  # Reading Testing data
  trackers = dict()
  br = libsolve.BagReader(testfile)
  br.read("/loc/vive/trackers", libnrvsolve.trackerhandler, trackers)
  # Reading light data
  data = [np.empty([0,4]),
    np.empty([0,1]),
    np.empty([0,4]),
    np.empty([0,1])]
  lr = libnrvsolve.LightReader(testfile, trackers)
  lr.read("/loc/vive/light", data)

  # # Coordinate conversion
  # reduced_h = np.empty([0,4])
  # reduced_v = np.empty([0,4])
  # for i in range(0,data[0].shape[0]):
  #   distance = np.sqrt(data[0][i,0]**2 + data[0][i,1]**2)
  #   alpha_h = np.arctan2(data[0][i,0] , data[0][i,2])
  #   height = data[0][i,1]
  #   Ry = np.matrix([[np.cos(alpha_h),0.0,np.sin(alpha_h)],
  #     [0.0, 1.0, 0.0],
  #     [-np.sin(alpha_h),0.0,np.cos(alpha_h)]])
  #   R = cv2.Rodrigues(data[0][i,3:6])[0]
  #   new_R = Ry * R
  #   trAl = cv2.Rodrigues(new_R)[0]
  #   orientation_x = trAl[0]
  #   orientation_y = trAl[1]
  #   orientation_z = trAl[2]
  #   features = np.array([distance, orientation_x, orientation_y, orientation_z])
  #   reduced_h = np.vstack((reduced_h, features))

  # for i in range(0,data[2].shape[0]):
  #   distance = np.sqrt(data[2][i,0]**2 + data[2][i,1]**2)
  #   alpha_h = np.arctan2(data[2][i,1] , data[2][i,2])
  #   height = data[2][i,0]
  #   Ry = np.matrix([[np.cos(alpha_h),0.0,np.sin(alpha_h)],
  #     [0.0, 1.0, 0.0],
  #     [-np.sin(alpha_h),0.0,np.cos(alpha_h)]])
  #   R = cv2.Rodrigues(data[2][i,3:6])[0]
  #   new_R = Ry * R
  #   trAl = cv2.Rodrigues(new_R)[0]
  #   orientation_x = trAl[0]
  #   orientation_y = trAl[1]
  #   orientation_z = trAl[2]
  #   features = np.array([distance, orientation_x, orientation_y, orientation_z])
  #   reduced_v = np.vstack((reduced_v, features))

  # Predict with test data
  predicted_hdata = hmodel.predict(data[0])
  predicted_vdata = vmodel.predict(data[2])

  # Plot
  fig, axs = plt.subplots(2, 2, sharex = False, sharey= False)
  axs[0,0].plot(predicted_hdata)
  axs[0,0].plot(data[1])
  axs[0,1].plot(predicted_vdata)
  axs[0,1].plot(data[3])
  hist, bin_edges = np.histogram(predicted_hdata - data[1], bins = 20)
  axs[1,0].bar(bin_edges[:-1], hist, width = bin_edges[0] - bin_edges[1])
  hist, bin_edges = np.histogram(predicted_vdata - data[3], bins = 20)
  axs[1,1].bar(bin_edges[:-1], hist, width = bin_edges[0] - bin_edges[1])
  plt.show()

  pass

if __name__ == '__main__':
  main(sys.argv)
import sys
import pickle
import libsolve
import libvsolve
import numpy as np
from sklearn.svm import SVR
import matplotlib.pyplot as plt

def main(args):
  trainfile = ""
  testfile = ""
  load = False
  save = False

  # Experiment
  R = np.matrix(np.eye(6))
  Rz = np.matrix([[0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0]])
  R[0:3,0:3] = Rz
  R[3:6,3:6] = Rz

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
    br.read("/loc/vive/trackers", libvsolve.trackerhandler, trackers)
    # Reading light data
    data = [np.empty([0,6]),
      np.empty([0,1]),
      np.empty([0,6]),
      np.empty([0,1])]
    lr = libvsolve.LightReader(trainfile, trackers)
    lr.read("/loc/vive/light", data)

    # print(data[2])
    # print(R*np.asmatrix(data[2]).transpose())
    # print(np.asarray((R*np.asmatrix(data[2]).transpose()).transpose()))
    stacked_data = np.vstack((data[0], np.asarray((R*np.asmatrix(data[2]).transpose()).transpose())))
    stacked_output = np.vstack((data[1], data[3]))

    # Undersampling
    SPLIT = 0.2
    idx = np.random.choice(range(stacked_data.shape[0]),
      int(stacked_data.shape[0] * SPLIT),
      replace=False)

    # Training SVR
    model = SVR(kernel = "rbf",
      gamma=1e-1, # Controls the degree of the RBF - change this one for more detail
      C=1e5,
      epsilon=0.005,
      verbose = True)
    model.fit(stacked_data[idx], stacked_output[idx])

    # Saving the data
    if save:
      dumpfile = open("tmp/svr_models3.bin", "wb")
      pickle.dump(model, dumpfile)
      dumpfile.close()
  else:
    # Loading the data
    loadfile = open("tmp/svr_models3.bin", "rb")
    model = pickle.load(loadfile)
    loadfile.close()

  #Reading Testing data
  trackers = dict()
  br = libsolve.BagReader(testfile)
  br.read("/loc/vive/trackers", libvsolve.trackerhandler, trackers)
  # Reading light data
  data = [np.empty([0,6]),
    np.empty([0,1]),
    np.empty([0,6]),
    np.empty([0,1])]
  lr = libvsolve.LightReader(testfile, trackers)
  lr.read("/loc/vive/light", data)

  # Predict with test data
  predicted_hdata = model.predict(data[0])
  predicted_vdata = model.predict(np.asarray((R*np.asmatrix(data[2]).transpose()).transpose()))

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
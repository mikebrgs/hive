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


    # Undersampling
    SPLIT = 0.2
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
      dumpfile = open("tmp/svr_models1.bin", "wb")
      pickle.dump(models, dumpfile)
      dumpfile.close()
  else:
    # Saving the data
    loadfile = open("tmp/svr_models1.bin", "rb")
    models = pickle.load(loadfile)
    loadfile.close()

    hmodel = models[0]
    vmodel = models[1]

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
  predicted_hdata = hmodel.predict(data[0])
  predicted_vdata = vmodel.predict(data[2])

  # Plot
  fig, axs = plt.subplots(1, 2, sharex = False, sharey= False)
  hist, bin_edges = np.histogram(predicted_hdata - data[1], bins = 20)
  axs[0].bar(bin_edges[:-1], hist, width = bin_edges[0] - bin_edges[1])
  hist, bin_edges = np.histogram(predicted_vdata - data[3], bins = 20)
  axs[1].bar(bin_edges[:-1], hist, width = bin_edges[0] - bin_edges[1])
  plt.show()

  # fig, axs = plt.subplots(2, 2, sharex = False, sharey= False)
  # axs[0,0].plot(predicted_hdata)
  # axs[0,0].plot(data[1])
  # axs[0,1].plot(predicted_vdata)
  # axs[0,1].plot(data[3])
  # hist, bin_edges = np.histogram(predicted_hdata - data[1], bins = 20)
  # axs[1,0].bar(bin_edges[:-1], hist, width = bin_edges[0] - bin_edges[1])
  # hist, bin_edges = np.histogram(predicted_vdata - data[3], bins = 20)
  # axs[1,1].bar(bin_edges[:-1], hist, width = bin_edges[0] - bin_edges[1])
  # plt.show()

  pass

if __name__ == '__main__':
  main(sys.argv)
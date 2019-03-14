import sys
import cv2
import pickle
import libvive
import libsolve
import libnrvsolve
import numpy as np
from sklearn.svm import SVR
import matplotlib.pyplot as plt

def defaultprint():
  # print("Usage:")
  # print("python solveneural4.py -train [<trainfile.bag> ...]  - test [<testfile.bag> ...]")
  # print("python solveneural4.py -test [<testfile.bag> ...]")
  sys.exit(0)

SAMPLES_TRAIN = 2000
SAMPLES_TEST = 1000

def main(args):
  data = [np.empty([0,4]),
    np.empty([0,1]),
    np.empty([0,4]),
    np.empty([0,1])]
  for _ in range(SAMPLES_TRAIN):
    pose = libvive.randompose()
    features = libvive.reduceverticalpose(pose)
    beta = libvive.verticalbetafrompose(pose)
    data[0] = np.vstack((data[0],features))
    data[1] = np.vstack((data[1],np.array(beta)))

  for _ in range(SAMPLES_TRAIN):
    pose = libvive.randompose()
    features = libvive.reduceverticalpose(pose)
    beta = libvive.verticalbetafrompose(pose)
    data[2] = np.vstack((data[2],features))
    data[3] = np.vstack((data[3],np.array(beta)))


    # Undersampling
    SPLIT = 1.0
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

  data = [np.empty([0,4]),
    np.empty([0,1]),
    np.empty([0,4]),
    np.empty([0,1])]
  for _ in range(SAMPLES_TEST):
    pose = libvive.randompose()
    features = libvive.reduceverticalpose(pose)
    beta = libvive.verticalbetafrompose(pose)
    data[0] = np.vstack((data[0],features))
    data[1] = np.vstack((data[1],np.array(beta)))

  for _ in range(SAMPLES_TEST):
    pose = libvive.randompose()
    features = libvive.reduceverticalpose(pose)
    beta = libvive.verticalbetafrompose(pose)
    data[2] = np.vstack((data[2],features))
    data[3] = np.vstack((data[3],np.array(beta)))

  # Predict with test data
  predicted_hdata = hmodel.predict(data[0])
  predicted_vdata = vmodel.predict(data[2])


  print("Horizontal Error: " + str(np.mean(np.absolute(predicted_hdata - data[1]))))
  print("Vertical Error: " + str(np.mean(np.absolute(predicted_vdata - data[3]))))
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
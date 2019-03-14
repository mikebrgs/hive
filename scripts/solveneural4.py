import sys
import cv2
import pickle
import libsolve
import libnrvsolve
import numpy as np
from sklearn.svm import SVR
import matplotlib.pyplot as plt

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

def build_model():
  model = keras.Sequential([
    layers.Dense(4, activation=tf.nn.sigmoid, input_shape=(4,)),
    layers.Dense(16, activation=tf.nn.sigmoid),
    layers.Dense(16, activation=tf.nn.sigmoid),
    layers.Dense(16, activation=tf.nn.sigmoid),
    layers.Dense(1)
  ])

  optimizer = tf.keras.optimizers.SGD(0.001)

  model.compile(loss='mse',
                optimizer=optimizer,
                metrics=['mae', 'mse'])
  return model

def defaultprint():
  print("Usage:")
  print("python solveneural4.py -train [<trainfile.bag> ...]  - test [<testfile.bag> ...]")
  print("python solveneural4.py -test [<testfile.bag> ...]")
  sys.exit(0)

def main(args):
  trainfiles = list()
  testfiles = list()
  load = False
  save = False

  # Reading arguments
  index = 0
  while index < len(args):
    if args[index] == "-h":
      defaultprint()
    if args[index] == "-s":
      save = True
    if args[index] == "-train":
      index += 1
      while (index < len(args)
        and args[index] != "-test"
        and args[index] != "-train"
        and args[index] != "-test"):
        trainfiles.append(args[index])
        index += 1
      index -= 1
    if args[index] == "-test":
      index += 1
      while (index < len(args)
        and args[index] != "-test"
        and args[index] != "-train"
        and args[index] != "-test"):
        testfiles.append(args[index])
        index += 1
      index -= 1
    index += 1

  if len(testfiles) == 0:
    defaultprint()

  if len(trainfiles) == 0:
    load = True

  # Reading tracker data
  hmodel = None
  vmodel = None
  if not load:
    data = [np.empty([0,4]),
      np.empty([0,1]),
      np.empty([0,4]),
      np.empty([0,1])]
    for trainfile in trainfiles:
      trackers = dict()
      br = libsolve.BagReader(trainfile)
      br.read("/loc/vive/trackers", libnrvsolve.trackerhandler, trackers)
      # Reading light data
      lr = libnrvsolve.LightReader(trainfile, trackers)
      lr.read("/loc/vive/light", data)

    # Undersampling
    SPLIT = 0.1
    h_idx = np.random.choice(range(data[0].shape[0]),
      int(data[0].shape[0] * SPLIT),
      replace=False)
    v_idx = np.random.choice(range(data[2].shape[0]),
      int(data[2].shape[0] * SPLIT),
      replace=False)

    # Model
    hmodel = build_model()
    vmodel = build_model()

    EPOCHS = 2000
    hmodel.fit(data[0][h_idx], data[1][h_idx],
      epochs=EPOCHS, validation_split = 0.0,
      verbose=True, batch_size=50, shuffle = True)
    vmodel.fit(data[2][v_idx], data[3][v_idx],
      epochs=EPOCHS, validation_split = 0.0,
      verbose=True, batch_size=50, shuffle = True)
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
  data = [np.empty([0,4]),
    np.empty([0,1]),
    np.empty([0,4]),
    np.empty([0,1])]
  for testfile in testfiles:
    trackers = dict()
    br = libsolve.BagReader(testfile)
    br.read("/loc/vive/trackers", libnrvsolve.trackerhandler, trackers)
    # Reading light data
    lr = libnrvsolve.LightReader(testfile, trackers)
    lr.read("/loc/vive/light", data)

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
  # print("Horizontal Error: " + str(np.mean(np.absolute(predicted_hdata - data[1]))))
  # print("Vertical Error: " + str(np.mean(np.absolute(predicted_vdata - data[3]))))


if __name__ == '__main__':
  main(sys.argv)
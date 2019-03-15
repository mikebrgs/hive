import sys
import cv2
import pickle
import libvive
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
    layers.Dense(32, activation=tf.nn.sigmoid),
    layers.Dense(32, activation=tf.nn.sigmoid),
    layers.Dense(32, activation=tf.nn.sigmoid),
    layers.Dense(32, activation=tf.nn.sigmoid),
    layers.Dense(32, activation=tf.nn.sigmoid),
    layers.Dense(1)
  ])

  optimizer = tf.keras.optimizers.RMSprop(0.001)

  model.compile(loss='mse',
                optimizer=optimizer,
                metrics=['mae', 'mse'])
  return model

def defaultprint():
  # print("Usage:")
  # print("python solveneural4.py -train [<trainfile.bag> ...]  - test [<testfile.bag> ...]")
  # print("python solveneural4.py -test [<testfile.bag> ...]")
  sys.exit(0)

SAMPLES_TRAIN = 5000
SAMPLES_TEST = 1000

def main(args):
  data = [np.empty([0,4]),
    np.empty([0,1]),
    np.empty([0,4]),
    np.empty([0,1])]
  for _ in range(SAMPLES_TRAIN):
    pose = libvive.randompose()
    features = libvive.reducehorizontalpose(pose)
    beta = libvive.horizontalbetafrompose(pose)
    data[0] = np.vstack((data[0],features))
    data[1] = np.vstack((data[1],np.array(beta)))
    features = libvive.reduceverticalpose(pose)
    beta = libvive.verticalbetafrompose(pose)
    data[2] = np.vstack((data[2],features))
    data[3] = np.vstack((data[3],np.array(beta)))


  # Model
  hmodel = build_model()
  vmodel = build_model()

  EPOCHS = 2000
  hmodel.fit(data[0], data[1],
    epochs=EPOCHS, validation_split = 0.0,
    verbose=True, batch_size= 50, shuffle = True)
  vmodel.fit(data[2], data[3],
    epochs=EPOCHS, validation_split = 0.0,
    verbose=True, batch_size= 50, shuffle = True)
  models = (hmodel,vmodel)

  data = [np.empty([0,4]),
    np.empty([0,1]),
    np.empty([0,4]),
    np.empty([0,1])]
  for index in range(SAMPLES_TEST):
    # pose = libvive.randompose()
    pose = np.array([0.0, 0.0, np.cos(3.14 * 4 * float(index) / float(SAMPLES_TEST)) + 1.5, 0.0, 0.0, 0.0])
    features = libvive.reducehorizontalpose(pose)
    beta = libvive.horizontalbetafrompose(pose)
    data[0] = np.vstack((data[0],features))
    data[1] = np.vstack((data[1],np.array(beta)))
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
  axs[0,0].legend(['Predicted','Real'])
  axs[0,1].plot(predicted_vdata)
  axs[0,1].plot(data[3])
  axs[0,1].legend(['Predicted','Real'])
  hist, bin_edges = np.histogram(predicted_hdata - data[1], bins = 20)
  axs[1,0].bar(bin_edges[:-1], hist, width = bin_edges[0] - bin_edges[1])
  hist, bin_edges = np.histogram(predicted_vdata - data[3], bins = 20)
  axs[1,1].bar(bin_edges[:-1], hist, width = bin_edges[0] - bin_edges[1])
  plt.show()

  pass

if __name__ == '__main__':
  main(sys.argv)
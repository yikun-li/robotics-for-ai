import numpy as np
import cv2
import os
from network import Network


NPY_STORAGE = os.environ['HOME'] + "/numpy/"

trainData = np.load(NPY_STORAGE + "trainData.npy")
trainLabels = np.load(NPY_STORAGE + "trainLabels.npy")

validationData = np.load(NPY_STORAGE + "testData.npy")
validationLabels = np.load(NPY_STORAGE + "testLabels.npy")

N_BATCHES = 20
CHECKPOINT_DIR = "./ckpt/network.ckpt"
validationDataBatches = np.array_split(validationData, N_BATCHES)
validationLabelBatches = np.array_split(validationLabels, N_BATCHES)


nData = len(validationData)
remainder_test = nData % N_BATCHES
batchsize = int(nData / N_BATCHES)
nBatchesValidation = int(nData / batchsize)


network = Network()
network.load_checkpoint(CHECKPOINT_DIR)
def test():

  #nBatches = len(validationDataBatches)
  
  runningAvg = 0.0
  nBatches = len(validationDataBatches)
  ##print "nData = ", nData
  #print "vallabshape ", validationLabelBatches.shape
  counter = 0
  for batchIdx in range(nBatches):
    if batchIdx == nBatches - 1:
        acc = network.test_batch(validationDataBatches[batchIdx][:remainder_test], validationLabelBatches[batchIdx][:remainder_test])
        acc *= remainder_test
        counter += remainder_test
    else:
        acc = network.test_batch(validationDataBatches[batchIdx], validationLabelBatches[batchIdx])
        acc *= batchsize
        counter += batchsize
    runningAvg += acc# * len(validationDataBatches)
  runningAvg /= counter
  return runningAvg

if __name__ == "__main__":
  print "validation score: ", test()

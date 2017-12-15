import numpy as np
import cv2
import os

from network import Network

NPY_STORAGE = os.environ['HOME'] + "/numpy/"

trainData = np.load(NPY_STORAGE + "trainData.npy")
trainLabels = np.load(NPY_STORAGE + "trainLabels.npy")

validationData = np.load(NPY_STORAGE + "testData.npy")
validationLabels = np.load(NPY_STORAGE + "testLabels.npy")


 
N_BATCHES = 150  #N batches
N_ITERATIONS = 2000

MAKE_CHECKPOINT_EACH_N_ITERATIONS = 100
CHECKPOINT_DIR = "./ckpt/network.ckpt"

PRINT_ACC_EVERY_N_EPOCHS = 5
network = Network()
#network.load_checkpoint("./ckpt/network.ckpt")


dataBatches = np.array_split(trainData, N_BATCHES)
labelBatches = np.array_split(trainLabels, N_BATCHES)

nData = len(trainData)
remainder_train = nData % N_BATCHES
batchsize = int(nData / N_BATCHES)
nBatches = int(nData / batchsize)


nData = len(validationData)
remainder_test = nData % N_BATCHES
batchsize = int(nData / N_BATCHES)
nBatchesValidation = int(nData / batchsize)

validationDataBatches = np.array_split(validationData, N_BATCHES)
validationLabelBatches = np.array_split(validationLabels, N_BATCHES)



#validationDataBatches = dataBatches
#validationLabelBatches = labelBatches

def train():
  nBatches = len(dataBatches)
  #print nBatches
  for epoch in range(N_ITERATIONS):
    for batchIdx in range(nBatches):
      if batchIdx == nBatches - 1:
        network.train_batch(dataBatches[batchIdx][:remainder_train], labelBatches[batchIdx][:remainder_train])
      else:
        network.train_batch(dataBatches[batchIdx], labelBatches[batchIdx])
    if epoch % PRINT_ACC_EVERY_N_EPOCHS == 0:
      print "@epoch ", epoch, "/", N_ITERATIONS, " validation accuracy = ", test()
    if epoch % MAKE_CHECKPOINT_EACH_N_ITERATIONS == 0 and epoch != 0:
        network.store_checkpoint(CHECKPOINT_DIR)
  
def test():

  #nBatches = len(validationDataBatches)
  
  runningAvg = 0.0
  nBatches = len(validationDataBatches)
  ##print "nData = ", nData
  #print "vallabshape ", validationLabelBatches.shape
  for batchIdx in range(nBatches):
    if batchIdx == nBatches - 1:
        acc = network.test_batch(validationDataBatches[batchIdx][:remainder_test], validationLabelBatches[batchIdx][:remainder_test])
    else:
        acc = network.test_batch(validationDataBatches[batchIdx], validationLabelBatches[batchIdx])
    runningAvg += acc# * len(validationDataBatches)
  runningAvg /= nBatchesValidation
  return runningAvg
    
if __name__ == "__main__":
  
  train()
  

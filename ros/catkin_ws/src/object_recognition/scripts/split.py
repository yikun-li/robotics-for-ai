import os
import numpy as np
import cv2
from preprocess import preprocess, oneHot

DATADIR = os.environ['HOME'] + "/myBigDataset/"
NPY_STORAGE = os.environ['HOME'] + "/numpy/"
imHeight = 32
imWidht  = 32

nLabels = len(os.listdir(DATADIR))
init = False
trainData = np.asarray([])
trainLabels = np.asarray([]) ### todo, initcheck
testLabels = np.asarray([])
testData = np.asarray([])

labels = os.listdir(DATADIR)
nLabels = len(labels)

oneHotLabels = {}
labelIdx = 0
for label in labels:
  oneHotLabels[label] = oneHot(nLabels, labelIdx)
  labelIdx += 1


for label in labels:
  print "Processing " + label

  currentData = []
  currentLabels = []
  dataInitialized = False
  
  for x in os.walk(DATADIR + label): #1 iter
    nImgs = len(x[2])
    currentData = np.zeros((nImgs, imHeight, imWidht, 3))
    currentLabels = np.zeros((nImgs, len(labels)))
    imIdx = 0
    for im in x[2]:

      print "reading ", (DATADIR + label + '/' + im)
      image = cv2.imread(DATADIR + label + '/' + im)
      currentData[imIdx] = preprocess(image)
      currentLabels[imIdx] = np.asarray(oneHotLabels[label])
      imIdx += 1
      continue
      tgt = preprocess(image)
      #print "tgtdims: ", tgt.shape
      if not dataInitialized:
        #print "initadd"
        currentData = np.asarray(tgt)
        currentLabels = np.asarray(oneHotLabels[label])
        dataInitialized = True
      else:
        #print "addmore"
        currentData = np.concatenate([currentData, tgt])
        #currentData.append([tgt])
        #currentLabels.append()
        #print oneHotLabels[label]
        currentLabels = np.concatenate([currentLabels, oneHotLabels[label]])
    
     # print "curLabelShape", len(currentLabels)
      #print "curdatashape ", len(currentData)
      ##cv2.imshow('test',tgt)
      #cv2.waitKey(0)
  currentData = np.asarray(currentData)
  print "nData in class ", currentData.shape
  print "check1: ", currentData.shape
  splitIdx = int(len(currentData) * 0.8)
  train = np.asarray(currentData[:splitIdx])
  print "appending shape ", train.shape
  trainL = np.asarray(currentLabels[:splitIdx])
  test = np.asarray(currentData[splitIdx:])
  testL = np.asarray(currentLabels[splitIdx:])
  if init == False:
    init = True
    trainData = np.asarray(train)
    trainLabels = np.asarray(trainL)
    testData = np.asarray(test)
    testLabels = np.asarray(testL)
  else:
    trainData = np.concatenate([trainData,train])
    trainLabels = np.concatenate([trainLabels, trainL])
    testLabels = np.concatenate([testLabels, testL])
    testData = np.concatenate([testData, test])
print "traindatashape: ", trainData.shape
np.save(NPY_STORAGE + "trainData", trainData)
np.save(NPY_STOREAGE + "trainLabels", trainLabels)
np.save(NPY_STORAGE + "testData", testData)
np.save(NPY_STORAGE + "testLabels", testLabels)

from numpy import max
from numpy import zeros
from numpy import average
from numpy import dot
from numpy import abs
from numpy import asfarray
from numpy import sort
from numpy import trace
from numpy import argmin
from numpy.linalg import *

from os.path import join, normpath
from os import listdir
import cPickle as pickle
from math import sqrt
from types import InstanceType
import cv
import svmlight
import Image
import multiprocessing
import time

class ImgData:
    def __init__(self, namelist, fspace, avgvals, facemat):
        self.namelist = namelist
        self.eigenfaces = fspace
        self.avgvals = avgvals
        self.facemat = facemat

class Labels:
    
    def __init__(self):
        self.__labellist = list()
    

    def makeAllLabels(self, name_list):
        for name in name_list:
            self.name2label(name)
    

    def label2name(self, label):
        if label < len(self.__labellist):
            return self.__labellist[label]
        return None

    
    def getLabellist(self):
        return self.__labellist
        
    
    def name2label(self, name):
        if name not in self.__labellist:
            self.__labellist.append(name)
        return self.__labellist.index(name)
                    
        
class EigenFaces:

    def __init__(self, numFaces, threshold):
        self.__numFaces = numFaces
        self.__threshold = threshold
        self.__trained = False
        self.__dim = 320  #Images are specified to be 320x320
        self.__saved = False


    def train(self, data):
        if type(data) == str:
            return self.__trainOnDir(data)
        if type(data) == list and not data == [] :
            return self.__trainOnList(data)
        return False


    def save(self, filename):
        data = dict(labels=self.getLabels(), imageData=self.__imgdata, weights=self.__weights)
        self.process = multiprocessing.Process(target=do_save, args=(data, filename))
        self.process.start()


    def load(self, filename):
        with open(filename, 'rb') as opened_file:
            data = pickle.load(opened_file)
        self.labelList = data['labels']
        self.__imgdata = data['imageData']
        self.__weights = data['weights']
        self.__makeModel()
        self.__trained = True
        #print 'Eigenfaces data loaded'
        
        
    #recognizer function. Needs OpenCV IplImage
    def recognize(self, img):
        if not self.__checkRecognizeReady(img):
            return False
        img = self.__convertImage(img)
        return self.__recognizeFromSVM(img)


    #retrieves the scores from all the SVM models. Used for experiments, not for robot.
    def getScores(self, img):
        if not self.__checkRecognizeReady(img):
            return False
        img = self.__convertImage(img)
        return self.__calculateScores(img)
        

    def getLabels(self):
        return self.labelList


    def setNumFaces(self, num):
        """Set number of eigenfaces used. 
        This value may be modified internally if necessary. Calling this
        method requires a subsequent call to train()"""
        self.__numFaces = num
        self.__trained = False
        #print 'Eigenfaces number of eigenfaces:', self.__numFaces


    def getNumFaces(self):
        """Retrieve the currently used amount of eigenfaces"""
        return self.__numFaces
        
        
    def setThreshold(self, num):
        """Supply matching threshold"""
        self.__threshold = num
        #print 'Eigenfaces threshold:', self.__threshold
        
        
    def getImgData(self):
        """Retrieve image data"""
        return self.__imgdata
    

    def getWeights(self):
        """Return weights vector"""
        return self.__weights 
        

    def getThreshold(self):
        """Retrieve current matching threshold"""
        return self.__threshold


    def getTrained(self):
        """Retrieve whether EigenFaces has performed training. If this
        value is False, no recognition can be performed"""
        return self.__trained
    
    def setSaved(self,value):
        """Set to True when all eigenfaces data has been saved,
        and False otherwise."""
        self.__saved = value
    
    def getSaved(self):
        """Return self.__saved, which should be True when all eigenfaces
        data has been saved, and False otherwise."""
        try:
            self.__saved = not self.process.is_alive()
        except:
            pass
        return self.__saved

    def __trainOnList(self, imgarray):
        """Trains eigenfaces on a list of tuples (name, cv.iplimage).
        Train-function automatically detects whether the input is a list or a dir.
        trainondir is used to create a tuple-list first and then train."""
        if imgarray == None:
            self.__trained = False
            return False
        self.__validateNumFaces(imgarray)
        imgarray = self.__convertAllToPIL(imgarray)
        if self.__trained:
            self.__appendImgData(imgarray)
        else:
            self.__genImgData(imgarray)
            
        self.__makeModel()
        self.__trained = True
        return True

    
    def __trainOnDir(self, imgdir):
        """Train EigenFaces on the supplied images. This function returns
        False when training fails due to missing or incorrect paths or files,
        and True otherwise"""
        imgdir = normpath(imgdir)
        imglist = self.__parseFolder(imgdir)
        if imglist == None:
            self.__trained = False
            return False
        return self.__trainOnList(imglist)
        

    def __parseFolder(self, imgdir):
        """Check whether supplied imgdir exists and holds valid images,
        return filenames and image objects of them if so"""
        try:
            filenamelist = listdir(imgdir)        
        except OSError:
            return None
        imglist = []
        try:
            for fname in filenamelist:
                if fname.lower().endswith('.png'):
                    img = cv.LoadImage(join(imgdir, fname), iscolor=cv.CV_LOAD_IMAGE_GRAYSCALE)
                    if (self.__dim, self.__dim) != cv.GetSize(img):
                        return None
                    imglist.append((fname.partition("_")[0], img))
        except IOError:
            return None
        if len(imglist) == 0:
            return None
        return imglist

    def __appendImgData(self, imglist):
        old_imgdata = self.__imgdata
        #clear imagedata to avoid memory problems :)
        self.__imgdata = None 
        namelist = old_imgdata.namelist
        namelist.extend([element[0] for element in imglist])
        numpixels = self.__dim * self.__dim
        numimgs = len(imglist) + len(old_imgdata.facemat[:, 0])
        new_facemat = zeros((numimgs, numpixels))
       
        for idx in range(len(old_imgdata.facemat[:, 0])):
            new_facemat[idx, :] = old_imgdata.facemat[idx, :]
        
        for (name, image) in imglist:
            idx = imglist.index((name, image)) + (len(old_imgdata.facemat[:,0]) - 1)
            new_facemat[idx, :] = self.__makePixArray(image)
        
        self.__processFaceMatrix(namelist, new_facemat, numimgs)

    def __genImgData(self, imglist):     
        numpixels = self.__dim * self.__dim
        numimgs = len(imglist)
        namelist = [element[0] for element in imglist]               
        #trying to create a 2d array ,each row holds pixvalues of a single image
        facemat = zeros((numimgs, numpixels))               
        for (name, image) in imglist:
            idx = imglist.index((name, image))
            facemat[idx, :] = self.__makePixArray(image)
        self.__processFaceMatrix(namelist, facemat, numimgs)
    
    
    def __makePixArray(self, image):
        # creating a single row from one image
        pixarray = asfarray(list(image.getdata()))
        pixarraymax = max(pixarray)
        return pixarray / pixarraymax                        
        

    def __processFaceMatrix(self, namelist, facemat, numimgs):
        #create average values ,one for each column(ie pixel)        
        avgvals = average(facemat, axis = 0)
        #substract avg val from each orig val to get adjusted faces(phi of T&P)     
        adjfaces = facemat - avgvals
        adjfaces_tr = adjfaces.transpose()

        L = dot(adjfaces, adjfaces_tr)

        egvals1, egvects1 = eigh(L)
        #to use svd, comment out the previous line and uncomment the next
        #egvects1, egvals1, vt = svd(L,0)        
        reversedevalueorder = egvals1.argsort()[::-1]
        egvects = egvects1[:, reversedevalueorder]               
        egvals = sort(egvals1)[::-1]                
        #rows in u are eigenfaces        
        u = dot(adjfaces_tr, egvects)
        u = u.transpose()               
        #NORMALISE rows of u
        for idx in range(numimgs):
            ui = u[idx]
            ui.shape = (self.__dim, self.__dim)
            norm = trace(dot(ui.transpose(), ui))            
            u[idx] = u[idx] / norm
        
        u = u[:self.__numFaces, :]
        self.__imgdata = ImgData(namelist, u, avgvals, facemat)      
        self.__weights = dot(u, adjfaces.transpose()).transpose() 
        

    def __makeModel(self):
        self.labelList = Labels()
        self.labelList.makeAllLabels(self.__imgdata.namelist)
        self.__models = list()
        for name in self.labelList.getLabellist():
            label = self.labelList.name2label(name)
            traindata = list()
            for imageidx in range(len(self.__weights)):
                imageweights = self.__weights[imageidx]
                facelabel = self.labelList.name2label(self.__imgdata.namelist[imageidx].partition("_")[0])
                
                if facelabel == label:
                    example = 1
                else:
                    example = -1
                
                traindata.append((example, self.__makeWeightTuplesList(imageweights)))
            temp_model = svmlight.learn(traindata, type='classification', verbosity=3)
            self.__models.append((name, temp_model))
    

    def __makeWeightTuplesList(self, weights):
        """Makes a list of tuples (index, value) needed for SVMlight"""
        weight_tuples_list = list()
        for weightidx in range(len(weights)):
            weight_tuple = ((weightidx+1), weights[weightidx])
            weight_tuples_list.append(weight_tuple)
        return weight_tuples_list


    def __runSVMModels(self, img):
        inputfacepixels = list(img.getdata())
        inputface = asfarray(inputfacepixels)
        pixlistmax = max(inputface)
        inputfacen = inputface / pixlistmax        
        inputface = inputfacen - self.__imgdata.avgvals
        usub = self.__imgdata.eigenfaces[:self.__numFaces,:]
        input_wk = dot(usub, inputface.transpose()).transpose()
        data = [(0, self.__makeWeightTuplesList(input_wk))]
        predictions = list()
        for (name, model) in self.__models:
            pred = svmlight.classify(model, data)
            predictions.append((name,pred[0]))
        return predictions    
    
    def __calculateScores(self, img):
        # Get all predictions from the SVM's
        predictions = self.__runSVMModels(img)
        
        #get all scores in a list
        scores = [element[1] for element in predictions]
        
        #calculate mean and average and normalize scores
        mean_score = average(scores)
        normalized_scores = scores - mean_score
        
        #calculate the maximum absolute score for threshold cutoff
        max_abs_score = max(abs(normalized_scores))
        return predictions, mean_score, normalized_scores, max_abs_score
        
    
    def __recognizeFromSVM(self, img):
        predictions, mean_score, normalized_scores, max_abs_score = self.__calculateScores(img)
        #print 'SVM INFO:'
        #print 'predictions:', predictions
        #print 'mean score:', mean_score, ', normalized score:', normalized_scores, ', max score (good threshold):', max_abs_score
        best = ''
        top = self.__threshold
        if max_abs_score < self.__threshold:
            return None
        for (name, prediction) in predictions:
            total_prediction = 0
            for (name2, prediction2) in predictions:
                prediction2 -= mean_score
                if name == name2:
                    #total_prediction += 2*prediction2
                    total_prediction += prediction2
                else:
                    total_prediction -= prediction2
            if total_prediction >= top:
                top = total_prediction
                best = name
        return best


    def __validateNumFaces(self, imgnamelist):
        """Make sure the selected number of eigenfaces is smaller than the number of images, and adjust where necessary"""
        numimgs = len(imgnamelist)
        if(self.__numFaces >= numimgs  or self.__numFaces == 0):
            print "Too many eigenfaces:", self.__numFaces
            self.__numFaces = numimgs / 2
            print "Changing numfaces to", self.__numFaces


    def __checkRecognizeReady(self, img):
        if not self.__trained:
            return False
        if not self.__validateImage(img):
            return False
        return True


    def __convertImage(self, img):
        return Image.fromstring("L", cv.GetSize(img), img.tostring())


    def __convertAllToPIL(self, imgarray):
        of_the_jedi = list()
        for (name, ipl_image) in imgarray:
            pil_image = self.__convertImage(ipl_image)
            of_the_jedi.append((name, pil_image))
        return of_the_jedi


    def __validateImage(self, img):
        if img == None:
            print 'Eigenfaces: No image'
            return False
        if not (type(img) == cv.iplimage):
            print 'Eigenfaces: No ipl image'
            return False
        if img.nChannels == 3:
            print 'Eigenfaces: Wrong number of channels'
            return False
        return True


    def __loadImg(self, filename):
        try:
            img = cv.LoadImage(filename, iscolor=cv.CV_LOAD_IMAGE_UNCHANGED)
        except IOError:
            return None
        if cv.GetSize(img) == (self.__dim, self.__dim):
            return img
        return None

def do_save(data, filename):
    with open(filename, 'wb') as opened_file:
        pickle.dump(data, opened_file, pickle.HIGHEST_PROTOCOL)

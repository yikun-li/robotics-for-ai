import time
import os
import math
import random 
import sys
import cPickle as pickle
import main_cv
import multiprocessing
import scipy.io as sio
import numpy
class CrossValidation(object):
    '''
    This class 
    '''
    def __init__(self,raddress,  waddress, k_fold):
        self.raddress = raddress
        self.waddress = waddress
        self.k_fold = k_fold
        try:
            self.rdata = pickle.load(os.path.join(self.raddress, 'data-label.dat'))
        except:
            self.rdata = []
    def readFolder(self, mode = "pic"):
        '''This function recursively reads folder in a path. Each folder name is counted as the class label and the files inside the data.'''
        if mode == "pic":
            #Reads Filename with picture extensions, and add their name and address to self.rdata
            if len(self.rdata) > 0:
                return
                
            for root, _, files in os.walk(self.raddress):
                className = os.path.basename(root)
                if className == "hist":
                    continue
                for filename in files:
                    path = os.path.join(root,filename)
                    (head, ext) = os.path.splitext(filename)
                    
                    if ext == ".png" or ext == ".jpg":
                        self.rdata.append((path,head,className))
            try:
                file = open(os.path.join(self.raddress, 'data-label.dat'),'w')
                pickle.dump(self.rdata, file)
            except:
                print "cannot open the data-label.dat file for writing"
                raise
        else:
            #Reads Filename with histogram npy extensions, and add their name and address to self.rdata
            self.raddress = self.waddress
            self.rdata = []
            for root, _, files in os.walk(self.raddress):
                className = os.path.basename(root)
                if className == "hist":
                    for filename in files:
                        path = os.path.join(root,filename)
                        (head, ext) = os.path.splitext(filename)
                        
                        if ext == ".npy":
                            self.rdata.append((path,head,className))
            try:
                file = open(os.path.join(self.raddress, 'data-label.dat'),'w')
                pickle.dump(self.rdata, file)
            except:
                print "cannot open the data-label.dat file for writing"
                raise
        return self.rdata
                
    def dataShuffler(self, data):
        '''
        This function receives a list, and shuffles it to another list of the same size
        '''
        shuffledData = []
        for _ in range(len(data)):
            index = random.randint(0,len(data) - 1)
            shuffledData.append(data.pop(index))
        return shuffledData
    
    def dataOrganizer(self):
        '''
        This function reads all the data, shuffles them, and then divides it to k different folds.
        For each fold a training and testing directory will be created to make it ready for testing.
        '''
        
        random.seed()
        
        #Having full path of all the files in one list
        nameList = [os.path.join(self.waddress,name) for name in os.listdir(self.waddress)]
        
        shuffledData = self.dataShuffler(nameList)
        
        #Slicing shuffled data in k different folds
        slicedData = []
        slices = int(math.floor(len(shuffledData)/self.k_fold))
        start = 0
        end = slices
        for _ in range(self.k_fold):
            slicedData.append(shuffledData[start:min(end, len(shuffledData))])
            start += slices
            end += slices    
        
        #extracting test and traisning set for each fold  
        self.setFolds(slicedData)
            
    def setFolds(self,slicedData):   
        '''
        This function sets the training and testing sets for each fold and then calls writeFolds function
        ''' 
        for i in range(self.k_fold):
            testSet = slicedData[i]
            trainingSet = []
            for j in range(self.k_fold):
                if j != i:
                    trainingSet.extend(slicedData[j])
                    
            self.writeFolds(i, testSet, trainingSet)
        
    def writeFolder(self, mode = "pic"):
        '''This function writes all the data into a single folder for easier inspection '''
        if mode == "pic":
            if len(self.rdata) < len(os.listdir(self.waddress)):
                return
            for path, head,_ in self.rdata:
                command = "cp -l " + path + " "+ os.path.join(self.waddress, head + ".png") 
                os.system(command)
        else:
            for path, head,_ in self.rdata:
                command = "cp -l " + path + " "+ os.path.join(self.waddress, head + ".npy") 
                os.system(command)
        return len([name for name in os.listdir(self.waddress) if os.path.isfile(os.path.join(self.waddress,name))])
    
    def writeFolds(self, currentFold, testSet, trainingSet):
        '''This function received current Fold, and two training and test list and writes them separately in a folder with current fold number
        and test and training set folders for easier cross validation'''
        
        writePath = os.path.join(self.waddress,str(currentFold))
        try:
            os.mkdir(writePath)
        except:
            print "The directory already exists"
            
        testSetPath = os.path.join(writePath, "testset")
        trainingSetPath = os.path.join(writePath, "trainingset")
        
        try:
            os.mkdir(testSetPath)
        except:
            print "The directory already exists"
        try:
            os.mkdir(trainingSetPath)
        except:
            print "The directory already exists"
        
        if len(testSet) == 1:
            command = "cp -l " + testSet[0] + " " + testSetPath
        else:
            for file in testSet:
                (dirpath, tail) = os.path.split(file)
                command = "cp -l " + file + " " + os.path.join(testSetPath,tail)
                os.system(command)
        if len(trainingSet) == 1:
            command = "cp -l " + trainingSet[0] + " " + trainingSetPath
        else:
            for file in trainingSet:
                (dirpath, tail) = os.path.split(file)
                command = "cp -l " + file + " " + os.path.join(trainingSetPath,tail)
    
                os.system(command)
                
    def load_labels(self, raddress = None):
        if not raddress:
            raddress = self.raddress
        path = os.path.join(raddress, 'data-label.dat')
        if os.path.isfile(path):
            f = open(path, 'r')
            self.rdata = pickle.load(f)
            return True
        return False

    def Validation(self, testLabels):
        clusterNum = testLabels[-1][1]
        testClassError = []
        labelIdx = 0
        #print "testLabel Size: ", len(testLabels)
        while (labelIdx < len(testLabels) ):      #Looping through all the rows in the testLabel array. Note that this nested while loop works like one loop.
            #print "Label idx is: ", labelIdx
            
            classIdx = testLabels[labelIdx][1]
            classArray = numpy.zeros(clusterNum)
            #print "Class idx is: ", classIdx
            while (labelIdx < len(testLabels) and classIdx == testLabels[labelIdx][1] ):        #Processing rows with the same className in testLabels. 
                for _,data,className in crossValidation.rdata:                 #For each picture, find it in ground truth database, and add one to the  
                    if abs(float(data) - testLabels[labelIdx][0]) <= 0.0001:   #respective ground truth class member.
                            classArray[int(className) - 1] += 1
                            break;
                labelIdx += 1
            
            
            max = numpy.max(classArray)
            if (max.size > 1):
                max = max[random.randint(0, len(max) - 1)]
            testClassError.append(1 - max / numpy.sum(classArray)) #Calculate the error for each class and append it to the list
            #print "class array is ", classArray
            #print "Error is: ", (1 - max / numpy.sum(classArray))    
            
        return numpy.sum(testClassError) / clusterNum        #returning Average error of this set
                
            
def matlabProcess(classNumber, path):
    command = "matlab -nosplash -nodesktop -r \" path(path, strcat(getenv('BORG'),'/Brain/data/hog_test/')); tmatrixreverse(" + str(classNumber) + ",'" + root + "')\""
    os.system(command)
    
if __name__ == "__main__":
    programStartTime = time.time()
    rpath = None
    wpath = None
    classNumber = 215
    try:
        rpath = str(sys.argv[1])
        wpath = str(sys.argv[2])
    except:
        pass
    #str1 = str(sys.argv[2])
    
        
    if rpath == None:
        rpath = "/data/scratch/experiments/data"
        wpath = "/data/scratch/experiments/dataw"
    #path = (os.environ['BORG'] + '/Brain/data/hog_test/matrixtest/')
    print "Reading path is", rpath[0:]
    

    #Copying and setting Cross validation and Folds
    rpath = os.path.join(rpath,"")
    wpath = os.path.join(wpath,"")
    
    crossValidation = CrossValidation(raddress = rpath, waddress = wpath, k_fold = 5)
    ''' OLD METHOD
                crossValidation.readFolder()        
                crossValidation.writeFolder()
                crossValidation.dataOrganizer()
                    
                #Calculating Picture information for all the files in the folds
                print "Picture database calculation started using ", multiprocessing.cpu_count(), "cpus."    
                for root, _, files in os.walk(wpath):
                    Parallel = True
                    base = os.path.basename(root)
                    if root != wpath and base != "hist" and (base == "trainingset" or base == "testset"):
                        print "Current Path is: ", root
                        main = main_cv.Main_pro(path = root, visualize=False, writing_method="cv", division="pro")
            
                        timer = time.time()
                        if Parallel:
                            main.process_handler(root)
                            
                        print "Calculation finished in:", time.time() - timer, " seconds"
                        result2 = main.cluster()
                        print "Operation for path: " + root + " finished in:", time.time() - timer, " seconds"
                        timer = time.time()
                        del main
                print "Extracting Image information is finished"
    '''
    crossValidation.readFolder("pic")
    
    crossValidation.writeFolder("pic")
    
    
    #Calculating the Picture information for the Top level folder - Then folds will be made out of histogram files
    print "Picture database calculation started using ", multiprocessing.cpu_count(), "cpus."    

    print "Current Path is: ", wpath
    main = main_cv.Main_pro(path = wpath, visualize=False, writing_method="cv", division="pro")

    timer = time.time()
    main.process_handler(wpath)   
    print "Calculation finished in:", time.time() - timer, " seconds"
    del main
    print "Extracting Image information is finished"
    
    
    #Reads the calculated hist folder, and requests Data organization from this folder
    crossValidation.raddress = os.path.join(crossValidation.waddress, "hist")   
    crossValidation.readFolder(mode = "hist")
    crossValidation.waddress = os.path.join(crossValidation.waddress, "hist") 
    crossValidation.dataOrganizer()
    
    for root, _, files in os.walk(crossValidation.waddress):
        Parallel = True
        base = os.path.basename(root)
        if root != wpath and base != "hist" and (base == "trainingset" or base == "testset"):
            print "Current Path is: ", root
            main = main_cv.Main_pro(path = root, visualize=False, writing_method="cv", division="pro") 
            result2 = main.cluster()
            del main
            
    #Train using Matlab on each training set
    processList = []
    counter = 0
    idx = 0
    for root, _, files in os.walk(wpath):
        Parallel = True
        base = os.path.basename(root)
        if root != wpath and base != "hist" and (base == "trainingset"):
            processList.append(multiprocessing.Process(target=matlabProcess, args=(classNumber,root,)))
            processList[idx].start()
            
            if idx % 8 == 0 and idx != 0:
                for jdx in range(8):
                    processList[counter + jdx].join()
                counter = idx
                print "Joined Processes, Counter is: ", counter
            idx += 1
        
    print "Calculating Centers and supervision is finished"
    
    '''
    while not crossValidation.load_labels():
        pass
    TotalError = 0
    FoldError = 0
    FoldErrorAverage = []
    for root, _, files in os.walk(wpath):
        Parallel = True
        base = os.path.basename(root)
        if root != wpath and base != "hist" and (base == "trainingset"):
            labels = sio.loadmat(os.path.join(root, "labels.mat"))
            labels = labels['labels']
            labels = numpy.multiply(labels, 100)
            labels = numpy.trunc(labels)
            labels = numpy.divide(labels, 100)
            labels = list(labels)
            Error = crossValidation.Validation(labels)
            FoldErrorAverage.append(Error)
    
    print FoldErrorAverage
    
    for i in FoldErrorAverage:
        TotalError += i
    
    print "Total Error is:", numpy.sum(FoldErrorAverage)/float(len(FoldErrorAverage))
    programEndTime = time.time() - programStartTime
    print "Total Calcuation time is: ", programEndTime
    '''
    print "Program Finished"
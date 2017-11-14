
import numpy
import logging
import util.nullhandler
import os
import divider
import gradient
import hog
import time
import sys
import math
import multiprocessing
import cv2.cv as cv
import cv2
from copy import deepcopy
import scipy.io as sio
import cProfile
logging.getLogger('Borg.Brain.Vision.hog.Main_pro').addHandler(util.nullhandler.NullHandler())

class Main_pro(object):
    def __init__(self, path ,visualize = False, writing_method = 'cv', division = "normal", column = 5, row = 5):
        self.logger = logging.getLogger("Borg.Brain.Vision.hog.Main_pro")
        self.visualize = visualize
        self.path = os.path.join(path,'')
        self.IDs = 0
        self.divider = divider.Divider()
        self.divider2 = divider.Divider()
        self.gradient = gradient.Gradient()
        self.hog = hog.HoG(writing_method)
        self.writing_method = writing_method
        self.IDnumbers = 0
        self.divisionOption = division
        self.column = column
        self.row = row
        
        self.dividerTime = []
        self.gradientTime = []
        self.histogramTime = []
        self.mainTime = 0
        self.cpu_count = multiprocessing.cpu_count()
        
    
                    
        
    def extractor(self):
        self.IDnumbers = 0
        
        for dirpath, _, filenames in os.walk(self.path):
            for file in filenames:
                
                if self.writing_method == "cv":
                    currentHog = []
                else:
                    currentHog = numpy.zeros((0,1))
                if dirpath.find("hist") != -1:
                    continue
                filePath = os.path.join(dirpath, file)
                filename, fileExtension = os.path.splitext(file)
                if not (fileExtension == ".cv" or fileExtension == ".hist" or fileExtension == ".sa" or fileExtension == ".dat" or fileExtension == ".npy" or fileExtension == ".txt"):
                    zimbo = time.time()
                    if filename == "tp_matrix":
                        continue
                    try:
                        currentPicture = cv.LoadImage(filePath)
                    except:
                        continue
                    
                    
                    (dx, dy) = self.gradient.sobelGradient(currentPicture)
                    Tangent = self.gradient.tangent(dx, dy)
                    Magnitude = self.gradient.Magnitude(dx, dy)
                                    
                    tangentList = self.divider.divide(Tangent, column, row, option = self.divisionOption)
                            
                    MagnitudeList = self.divider.divide(Magnitude, column, row, option = self.divisionOption)
                    List = zip(tangentList, MagnitudeList)
                    
                    for tangent, magnitude in List:
                        if self.writing_method == "cv" :
                            currentHog.append(self.hog.HoG(tangent, magnitude, self.writing_method))
                        elif self.writing_method == "numpy":
                            tempHist = self.hog.HoG(tangent, magnitude, self.writing_method)
                            currentHog = numpy.vstack((currentHog, tempHist))

                else:
                    continue
                  
                #a = time.time()
                self.store_numbers(currentHog, dirpath, self.writing_method, filename)
                self.IDnumbers += 1
                
                #if self.IDnumbers % 100 == 1:
                #    print "Time:", (time.time() - zimbo) * 1000, ' ms' 
                #print time.time() - a
                
        
            
        return self.IDnumbers
        
               
    
        
    def store_numbers(self, HoGList, path, method, filename):
        writingPath = os.path.join(path , 'hist')
        try:
            os.makedirs(writingPath)
        except:
            pass
        
        '''if method == "cv":
            matrix = cv.CreateMat(1, len(HoGList) * 8, cv.CV_32FC1)
            index = 0
            for histogram in HoGList:
                for x in range(8):
                    matrix[0, index * 8 + x] = cv.QueryHistValue_1D(histogram, x)
                index += 1
                
            cv.Save(writingPath + '/' + filename + '.cv', matrix)'''
        if method == "cv":
            index = 0
            matrix = numpy.zeros((1, len(HoGList) * 8))
            for histogram in HoGList: 
                for x in range(8):
                    matrix[0, index * 8 + x] = cv.QueryHistValue_1D(histogram, x)
                index += 1
            numpy.save(os.path.join(writingPath,filename), matrix) 
        elif method == 'numpy':
            numpy.save(os.path.join(writingPath,filename), HoGList)
            
    def hierarchical_cluster(self, path=""):
        if path == "":
            path = self.path
            
        filelabels = []
        matrix = []
        
        if self.writing_method != "cvold":
            data = numpy.array([])

        
        for dirpath, _, filenames in os.walk(self.path):
            for file in filenames:      
                   
                filePath = os.path.abspath(dirpath) + '/' + file
                print filePath
                filename, fileExtension = os.path.splitext(file)
                if fileExtension == ".cv":
                    filelabels.append(file)
                    datafile = cv.Load(filePath)
                    matrix.append(datafile)
                if fileExtension == ".npy":
                    if filename == "tp_matrix":
                        continue
                    filelabels.append(filename)
                    temp = numpy.load(filePath)
                    if data.shape[0] < 1:
                        data = temp
                    else:
                        data = numpy.vstack((data, temp))
                        
        result = fastcluster.linkage(data, method='single', metric='euclidean', preserve_input=True)
        print result
        result2 = cluster.fcluster(result, 1.141)
        print result2
        
    def cluster(self, path = "", writing_method = ""):
        if writing_method != "":
            self.writing_method = writing_method
        if path == "":
            path = self.path
            
        filelabels = []
        matrix = []
        
        if self.writing_method != "cvold":
            data = numpy.array([])

        
        for dirpath, _, filenames in os.walk(self.path):
            print "Current Path is: ", dirpath
            #if os.path.basename(dirpath) != "hist":
            #    continue
            for file in filenames:      
                   
                filePath = os.path.join(os.path.abspath(dirpath),file)
                filename, fileExtension = os.path.splitext(file)
                if fileExtension == ".cv":
                    filelabels.append(file)
                    datafile = cv.Load(filePath)
                    matrix.append(datafile)
                if fileExtension == ".npy":
                    if filename == "tp_matrix":
                        continue
                    filelabels.append(filename)
                    temp = numpy.load(filePath)
                    if data.shape[0] < 1:
                        data = temp
                    else:
                        data = numpy.vstack((data, temp))
        
        print "Hisograms Gathered"
        if self.writing_method == "cvold":        
            datacv = cv.CreateMat(len(matrix), cv.GetSize(matrix[0])[0], cv.CV_32FC1)   
            labels = cv.CreateMat(len(matrix),1, cv.CV_32S)
        
            index = 0
            
            for histogram in matrix:
                for x in range(cv.GetSize(matrix[0])[0]):
                    datacv[index,x] = cv.QueryHistValue_1D(histogram, x)
                index += 1 
                   
            temp = time.time()
            now = time.gmtime()    
            print "Clustering started at: ", now[3], ":",now[4], ":",now[5]
            
            
            cv.KMeans2(datacv, nclusters, labels, (type, max_iter, epsilon)) 
            print "Clustering end at: ", now[3], ":",now[4], ":",now[5]
            print "Clustering Duration: " , time.time() - temp , " seconds"
            
            self.store_labels(labels, filelabels)
            
        else:
            
            self.store_data_matlab(data,filelabels)
            #clustering = kmeans.Kmeans(data)
            #labels = clustering.cluster()
            #self.store_labels_hist(None, filelabels)
        return 0
    
    def store_labels(self, labels, filenames):
        finalLabels = []
        for x in range(len(filenames)):
            finalLabels.append((labels[x,0], filenames[x]))
            
        f = open( os.path.join(self.path , 'labels.dat'), 'w')
        f.write(str(finalLabels))
        f.close
        
    def store_labels_hist(self, labels, filenames):
        numpy.save(os.path.join(self.path,'labels.npy'), labels)
        finalLabels = []
        for x in range(len(filenames)):
            finalLabels.append((labels[x], filenames[x]))
            
        with open( os.path.join(self.path,'labels.dat'), 'w') as f:
            f.write(str(finalLabels))  
           
        
               
    def store_data_matlab(self, data, filenames, directtomat = True):
        #Stores data to be read by matlab. Either directly to .mat format or to txt format
        
        
        if directtomat:
            path = os.path.join(self.path ,'matlabdata')  
            sio.savemat(path, {'matlabdata':data})
        else: 
            path = os.path.join(self.path, 'matlabdata.dat')  
            numpy.savetxt(path, data, delimiter=',')
        
        
        
        
        with open( os.path.join(self.path ,'matlablabels.dat'), 'w') as f:
            for x in filenames:
                f.write(str(x) + "\n")
        
    def store_clusterMeans(self, means):
        pass
    
    def extract_clusterMeans(self, labels, data):    
        pass
    
    def process_handler(self, path):
        fileList = []
        cpu_count = multiprocessing.cpu_count() / 2
        for dirpath, _, filenames in os.walk(path):
            for file in filenames:
                if dirpath.find("hist") != -1:
                    continue
                filePath = os.path.join(dirpath ,file)
                filename, fileExtension = os.path.splitext(file)
                if not (fileExtension == ".cv" or fileExtension == ".hist" or fileExtension == ".sa" or fileExtension == ".dat" or fileExtension == ".npy" or fileExtension == ".txt"):
                    if filename == "tp_matrix":
                        continue
                    fileList.append(filePath)
        chunk = int(len(fileList)/cpu_count)
        tempList = []
        #pool = multiprocessing.Pool(processes=cpu_count)              # start 4 worker processes
        processList = []
        lock = multiprocessing.Lock()
        #self.parallel_extractor(lock,path,fileList)
        
        
        for x in range(cpu_count):
            if x == cpu_count - 1:
                print x*chunk, " :to: ", (len(fileList) -1)
                tempList.append(deepcopy(fileList[(x*chunk):(len(fileList) -1)]))
            else:
                print x*chunk, " :to: ", ((x+1)*chunk)
                tempList.append( deepcopy(fileList[(x*chunk):((x+1)*chunk)]) )
                
            #For Profiling
            #processList.append(multiprocessing.Process(target=self.profiler, args=(lock,path, tempList[x],)))
            #For NormalUse
            processList.append(multiprocessing.Process(target=self.parallel_extractor, args=(lock,path, tempList[x],)))
            processList[x].start()
            
        processList[cpu_count-1].join()
        time.sleep(1)
        
        
    #print len(tempList[0]),len(tempList[1]),len(tempList[2]),len(tempList[3])
    #pool.map(parallel_extractor, tempList)
    
    def profiler(self, lock,path,tempList):
        cProfile.run('self.parallel_extractor(lock, path, tempList)', 'profileResult.txt')
            
        
    def parallel_extractor(self,lock, path, tempList):
        
        timeAverage = 0;
        
        writing_method = "numpy"
        divisionOption = "pro"
        column = 5
        row = 5
        storeList = []
        IDnumbers = 0
        
        for filePath in tempList:
            if writing_method == "cv" :
                currentHog = []
            elif writing_method == "numpy":
                currentHog = numpy.zeros((1,0))
                
            zimbo = time.time()
            try:
                currentPicture = cv.LoadImage(filePath)
            except:
                continue
                            
            #(Tangent, Magnitude) = self.gradient.cannyTangent(currentPicture,False, T1 = 20, T2 = 250)
            numpy_pic = numpy.asarray(currentPicture[:,:])

            (dx, dy) = self.gradient.sobelGradient(currentPicture)
            Tangent = self.gradient.tangent(dx, dy)
            Magnitude = self.gradient.Magnitude(dx, dy)
                            
            tangentList = self.divider.divide(Tangent, column, row, option = divisionOption)
                    
            MagnitudeList = self.divider.divide(Magnitude, column, row, option = divisionOption)
            
            List = zip(tangentList, MagnitudeList)
                    
            for tangent, magnitude in List:
                        
                if writing_method == "cv" :
                    currentHog.append(self.hog.HoG(tangent, magnitude, writing_method))
                elif writing_method == "numpy":
                    tempHist = self.hog.HoG(tangent, magnitude, writing_method)
                    
                    tempHist = numpy.transpose(tempHist)
                    #print tempHist.shape
                    currentHog = numpy.hstack((currentHog, tempHist))
        
            dirpath ,file = os.path.split(filePath)
            filename, _ = os.path.splitext(file)
            
            dirpath = os.path.join(str(dirpath) , "")
            
            storeList.append([currentHog, filename])
            
            IDnumbers += 1
            
            timeAverage += (time.time() - zimbo) * 1000
            if IDnumbers % 100 == 1:
                lock.acquire()
                    
                print "Average Time:", timeAverage / 100, ' ms'
                timeAverage = 0
                   
                lock.release()
        for list in storeList:
            self.store_numbers(list[0], dirpath, writing_method, list[1])
            
        return IDnumbers 
    
if __name__ == "__main__":
    path = None
    str1 = None
    do = False
    try:
        path = str(sys.argv[1])
    except:
        pass
    #str1 = str(sys.argv[2])
    
        
    if path == None:
        path = ('/home/borg/data/aug-data-30-500w')
        
        #path = ('/data/scratch/experiments/data/59')
        #path = '/home/borg/data_test/1/trainingset/'
        #path = ('/home/borg/DATA/Shantia/Labeled Cluster/labelc/')
        #path = "/dev/shm/images/webcam/"
    #path = (os.environ['BORG'] + '/Brain/data/hog_test/matrixtest/')
    print path[0:]
    '''
    column = 5
    row = 5
    Parallel = True
    main = Main_pro(path ,visualize = True, writing_method = 'numpy', division = "pro")
    
    
    print "Picture database calculation started using ", multiprocessing.cpu_count(), "cpus."
    timer = time.time()
    if Parallel:
        main.process_handler(path)
        pass
    else:
        result = main.extractor()
        pass
    
    print "Calculation finished in:", time.time() - timer, " seconds"
    
    #dTime = numpy.array(main.dividerTime)
    #gTime = numpy.array(main.gradientTime)
    #hTime = numpy.array(main.histogramTime)
    #print "total time is approx:", numpy.sum(dTime) + numpy.sum(dTime) + numpy.sum(hTime)
    #print "divider Time Average per picture", numpy.sum(dTime)/len(main.dividerTime)
    #print "Gradient  Time Average per picture", numpy.sum(gTime)/len(main.gradientTime)
    #print "Histogram Time Average per picture", numpy.sum(hTime)/len(main.histogramTime)
    #result1 = main.hierarchical_cluster()
    print "Preparin Data in Matlab readble format"
    timer = time.time()
    result2 = main.cluster()
    
    print "Operation finished in:", time.time() - timer, " seconds"
    
    print "finished"
    '''
    classNumber = 600
    root = path
    distance = 'sqEuclidean'
    command = "matlab -nosplash -nodesktop -r \" path(path, strcat(getenv('BORG'),'/brain/data/hog_test/')); tmatrixreverse(" + str(classNumber) + ",'" + root + "', '" + distance + "')\""
    os.system(command)
    

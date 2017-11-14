#!/usr/bin/python

"""example stand-alone vision module"""
import sys
import cv2
import time
import logging
import util.nullhandler
import util.loggingextra
import math

from util.ticker import Ticker
from util.timer import Timer

import configparse
import util.vidmemreader

import numpy as np
#import roslib; roslib.load_manifest('')

from abstractvisionmodule import AbstractVisionModule
try:
    from pybrain.tools.shortcuts import buildNetwork
    from pybrain.supervised.trainers import BackpropTrainer
    from pybrain.datasets import SequentialDataSet
    from pybrain.structure import SigmoidLayer
    from pybrain.structure import LSTMLayer
except:
    print "Pybrain not installed"
import itertools 
logging.getLogger('Borg.Brain.Vision.smokeDetect').addHandler(util.nullhandler.NullHandler())


vid_files = [
    '/home/borg/Pictures/Indoor_outdoor(short_distance)smoke/indoor_daytime_smoke.avi',
    '/home/borg/Pictures/Smoke_flame_like_moving_object/smoke_or_flame_like_object_1.avi',
    '/home/borg/Pictures/Smoke_flame_like_moving_object/smoke_or_flame_like_object_2.avi',
    '/home/borg/Pictures/Smoke_flame_like_moving_object/smoke_or_flame_like_object_3.avi',
    '/home/borg/Pictures/Smoke_flame_like_moving_object/smoke_or_flame_like_object_4.avi',
    '/home/borg/Pictures/Smoke_flame_like_moving_object/smoke_or_flame_like_object_8.avi'
]

# Available size: 1840 x 1150
 
#   1 3
#   2 4
#   

class smokeDetect(AbstractVisionModule):
    """example vision class. Detects faces using the openCV face detector"""

    def __init__(self, host, port, update_frequency, source="webcam", verbose=False):
        self.logger = logging.getLogger("Borg.Brain.Vision.smokeDetect")
        super(smokeDetect, self).__init__(host, port) 
        self.update_frequency = update_frequency
        self.__ticker = Ticker(frequency=update_frequency)
        
        self.verbose = verbose

        self.source = [source]
        self.vid_mem_reader = util.vidmemreader.VidMemReader(self.source)
        # set the type of objects that someone can detect
        self.set_known_objects(['smoke'])
        
        self.windowLocations = []
        
        self.labels = []
        self.data = []
        
        self.frameAverage = 5
        self.svm = None
        self.load_svm()
        
    def __del__(self):
        cv2.destroyAllWindows()  
        
    def init_nn(self, datain, dataout):
        INPUTS = 5
        OUTPUTS = 1
        HIDDEN = 20
        self.net = buildNetwork(INPUTS, HIDDEN, OUTPUTS, hiddenclass = LSTMLayer, outclass = SigmoidLayer, recurrent = True, bias = True)
        self.ds = SequentialDataSet(INPUTS, OUTPUTS)           
        
        for x,y in itertools.izip(datain,dataout):
            self.ds.newSequence()
            self.ds.appendLinked(tuple(x), tuple(y))
            
        self.net.randomize()

        trainer = BackpropTrainer(self.net, self.ds)

        for _ in range(1000):
            print trainer.train()
            
    def train(self):
        pass
    
    def stop(self):
        cv2.destroyAllWindows()

    def get_vid(self):
        if not hasattr(self, 'vid_idx'):
            self.vid_idx = 0
        else:
            self.vid_idx += 1
            if self.vid_idx >= len(vid_files):
                self.vid_idx = 0
        
        print "NOW LOADING ", vid_files[self.vid_idx]
        return cv2.VideoCapture(vid_files[self.vid_idx])
    
    def get_image_from_video(self,vid, size = (320, 240)):
        (result, image) = vid.read()
        if not result:
            #vid.release()
            vid = self.get_vid()
            (_, image) = vid.read()
        image = cv2.resize(image, size)
        return image
    
    def run(self):
        """start loop which gets a new image, then processes it"""
        #cv2.namedWindow("color_hist", cv2.cv.CV_WINDOW_NORMAL) 
        im = None
        while im == None:
            im = self.vid_mem_reader.get_latest_image()
            if im == None:
                print "not receiving images yet..."
                time.sleep(0.2)

        vid = self.get_vid()
        idx = -1   
        while True:
            histogramList=[]
            imageList = []
            colorList = []
            imageOrigList = []
            croppedList = []
            timeBlock = time.time()
            heartBeat = time.time()
            idx = -1
            while time.time() - timeBlock < 3:
                idx += 1
                if idx > 300:
                    break;
                #Getting pictures for 5 seconds
                if time.time() - heartBeat > 2:
                    self._AbstractVisionModule__send_heartbeat()
                    heartBeat = time.time()
               
                #temp = self.vid_mem_reader.get_latest_image()[0] 
                #image = np.asarray(temp[:,:])
                
                
                #image = self.get_image_from_video(vid)
                (result, image) = vid.read()
                if not result:
                    #vid.release()
                    vid = self.get_vid()
                    (_, image) = vid.read()
                image = cv2.resize(image, (320,240))
                if self.vid_idx == 0:
                    w, h, _ = image.shape
                    image = image[100:240, :, :]
                image = cv2.resize(image, (320,240))
            
                image_gs = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)   
                      
                colorList.append(image) 
                imageOrigList.append(image_gs)    
                image_gs = cv2.blur(image_gs, (15,15))
                imageList.append(image_gs)
                    
                #Calculating Histogram of Gray scale Image and Normalizing it
                hist = cv2.calcHist(images = [image_gs], channels = [0], mask = None, histSize = [32], ranges = [0,256])
                cv2.normalize(hist,hist,0,255,cv2.NORM_MINMAX)
                histogramList.append(hist)
                
                #Calculating Histogram of the Color Image
                curve = hist_curve(image, 128)
                           
                if self.verbose:
                    image_gs_rs = cv2.resize(image_gs, (613,408))
                    cv2.imshow("Smoothed_Gray_Scale", image_gs_rs)
                    cv2.moveWindow("Smoothed_Gray_Scale", 55, 0)                
                    cv2.waitKey(1)
                
            self._AbstractVisionModule__send_heartbeat()
                        
            contourList = []
            for i in range(len(imageList) - 1):
                
                if time.time() - heartBeat > 4:
                    self._AbstractVisionModule__send_heartbeat()
                    heartBeat = time.time()
                
                motion, equalized_motion, denoised_motion, denoised_mask = self.extractMotion(imageList[i], imageList[i+1])                        
                
                #Calculating Normalized Histogram of motion image
                motion_hist = hist_curve(equalized_motion, bin = 64, mask = denoised_motion)                
                y = cv2.resize(motion_hist, (320,240))
                
                #Cropped Grayscale image based on motion
                final = np.zeros(image_gs.shape, dtype = np.int8)
                final = cv2.add(final, imageOrigList[i], mask = denoised_mask, dtype = cv2.CV_8UC1)
                
                #Getting cropped color image
                finalColor = self.cropColor(colorList[i], denoised_mask)
                croppedList.append(finalColor)
                finalOrigColor = colorList[i]
                
                #Calculating Contours
                contours, hierarchy = cv2.findContours(denoised_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                sanityList = []
                #Accept frames having more than one contour
                if len(contours) > 1: 
                    for array in contours:
                        temp_data = []
                        #Calculate contour structures
                        area = cv2.contourArea(array)
                        if area < 100:
                            continue
                        center, radius = cv2.minEnclosingCircle(array)
                        circleArea = math.pi * math.pow(radius, 2)
                        isConvex = cv2.isContourConvex(array)
                        boundingRect = cv2.boundingRect(array)
                        rectArea = boundingRect[2] * boundingRect[2]
                        ellipseRect = cv2.fitEllipse(array)
                        rectArea = ellipseRect[2] * ellipseRect[2]
                        temp_data.extend([area, circleArea, rectArea])
                        temp_data.extend([radius, float(isConvex)])
                        
                        xpos , ypos , w , h = boundingRect
                        contour_hist = cv2.calcHist(images = [equalized_motion[xpos: xpos + w, ypos:ypos + h]], 
                                            channels = [0], 
                                            mask = denoised_motion[xpos: xpos + w, ypos: ypos + h], 
                                            histSize = [32], 
                                            ranges = [0,256])  
                        cv2.normalize(hist,hist,0,255,cv2.NORM_MINMAX)

                        self.logger.debug("contour hist shape : %s", str(contour_hist.shape))
                        temp_data.extend(np.array(contour_hist).reshape(-1,).tolist())
                        #Add deficiency information
                        if not isConvex:
                            temp_data.extend(self.convexDeficiency(array))
                        else:
                            #Put zero for all the expected values
                            temp_data.extend([0.,0.,0.,0.,0.])
                        
                        #Ignore big areas and areas which look rectangular
                        if area > 100 and rectArea / area > 2.5:
                            sanityList.append(center)
                            self.data.append(temp_data)
                            #Automatic Labeling based on video
                            if self.vid_idx == 0:
                                self.labels.append(1.0)
                            else:
                                self.labels.append(0.0)
                        
                        test = np.array(temp_data)
                        test = test.astype(np.float32)
                        
                        if self.svm.predict(test) == 1:
                            tempol = np.copy(finalOrigColor)
                            cv2.drawContours(tempol, array, -1, (255,0,0))
                            tempol_rs = cv2.resize(tempol, (614, 408))
                            cv2.imshow("Finall", tempol_rs)
                            cv2.moveWindow("Finall", 1800 + 45, 460)
                            cv2.waitKey(1)
                        
                mean = np.mean(np.asarray(sanityList))
                std = np.std(np.asarray(sanityList))
                contourList.append((mean,std, contours))    
                if self.verbose: 
                    motion_rs = cv2.resize(motion, (613,408))
                    cv2.normalize(motion_rs, motion_rs, 0, 255, cv2.NORM_MINMAX)
                    cv2.imshow("motion", motion_rs)
                    
                    cv2.moveWindow("motion", 640 + 45, 0)
                    temp_rs = cv2.resize(denoised_motion, (613,408))
                    cv2.imshow("equap", temp_rs)
                    cv2.moveWindow("equap", 1280 + 45, 0)
                    finalColor_rs = cv2.resize(finalColor, (613,408))
                    cv2.imshow("cropped-motion", finalColor_rs)
                    cv2.moveWindow("cropped-motion", 0 + 45, 460)
                    y_rs = cv2.resize(y, (613,408))
                    cv2.imshow("cropped-motion-hist", y_rs)
                    cv2.moveWindow("cropped-motion-hist", 640 + 45, 460)
                    cv2.waitKey(1)
                    
                    #time.sleep(0.1)
                    '''
                    DISCRETE FOURIER TRANSFORM
                    dft_im = dft(denoised_motion)
                    pyrdown = dft_im
                    for i in range(4):
                        pyrdown = cv2.pyrDown(pyrdown)
                    w,h = dft_im.shape
                    pyrdown = cv2.resize(pyrdown, (h,w))
                    cv2.imshow("pyrdown", pyrdown)
                    '''
                #Scoring detections in frame sequences
                Score = 0
                minStd = 100
                for mean, std, contours in contourList:
                    
                    #print 'Standard deviation of contours', std
                    if std > 0 and std < 50:
                        if std < minStd:
                            minStd = std
                            bestContour = contours
                            minMean = mean
                        Score += 1
                    elif std > 50:
                        Score -= 30
                    elif std > 39:
                        Score -= 1
                    else:
                        #Score -= 1
                        pass
                if Score > 0:
                    self.add_property('name', 'smoke')
                    self.add_property('center', minMean)
                    self.add_property('image_path', '/dev/shm/smoke.png')
                    self.store_observation()
                    
                    tempo = np.copy(finalOrigColor)
                    cv2.drawContours(tempo, contours, -1, (255,0,0))
                    cv2.imwrite('/dev/shm/smoke.png', tempo)

                    tempo_rs = cv2.resize(tempo, (614, 408))
                    cv2.imshow("Final", tempo_rs)
                    cv2.moveWindow("Final", 1280 + 45, 460)
                    cv2.waitKey(1)
                    self.update()
                   
            self.save(self.data, self.labels)                
            self.update()
            
    def save(self, data, labels):
        data = np.array(data)
        labels = np.array(labels)
        
        self.logger.info("Data size is %s", str(data.shape))
        self.logger.info("label size is %s", str(labels.shape))
        np.save("/dev/shm/data", data)
        np.save("/dev/shm/label", labels)
        
        np.savetxt("/dev/shm/data.txt", data)
        np.savetxt("/dev/shm/label.txt", labels)
        
        
    def convexDeficiency(self, contour):
        '''
        Calculates the convex deficiency and returns a list of:
        1- mean and standard deviation of deficiency depth, and length of the starting/ending point of the deficiency
        2- Number of deficiencies
        3- perimiter of each defeciency  
        @contour The contour of the image
        '''            
        convex_hull = cv2.convexHull(contour, returnPoints = False)
        convex_defects = cv2.convexityDefects(contour, convex_hull)
        self.logger.debug("convex_defects is: %s", str(convex_defects))
        
        defect_number = len(convex_defects)
        distance_list = []
        depth_list = []
        premeter_list = []
        if defect_number:
             
            for defect in convex_defects:
                self.logger.debug("current defect is: %s", str(defect))
                start_idx, end_idx, farthest_pt, pt_depth, = defect[0]
                self.logger.debug("start/end contour points are %s and %s ", str(contour[start_idx][0]), str(contour[end_idx][0]))
                distance = math.pow((contour[start_idx][0][0] - contour[end_idx][0][0]), 2) + math.pow((contour[start_idx][0][1] - contour[end_idx][0][1]), 2)
                distance_list.append(distance)
                depth_list.append(pt_depth)
        
        mean_distance = np.mean(np.asarray(distance_list))
        std_distance = np.std(np.asarray(distance_list))
        mean_depth = np.mean(np.asarray(depth_list))
        std_depth = np.std(np.asarray(depth_list))        
        
        return [mean_distance, std_distance, mean_depth, std_depth, defect_number]
    
    def cropColor(self, image, mask):
        #Crop a color image based on a mask
        ch1 = np.zeros(mask.shape, dtype = np.int8)
        ch2 = np.zeros(mask.shape, dtype = np.int8)
        ch3 = np.zeros(mask.shape, dtype = np.int8)   

        c1 = np.copy(image[:,:,0])
        c2 = np.copy(image[:,:,1])
        c3 = np.copy(image[:,:,2])
                 
        ch1 = cv2.add(ch1, c1, mask = mask, dtype = cv2.CV_8UC1)
        ch2 = cv2.add(ch2, c2, mask = mask, dtype = cv2.CV_8UC1)
        ch3 = cv2.add(ch3, c3, mask = mask, dtype = cv2.CV_8UC1)
        
        finalColor = cv2.merge((ch1,ch2,ch3))
        
        return finalColor

    def extractMotion(self, image1, image2):
        #Background Subtraction    
        motion = cv2.absdiff(image1, image2)
            
        #Histogram Equalization of the motion image
        #TODO: Test without equalization and only normalization
        #eqaulized_motion = cv2.equalizeHist(motion)
        #eqaulized_motion = np.zeros(motion.shape)
        eqaulized_motion = cv2.normalize(src = motion,alpha = 0, beta = 255, norm_type = cv2.NORM_MINMAX)
        #Removing Noise by opening and closing the image
        se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8,8))
        temp = cv2.morphologyEx(eqaulized_motion, cv2.MORPH_OPEN, se)
                        
        se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        temp = cv2.morphologyEx(temp, cv2.MORPH_CLOSE, se)
        
        denoised_motion = temp
        
        #Making a mask from the closed/opened image
        denoised_mask = np.zeros(image1.shape, dtype = np.uint8)
        denoised_mask[denoised_motion > 70] = 1
                
        return motion, eqaulized_motion, denoised_motion, denoised_mask
    
    def deltaHistogram(self, histogramList, image_gs, hist):
        '''
        This function doesn't work, it should calculate delta histograms
        '''
        for i in range(len(histogramList) - 1):
            deltaHist = np.absolute(np.subtract(histogramList[i], histogramList[i+1]))
            deltaHist /= np.sum(deltaHist)
            
            nans = np.isnan(deltaHist)
            deltaHist[nans] = 0;
            
            deltaImage = np.zeros(image_gs.shape)
            binStep = image_gs.shape[1] / deltaHist.shape[0] 
            
            print deltaHist
            
            for i in range(len(hist) - 1):
                pt1 = (binStep * i, image_gs.shape[0] - (deltaHist[i] * image_gs.shape[0]))
                pt2 = (binStep * i+1, image_gs.shape[0] - (deltaHist[i+1] * image_gs.shape[0]))
                color = 255
                cv2.line(deltaImage, pt1, pt2, color, 2,8,0)
            
            if self.verbose:
                cv2.imshow("delta_hist", deltaImage)
                cv2.moveWindow("delta_hist", 1600, 0)
                cv2.waitKey(10)
            time.sleep(0.1)
        
    def archive(self, image):
        #Adds all types of image conversion to the specified list
        pass

    def load_svm(self):
        self.svm = cv2.SVM()
        try:
            self.svm.load("/dev/shm/svm.model")
        except:
            self.svm = None
            
def shift_dft(src, dst=None):
    '''
        Rearrange the quadrants of Fourier image so that the origin is at
        the image center. Swaps quadrant 1 with 3, and 2 with 4. 
       
        src and dst arrays must be equal size & type
    '''
    
    if dst is None:
        dst = np.empty(src.shape, src.dtype)
    elif src.shape != dst.shape:
        raise ValueError("src and dst must have equal sizes")
    elif src.dtype != dst.dtype:
        raise TypeError("src and dst must have equal types")
    
    if src is dst:
        ret = np.empty(src.shape, src.dtype)
    else:
        ret = dst
    
    h, w = src.shape[:2]
    
    cx1 = cx2 = w/2
    cy1 = cy2 = h/2
    
    # if the size is odd, then adjust the bottom/right quadrants
    if w % 2 != 0:
        cx2 += 1
    if h % 2 != 0:
        cy2 += 1 
        
    # swap quadrants
    
    # swap q1 and q3
    ret[h-cy1:, w-cx1:] = src[0:cy1 , 0:cx1 ]   # q1 -> q3
    ret[0:cy2 , 0:cx2 ] = src[h-cy2:, w-cx2:]   # q3 -> q1
    
    # swap q2 and q4
    ret[0:cy2 , w-cx2:] = src[h-cy2:, 0:cx2 ]   # q2 -> q4
    ret[h-cy1:, 0:cx1 ] = src[0:cy1 , w-cx1:]   # q4 -> q2
    
    if src is dst:
        dst[:,:] = ret
    
    return dst

def dft(im):
    
    #im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    h, w = im.shape[:2]
    
    realInput = im.astype(np.float64)
    
    # perform an optimally sized dft
    dft_M = cv2.getOptimalDFTSize(w)
    dft_N = cv2.getOptimalDFTSize(h)

    # copy A to dft_A and pad dft_A with zeros
    dft_A = np.zeros((dft_N, dft_M, 2), dtype=np.float64)
    dft_A[:h, :w, 0] = realInput
    
    # no need to pad bottom part of dft_A with zeros because of
    # use of nonzeroRows parameter in cv2.dft()
    cv2.dft(dft_A, dst=dft_A, nonzeroRows=h)
    
    cv2.imshow("win", im)
    
    # Split fourier into real and imaginary parts
    image_Re, image_Im = cv2.split(dft_A)
    
    # Compute the magnitude of the spectrum Mag = sqrt(Re^2 + Im^2)
    magnitude = cv2.sqrt(image_Re**2.0 + image_Im**2.0)
    
    # Compute log(1 + Mag)
    log_spectrum = cv2.log(1.0 + magnitude)
    
    # Rearrange the quadrants of Fourier image so that the origin is at
    # the image center
    shift_dft(log_spectrum, log_spectrum)

    # normalize and display the results as rgb
    cv2.normalize(log_spectrum, log_spectrum, 0.0, 1.0, cv2.cv.CV_MINMAX)
    cv2.imshow("magnitude", log_spectrum)

    cv2.waitKey(1)
    return log_spectrum

        
def train_svm():
    data = np.load("/dev/shm/data.npy")
    labels = np.load("/dev/shm/label.npy")
    
    data = data.astype(np.float32)
    labels = labels.astype(np.float32)
    svm_trainer = cv2.SVM()
    params = dict( kernel_type = cv2.SVM_RBF, 
                       svm_type = cv2.SVM_C_SVC )
    cgrid = [1, 10 ,100, 1000, 10000, 100000]
    gammagrid = [0.000001, 0.00001, 0.0001, 0.001, 0.01]
    svm_trainer.train_auto(data, labels, varIdx = None, sampleIdx = None, params = params, k_fold = 10, Cgrid = cgrid, gammaGrid = gammagrid)
    
    print svm_trainer
    svm_trainer.save("/dev/shm/svm.model")
    
def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (_, _) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

def hist_curve(im, bin = 32, mask = None):
    bins = np.arange(bin).reshape(bin,1)
    h = np.zeros((300,bin,3))
    if len(im.shape) == 2:
        color = [(255,255,255)]
    elif im.shape[2] == 3:
        color = [ (255,0,0),(0,255,0),(0,0,255) ]
    for ch, col in enumerate(color):
        hist_item = cv2.calcHist([im], [ch], mask, [bin], [0,256])
        cv2.normalize(hist_item,hist_item,0,255,cv2.NORM_MINMAX)
        hist=np.int32(np.around(hist_item))
        pts = np.int32(np.column_stack((bins,hist)))
        cv2.polylines(h,[pts],False,col)
    y=np.flipud(h)
    return y
          
def usage():
    print "You should add the configuration options on the command line."
    print ""
    print "Usage: ", sys.argv[0], " host=<controller_host> port=<controller_port> updatefreq=<update frequency>"
    print ""

if __name__ == "__main__":
    print len(sys.argv)
    if len(sys.argv) == 2: 
        args =  sys.argv[1:]
        print args
        if args[0] == "train":
            train_svm()
            exit()
        else:
            exit()
    if len(sys.argv) < 3:
        usage()
        exit()

    sec = "smokedetect" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)

    update_frequency = config_dict.get_option(sec, "updatefreq")
    source = config_dict.get_option(sec, "video_source")
    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    use_verbose = config_dict.get_option(sec, "verbose", "False")
    sect = config_dict.get_section(sec)
    print sect

    if not (update_frequency and controller_ip and controller_port):
        usage()
        exit()

    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.INFO)

    vision = smokeDetect(controller_ip, controller_port, update_frequency, source=source, verbose=use_verbose)
    vision.connect()
    if (vision.is_connected()):
        vision.run()

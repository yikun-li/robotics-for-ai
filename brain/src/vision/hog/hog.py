import cv2.cv as cv
import logging
import util.nullhandler
import numpy

logging.getLogger('Borg.Brain.Vision.hog.HoG').addHandler(util.nullhandler.NullHandler())

class HoG(object):
    '''
    In this function we calculate the histogram of oriented gradients of an image using opencv Calchist and manual histogram calculation
    '''
    def __init__(self, visualize = False, writing_method="cv"):
        self.logger = logging.getLogger("Borg.Brain.Vision.hog.HoG")
        self.visualize = visualize
        self.writing_method = writing_method
        
    def HoG(self, tangent, magnitude, writing_method = ""):
        
        if writing_method != "":
            self.writing_method = writing_method
            
        if self.image_check(tangent) < 0 or self.image_check(magnitude) < 0: 
            return -1
        size = cv.GetSize(tangent)
        
        
        if self.writing_method == "cv":
            #Create histograms using tangent function. Magnitude is only used as mask 
            #ranges = [(-181,-157,-112,-67,-22,22,67,112,157,181)]
            ranges= [(0,360)]
            
                        
            #ranges = [[0,22,67,112,157,202,247,292,337,360]]
            hist = cv.CreateHist([8], cv.CV_HIST_ARRAY, ranges,1)
            cv.CalcHist([tangent],hist,accumulate=0,mask=magnitude)
            
        elif self.writing_method == "numpy":
            #DONT USE THIS: WRONG IMPLEMENTATION ATM
            #Using magnitude to give ranks/value to tangent picture. Then, the histogram is created
            tangent_asarray = numpy.asarray(tangent[:,:])
            magnitude_asarray = numpy.asarray(magnitude[:,:])
            
            tangent_result = numpy.multiply(tangent_asarray, numpy.divide(magnitude_asarray,255))
             
            normalizedTangent = cv.CreateImageHeader((tangent_result.shape[1], tangent_result.shape[0]), cv.IPL_DEPTH_32F, 1)
            cv.SetData(normalizedTangent, tangent_result.tostring(), tangent_result.dtype.itemsize * 1 * tangent_result.shape[1])
            #ranges = [(-181,-157,-112,-67,-22,22,67,112,157,181)]
            ranges= [(0,360)]
            
                        
            #ranges = [[0,22,67,112,157,202,247,292,337,360]]
            hist = cv.CreateHist([8], cv.CV_HIST_ARRAY, ranges,1)
            cv.CalcHist([normalizedTangent],hist,accumulate=0,mask=magnitude)            
           
        else:
            #Very slow approach. Use numpy case instead
            hist = numpy.zeros(8)
            for x in range(size[0]):
                for y in range(size[1]):
                    if magnitude[y,x] > 0:
                        if tangent[y,x] < -157.5 or tangent[y,x] > 157.5:
                            hist[0] += 1 *magnitude[y,x]/255
                        elif tangent[y,x] < -112.5 and tangent[y,x] > -157.5 :
                            hist[7] += 1 *magnitude[y,x]/255
                        elif tangent[y,x] < -67.5 and tangent[y,x] > -112.5 :
                            hist[6] += 1 *magnitude[y,x]/255
                        elif tangent[y,x] < -22.5 and tangent[y,x] > -67.5:
                            hist[5] += 1 *magnitude[y,x]/255
                        elif tangent[y,x] < 22.5 and tangent[y,x] > -22.5:
                            hist[4] += 1 *magnitude[y,x]/255
                        elif tangent[y,x] < 67.5 and tangent[y,x] > 22.5:
                            hist[3] += 1 *magnitude[y,x]/255
                        elif tangent[y,x] < 112.5 and tangent[y,x] > 67.5:
                            hist[2] += 1 *magnitude[y,x]/255
                        elif tangent[y,x] < 157.5 and tangent[y,x] > 112.5:
                            hist[1] += 1 *magnitude[y,x]/255
                            
            #sum = numpy.sum(hist)
             
            #return numpy.divide(hist,sum)      
        if self.visualize:
            pass #draw the histogram
        
        return hist
    
        
    def image_check(self, image):
        try:
            nchannels = image.channels
            if nchannels <= 0:
                self.logger.error("The image has no channels")
                return -1
            if nchannels > 1:
                self.logger.warning("The image should be grayscale")
        except: 
            self.logger.error("The image is not an iplimage")  
            return -1
        return 0
    

'''
Created on Sep 21, 2011

@author: shantia
'''

import cv2.cv as cv
import logging
import util.nullhandler


logging.getLogger('Borg.Brain.Vision.hog.ContrastNormalizer').addHandler(util.nullhandler.NullHandler())

class ContrastNormalizer(object):
    '''
    This function simply checks whether an image is BW and then equalize the histogram of it.
    '''
    def __init__(self, visualize = False):
        self.logger = logging.getLogger("Borg.Brain.Vision.hog.ContrastNormalizer")
        self.visualize = visualize
    def normalize(self, image):
        
        #Checks whether inputs are correct
        if self.image_check(image) < 0: 
            return -1
        
        #chaning the image to grayscale
        
        gsimage = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 1)
        newgsimage = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 1)
        cv.CvtColor(image, gsimage, cv.CV_RGB2GRAY)
        cv.EqualizeHist(gsimage, newgsimage)
        
        if self.visualize:
            while True:
                cv.NamedWindow("Normal")
                cv.ShowImage("Normal", gsimage)
                cv.WaitKey(5)
                cv.NamedWindow("Histogram Equalized")
                cv.ShowImage("Histogram Equalized", newgsimage)
                if cv.WaitKey(5) == 1048603:
                    break
            cv.DestroyAllWindows()
        
        
        return newgsimage
        
    def image_check(self, image):
        try:
            nchannels = image.channels
            if nchannels <= 0:
                self.logger.error("The image has no channels")
                return -1
        except: 
            self.logger.error("The image is not an iplimage")  
            return -1
        return 0
    

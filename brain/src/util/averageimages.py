# To change this template, choose Tools | Templates
# and open the template in the editor.
__author__="Jorge Davila Chacon <jorgedch@gmail.com>"
__date__ ="$Dec 7, 2010 12:41:47 PM$"

import cv
import sys
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Util.AverageImages').addHandler(util.nullhandler.NullHandler())

class  AverageImages(object):

    def __init__(self):
        '''
        Constructor
        '''
        self.logger = logging.getLogger('Borg.Brain.Util.AverageImages')

    def get_images(self, cameranumber=0, source='camera', filename=None):
        if source == 'camera':
            CAM = cv.CaptureFromCAM(cameranumber)
            if str(CAM) == "<Capture (nil)>":
                return "Error! No camera Detected"
            else:
                return cv.QueryFrame(CAM)

        elif source == 'file':
            if filename == None:
                Error = "ERROR: Filename not specified."
                print Error
                return Error
            iscolor = cv.CV_LOAD_IMAGE_UNCHANGED
            IMG = cv.LoadImage(filename,iscolor)
            return IMG

    def get_average(self, imagesArray):
        bitDepth = imagesArray[0].depth
        if bitDepth > 8:
            print 'ERROR: Image bit depth too large. Adjust get_average method parameters.'
            sys.exit()
        
        imageSize = cv.GetSize(imagesArray[0])
        averageImage = cv.CreateImage(imageSize, cv.IPL_DEPTH_16U, 3)
        cv.Set(averageImage,0)
        for image in imagesArray:
            imageExpansion = cv.CreateImage(imageSize, cv.IPL_DEPTH_16U, 3)
            cv.Convert(image, imageExpansion)
            cv.Add(averageImage, imageExpansion, averageImage)

        nImages = 1/float(len(imagesArray))
        imageReduction = cv.CreateImage(imageSize, cv.IPL_DEPTH_8U, 3)
        cv.CvtScale(averageImage,imageReduction,nImages)

        return imageReduction

    def get_grayscale(self, image):
        image_gray = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 1)
        cv.CvtColor(image, image_gray, cv.CV_RGB2GRAY)

        return image_gray

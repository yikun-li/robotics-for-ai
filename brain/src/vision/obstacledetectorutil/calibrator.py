import sys
import os
import glob
import cv
import socket
import math

import calibrator

import util.averageimages
import util.kinectvideo as kv
#import util.openni_kinectvideo as kv

import segmentizer
from segmentizer import Segmentizer


class Calibrator:
    """ This class calibrates the obstacle detector.
        The class uses kinectoutput for calibration.
        During calculation, the data from the kinectouput is read and for a
        specified region (specified in segmentizer.py) the distance to the
        floor is set.
    """

    def __init__(self, source='kinect', visuals=False, filesource='default', caldestination='default'):

        # bit depth of the used depth image
        self.imageDepth = 8

        #current frame thats used for calibration
        self.currentFrame = 0

        #segmentizer, used for segmentating images into segments
        #able to create a matrix with the size of the future obstaclematrix
        self.segmentizer = Segmentizer()

        # create visualizer to show region and segment size
        # The visualizer is the same as the visualizer used to show obstacle avoidance
        # using the visualizer the region of interest for obstacle detection and the segment size can be changed
        if visuals == True:
            import visualisation.avoidancevisualizer
            self.visualizer = visualisation.avoidancevisualizer.AvoidanceVisualizer()
        
        # filesource is the folder for testfiles. Used for testing
        self.filesource = filesource

        # caldestination is the folder where calibration files will be saved
        # if this module is run on 'kinect-nav', the files will later automatically be send to the 'brain'
        if caldestination == 'default':
            self.caldestination = os.environ['BORG'] + "/brain/src/vision/obstacledetectorutil/"
        else:
            self.caldestination = caldestination

        # specify the source (file or kinect) where the images will be taken from
        # 'file' is used for testing
        # When 'kinect', the images will be taken directly from the Kinect
        self.source = source
        if self.source == "file":
            if self.filesource == 'default':
                self.path = os.environ['BORG'] + '/brain/data/od_images/ball_none/'
            else:
                self.path = self.filesource
            self.filelist = glob.glob( os.path.join(self.path, '*.png') )

    def sendFiles(self):
        """
            Send the settings for the region (of interest) and the segsize (segment size)
            from the 'kinect-nav' laptop to the 'brain' laptop
        """

        localfile = "region"
        remotehost = "borg@brain"
        remotefile = os.environ['BORG'] + "/brain/src/vision/obstacledetectorutil/region"
        os.system('scp "%s" "%s:%s"' % (localfile, remotehost, remotefile) )

        localfile = "segsize"
        remotehost = "borg@brain"
        remotefile = os.environ['BORG'] + "/brain/src/vision/obstacledetectorutil/segsize"
        os.system('scp "%s" "%s:%s"' % (localfile, remotehost, remotefile) )

        print "Files send to the brain"

    def visualCalibrate(self, frames = 30, test=0):
        """ use the visualize to adjust region of interest and the segment size """

        imageWidth = 640
        imageHeight = 480

        # get the current settings for the region of interest and the segment size        
        region = segmentizer.getRegion()
        segsize = segmentizer.getSegSize()

        while True:
            # create dummy matrix to show on the screen
            columns = int(math.ceil(region[2]/segsize[0]))
            rows = int(math.ceil(region[3]/segsize[1]))
            matrix = [[0 for col in range(columns)] for row in range(rows)]

            # set the background image of the visualizer
            if self.source == 'kinect':
                # get the rbg image from the kinect to show in the visualizer
                image = kv.GetRGB()
                self.visualizer.setRGBImage(image)
            else: #used for testing (instead of a rgm image, the depth image is used for background)
                image = self.getNextImage()
                if image == None:
                    self.currentFrame = 0
                else:
                    self.visualizer.setDepthImage(image)
                    self.currentFrame += 1
            
            # set the current segment size in the visualizer
            self.visualizer.setSegSize(segsize)
            # set the obstacle matrix to be shown in the visualizer
            self.visualizer.setObstacleMatrix(matrix)
            # set the region of interest in the visualizer
            self.visualizer.setRegionRect(region)
            # finally, draw to the screen.
            # the result can be a command (keytouch) performed by the user
            result = self.visualizer.draw()
            if result == 'exit': # exit the program
                sys.exit()
            elif result == 99: # c is pressed so calibrate using new settings
                # save new settings to files
                segmentizer.setRegion(region)
                segmentizer.setSegSize(segsize)
                # load the new settings from file to segmentizer
                self.segmentizer.loadSettings()

                # calibrate
                self.calibrate(frames=frames, test=test)

                # send files to the brain laptop (region and segsize, not the actual calibration data)
                if socket.gethostname() == 'kinect-nav':
                    self.sendFiles()

                sys.exit()

            else: # some key is pressed to manipulate the region of interest or the segment size
                l,t,w,h = region
                sw, sh = segsize
                if result == 119: # key w is pressed so region should go up
                    if t >= 5:
                        t -= 5
                elif result == 115: # key s is pressed so region should go down
                    if (t + h) <= (imageHeight - 5):
                        t += 5
                elif result == 101: # key e is pressed so region should be wider
                    if w < imageWidth:
                        w += 10
                        l -= 5
                elif result == 100: # key d is pressed so region should be narrower
                    if w >= 20:
                        w -= 10
                        l += 5
                elif result == 114: # key r is pressed so region should be taller
                    if (h + t) < (imageHeight - 5):
                        h += 5
                elif result == 102: # key f is pressed so region should be smaller
                    if h >= 10:
                        h -= 5
                elif result == 116: # key t is pressed so segsize should be larger
                        sw += 5
                        sh += 5
                elif result == 103: # key g is pressed so segsize should be smaller
                    if sw >= 10:
                        sw -= 5
                    if sh >= 10:
                        sh -= 5

                # new region and segsize
                region = (l,t,w,h)
                segsize = (sw, sh)

    def calibrate(self, frames = 30, test=0):
        """ calibrate the obstacle detector """
        """ if used with the kinect as source, make sure the kinect is pointed to a clear floor """

        print "start calibrating..."

        self.frames = int(frames)    # number of frames to calibrate for (30 is enough, for sure)

        if (self.frames <= 0):
            print "Error in calibrator.py: Please specify a legel amount of frames to calibrate for..."
            return

        # get 'self.frames' amount of images
        self.currentFrame = 0
        images = []
        while self.currentFrame < self.frames:

            if self.source == "file": # if files are used instead of the kinect (used for testing)
                # exit loop if all files in filelist have been processed
                if self.currentFrame >= len(self.filelist):
                    break

            print str(self.currentFrame + 1) + "/" + str(self.frames) # print current status
            sys.stdout.flush()
    
            # retieve images from file or kinect
            image = self.getNextImage()

            # if no more images are in the list stop the image retreival
            if image == None:
                break

            #add image to array
            images.append(image)

            self.currentFrame += 1

        #get min, max and average values of the images
        minImage = self.getMinValues(images)
        maxImage = self.getMaxValues(images)
        avgImage = self.getAverageValues(images)

        #conver the images to a segmented matrices
        minMatrix = self.segmentizer.getSegments(minImage)
        maxMatrix = self.segmentizer.getSegments(maxImage)
        avgMatrix = self.segmentizer.getSegments(avgImage)
        
        #write calibrated data (matrices) to file
        if test == 0:
            minFile = open(self.caldestination + "min.cal", "w")
            maxFile = open(self.caldestination + "max.cal", "w")
            avgFile = open(self.caldestination + "avg.cal", "w")
        else: # used for testing
            minFile = open(self.caldestination + "testmin.cal", "w")
            maxFile = open(self.caldestination + "testmax.cal", "w")
            avgFile = open(self.caldestination + "testavg.cal", "w")
        for row in range(len(avgMatrix)):
            minFile.write(str(minMatrix[row]) + "\n")
            maxFile.write(str(maxMatrix[row]) + "\n")
            avgFile.write(str(avgMatrix[row]) + "\n")
        minFile.close()
        maxFile.close()
        avgFile.close()

        print "calibration done!"

    def getNextImage(self):
        """returns a image which can be used for calibration"""

        #return kinect frame
        if self.source == "kinect":
            if self.imageDepth == 8:
                return kv.GetDepth8()
            elif self.imageDepth == 11 or self.imageDepth == 16:
                return kv.GetDepth11()
            else:
                print "Illegal image depth: '" + str(self.imageDepth) + "'. Using 8 bit"
                return kv.GetDepth8()

        # return testimage
        elif self.source == "file":
            # return None if there are no more images in the filelist
            if len(self.filelist) <= self.currentFrame:
                print "No more images in filelist..."
                return None
            else:
                return cv.LoadImage(self.filelist[self.currentFrame],cv.CV_LOAD_IMAGE_GRAYSCALE)

    def getMinValues(self, images):
        """ get min values over the images """

        minImage = cv.CloneImage(images[0])
        for image in images:
            cv.Min(minImage,image,minImage)

        return minImage

    def getMaxValues(self, images):
        """ get max values over all the images """

        maxImage = cv.CloneImage(images[0])
        for image in images:
            cv.Max(maxImage,image,maxImage)

        return maxImage

    def getAverageValues(self, images):
        """ get the average values over all the images
            adds them all together and then divides them with the numb. images
        """
        if len(images) == 0:
            return None
        if len(images) == 1:
            return images[0]

        imageSize = (images[0].width, images[0].height)

        sumImage = cv.CreateImage(imageSize, cv.IPL_DEPTH_32S, 1)

        cv.Set(sumImage,0)
        for image in images:
            tempImage = cv.CreateImage(imageSize, cv.IPL_DEPTH_32S, 1)
            cv.Convert(image, tempImage)
            cv.Add(sumImage, tempImage, sumImage)

        nImages = 1/float(len(images))
        meanImage = cv.CreateImage(imageSize, cv.IPL_DEPTH_8U, 1)
        cv.CvtScale(sumImage, meanImage, nImages)

        return meanImage


    # depricated
    def getAverageValues2(self, images):
        """ get the average values over all the images
            for every two images, divides the images by 2
            then adds them together
        """

        if len(images) == 0:
            return None
        if len(images) == 1:
            return images[0]

        width = images[0].width
        height = images[0].height
        # create image with only 2's
        # this will be used for division
        divisionImage = cv.CreateImage((width, height), cv.IPL_DEPTH_8U, 1)
        cv.Set(divisionImage, 2)
        image1 = cv.CreateImage((width, height), cv.IPL_DEPTH_8U, 1)
        image2 = cv.CreateImage((width, height), cv.IPL_DEPTH_8U, 1)

        avgImage = cv.CloneImage(images[0])
        for image in images:
            # divide images by 2
            cv.Div(avgImage, divisionImage, image1)
            cv.Div(image, divisionImage, image2)

            # add them to get result
            cv.Add(image1, image2 ,avgImage)

        return avgImage

if __name__ == "__main__":
    """ calibrate the floor """

    # calibrate
    if len(sys.argv) > 1:
        if sys.argv[1] == 'quick':
            # create calibrator
            cator = calibrator.Calibrator()
            cator.calibrate()
        else:
            print "USAGE:"
            print "Run calibrator.py without arguments to calibrate using visualization"
            print "Run calibrator.py with 'quick' as argument to quick calibrate"
    else:
        cator = calibrator.Calibrator(visuals=True, source='kinect')
        cator.visualCalibrate(frames=30, test=0)

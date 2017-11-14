import cv
import pickle
import os

def getRegion():
    """Read the region of interest where obstacles should be recognized from a file"""
    f = open(os.environ['BORG'] + "/brain/src/vision/obstacledetectorutil/region",'r')
    region = pickle.load(f) # (left,top,width,height)
    f.close()
    return region
def getSegSize():
    """Read the size of the segments from a file"""
    f = open(os.environ['BORG'] + "/brain/src/vision/obstacledetectorutil/segsize",'r')
    segSize = pickle.load(f) # (width, height)
    f.close()
    return segSize

def setRegion(region):
    """ Save the region of interest to a file """
    f = open(os.environ['BORG'] + "/brain/src/vision/obstacledetectorutil/region",'w')
    pickle.dump(region,f)
    f.close()

def setSegSize(segsize):
    """ Save the segment size to a file """
    f = open(os.environ['BORG'] + "/brain/src/vision/obstacledetectorutil/segsize",'w')
    pickle.dump(segsize,f)
    f.close()

class Segmentizer:
    """ This class segmentates an image into segments
        For every segment the average is calculated
        The result is a matrix containing one value per segment
    """

    def __init__(self):
        #load the region of interest and segment size settings
        self.loadSettings()
        self.segments = 0

    def loadSettings(self):
        """
            Loads the region of interest and segment size settings from file
        """
        self.segmentSize = getSegSize()
        self.regionOfInterest = getRegion()

    def setSegmentSize(self, segSize):
        """
            Set the size of the segments
        """
        self.segmentSize = segSize

    def setRegionOfInterest(self, region):
        """
            Set the region of interest to be used in the image
        """
        self.regionOfInterest = region

    def getSegments(self, image):
        """
        segmentize the region of interest into segments, each containing a mean value
        """

        left, top, width, height = self.regionOfInterest

        #create a new image containing just the distances in the region of interest
        imageDepth = image.depth
        imageChannels = image.nChannels
        regionDistances = cv.CreateImage( (width, height), imageDepth, imageChannels)
        src_region = cv.GetSubRect(image, (left, top, width, height) )
        cv.Copy(src_region, regionDistances)

        #segmentize the distances into segments
        segments = self.segmentize(regionDistances)

        return segments
    
    def segmentize(self, distances):
        """ segmentizes the image into segments
            for every segment the average pixelvalue is calculated.
        """

        # get the ranges (location and dimensions) of each segment.
        ranges = self.getSegmentRanges()

        # for every segment in the image calculate the average pixel value in that segment
        segments = []
        for y in range(len(ranges)):
            row = []
            for x in range(len(ranges[0])):
                average = self.getAverage(distances,ranges[y][x])
                row.append(average)
            segments.append(row)

        # reset image region of interest
        cv.ResetImageROI(distances)

        # return the segments. Each segment has 1 value (the average pixelf value in that segment location)
        return segments

    def getAverage(self, distances, segRange):
        """ calculate the average pixel value over a perticular range in the image """

        # get segment information
        segWidth, segHeight = self.segmentSize
        pixelsPerSegment = segWidth * segHeight

        # calculate average
        cv.SetImageROI(distances, segRange)

        average =  cv.Avg(distances)
        
        return average[0]

    def getSegmentRanges(self):
        """    get all ranges for every segment in the matrix
               That is, the x and y location of the specific segment and the dimensions of the specific segment
        """

        segWidth, segHeight = self.segmentSize
        regionWidth = self.regionOfInterest[2]
        regionHeight = self.regionOfInterest[3]

        segmentRanges = []
        for y in range(0, regionHeight, segHeight):
            row = []
            for x in range(0, regionWidth, segWidth):
                row.append((x, y, segWidth, segHeight))
            segmentRanges.append(row)

        return segmentRanges

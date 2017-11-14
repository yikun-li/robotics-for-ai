import unittest
import cv
import math
import os

from vision.obstacledetectorutil import segmentizer

class SegmentizerTestCase(unittest.TestCase):
    def setUp(self):
        self.image = cv.LoadImage(os.environ['BORG'] + "/brain/data/od_images/ball_left/depth103.png",cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.segmentizer = segmentizer.Segmentizer()

    def tearDown(self):
        self.segmentizer = None

    def test_getSegmentRanges(self):
        width = self.segmentizer.regionOfInterest[2] * 1.0
        height = self.segmentizer.regionOfInterest[3] * 1.0

        # test how many segments are generated with differend segmentsize settings
        segSize = (10,10)
        self.segmentizer.setSegmentSize(segSize)
        ranges = self.segmentizer.getSegmentRanges()
        # number of rows is the image height devided by segment height
        self.assertEqual(len(ranges), math.ceil(height/segSize[1]), "Segmentation problem. Incorrect number of rows")
        # number of items per row (columns) is the image width devided by the segment width
        self.assertEqual(len(ranges[0]), math.ceil(width/segSize[0]), "Segmentation problem. Incorrect number of columns")

        segSize = (15,20)
        self.segmentizer.setSegmentSize(segSize)
        ranges = self.segmentizer.getSegmentRanges()
        self.assertEqual(len(ranges), math.ceil(height/segSize[1]), "Segmentation problem. Incorrect number of rows")
        self.assertEqual(len(ranges[0]), math.ceil(width/segSize[0]), "Segmentation problem. Incorrect number of columns")

    def test_getAverage(self):
        ranges = self.segmentizer.getSegmentRanges()

        for y in range(len(ranges)):
            for x in range(len(ranges[0])):
                average = self.segmentizer.getAverage(self.image, ranges[y][x])
                self.assertTrue(average >= 0 and average <= 255)

    def test_segmentize(self):
        self.segmentizer.setSegmentSize(segmentizer.getSegSize())
        self.segmentizer.setRegionOfInterest(segmentizer.getRegion())
        segments = self.segmentizer.getSegments(self.image)

        # check if segments are set
        segSize = self.segmentizer.segmentSize
        region = self.segmentizer.regionOfInterest

        self.assertEqual(len(segments), math.ceil(float(region[3])/segSize[1]), "Segmentation problem. Incorrect number of rows")
        self.assertEqual(len(segments[0]), math.ceil(float(region[2])/segSize[0]), "Segmentation problem. Incorrect number of columns")

    def test_getSegments(self):
        segments = self.segmentizer.getSegments(self.image)
        # test if there are actually segments created
        self.assertTrue(len(segments) > 0, "No segments?")

    def test_SegSizeAndRegionHandlers(self):
        # get the segsize and the region from the files
        region = segmentizer.getRegion()
        segsize = segmentizer.getSegSize()

        # should be tuples of appropiate length
        self.assertEqual(len(region), 4, "region contains to much items (should be 4)")
        self.assertEqual(len(segsize), 2, "segsize contains to much items (should be 2)")

        # save the segsize and the region to the files
        segmentizer.setRegion(region)
        segmentizer.setSegSize(segsize)

        # check if they are saved correct
        savedRegion = segmentizer.getRegion()
        savedSegsize = segmentizer.getSegSize()

        self.assertEqual(region, savedRegion, "Region is not saved correct.")
        self.assertEqual(segsize, savedSegsize, "segsize is not saved correct.")


def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(SegmentizerTestCase))
    return suite

if __name__ == '__main__':
    unittest.main()

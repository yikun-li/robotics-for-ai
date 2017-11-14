import unittest
import os

from vision.obstacledetectorutil import calibrator

class CalibratorTestCase(unittest.TestCase):
    def setUp(self):
        self.calibrator = calibrator.Calibrator('file')

    def tearDown(self):
        self.calibrator = None

    def test_calibrate(self):
        # calibrate 5 frames, (the 1 indicates its a test, so the calibration data is written to a test file)
        self.calibrator.calibrate(5, 1)

        files = []
        files.append(open(os.environ['BORG'] + '/brain/src/vision/obstacledetectorutil/testmin.cal'))
        files.append(open(os.environ['BORG'] + '/brain/src/vision/obstacledetectorutil/testmax.cal'))
        files.append(open(os.environ['BORG'] + '/brain/src/vision/obstacledetectorutil/testavg.cal'))
        calibrationdataLists = []
        for file in files:
            calibrationdataList = []
            while 1:
                line = file.readline()
                if not line: break
                #convert line to list
                calibrationdataList.append(eval(line))
            file.close()
            calibrationdataLists.append(calibrationdataList)

        for list in calibrationdataLists:
            # list shouldn't be empty
            self.assertFalse(list == 0, "No data is written to calibration file")
            self.assertFalse(list == '', "No data is written to calibration file")
            self.assertFalse(list == None, "No data is written to calibration file")

            # list should contain values
            self.assertTrue(sum(list[0]) != 0, "First row of the calibrationdata contains only zeroes!?")

    def test_getNextImage(self):
        #allready used in test_calibrate() (it would have failed)
        pass

    def test_getMinValues(self):
        # receive some images
        images = []
        for i in range(5):
            image = self.calibrator.getNextImage()
            images.append(image)

        minMatrix = self.calibrator.getMinValues(images)

        self.assertFalse(minMatrix == 0, "minMatrix isn't a valid matrix")
        self.assertFalse(minMatrix == '', "minMatrix isn't a valid matrix")
        self.assertFalse(minMatrix == None, "minMatrix isn't a valid matrix")

    def test_visualCalibrate(self):
        # can't be done automatically (needs interaction)
        pass

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(CalibratorTestCase))
    return suite

if __name__ == '__main__':
    unittest.main()

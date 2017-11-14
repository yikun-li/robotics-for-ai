import unittest

import vision.obstacledetector
import vision.obstacledetectorutil.calibrator
import navigation.obstacleavoidance.obstacleavoider

class ObstacleAvoiderTestCase(unittest.TestCase):
    def setUp(self):
        self.obstacleDetector = vision.obstacledetector.ObstacleDetector(0,0,'file')
        self.calibrator = vision.obstacledetectorutil.calibrator.Calibrator(visuals=False, source='file')
        self.obstacleAvoider = navigation.obstacleavoidance.obstacleavoider.ObstacleAvoider(standalone=True)

    def tearDown(self):
        self.obstacleDetector = None
        self.calibrator = None
        self.obstacleAvoider = None


    def test_getScannedObstacleMatrix(self):
        #get obstacle matrix
        self.calibrator.calibrate(5, 1)
        matrix = self.obstacleDetector.getObstacleMatrix(3)
        obstacleMatrix = self.obstacleDetector.getObstacleMatrix(3)

        # get scanned matrix
        scannedMatrix = self.obstacleAvoider.getScannedObstacleMatrix(obstacleMatrix)

        self.assertEqual(len(scannedMatrix), len(obstacleMatrix), "Both matrices should have the same amount of rows")


def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(ObstacleAvoiderTestCase))
    return suite

if __name__ == '__main__':
    unittest.main()
# imports
import intervalestimation
import unittest

class  IntervalEstimationTestCase(unittest.TestCase):
    def setUp(self):
    	self.ie = intervalestimation.IntervalEstimation()

    def tearDown(self):
        self.ie = None
        
    def test_get_upperbound(self):
        """testing the upperbound of the interval estimation algorithm"""
        self.assertTrue( self.ie.get_upperbound( 98, [0,0,0] ) <= 1, "upperbound too high")
        
    def test_get_lowerbound(self):
        """testing the lowerbound of the interval estimation algorithm"""
        self.assertTrue( self.ie.get_lowerbound( 95, [0,0,0] ) >-1, "lowerbound too low")

    def test_get_mean(self):
        """testing the mean of the interval estimation algorithm"""
        self.assertTrue( self.ie.get_mean( [0,0,0] ) == 0, "incorrect mean")
        

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(IntervalEstimationTestCase))
    return suite

if __name__ == '__main__':
    unittest.main()



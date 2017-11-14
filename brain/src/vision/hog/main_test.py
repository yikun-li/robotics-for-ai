import unittest

import os
import main
import shutil

class Main_test(unittest.TestCase):
    '''
    The tests for the divider
    '''
        
    def setUp(self):
        path = (os.environ['BORG'] + '/Brain/data/hog_test/maintest/')
        
        self.main = main.Main(path)

        
    def tearDown(self):
        shutil.rmtree(os.environ['BORG'] + '/Brain/data/hog_test/maintest/hist')
    
    def test_Main(self): 
        result = self.main.extractor(4, 4)
        self.assertEqual(result, 6, "The main function didnt completely extract 6 picture IDs")
        
        result = self.main.extractor(4, 4, writing_method = "hist")
        self.assertEqual(result, 6, "The main function didnt completely extract 6 picture IDs")
        
    def test_cluster(self):
        _ = self.main.extractor(4, 4, writing_method = "cv")       
        result2 = self.main.cluster(nclusters = 2)
        
        self.assertEqual(result2, 0, "The main function didnt completely extract 6 picture IDs")
        
        _ = self.main.extractor(4, 4, writing_method = "hist")       
        result2 = self.main.cluster(nclusters = 2 , writing_method = "hist")
        self.assertEqual(result2, 0, "The main function didnt completely extract 6 picture IDs")
    
def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(Main_test))
    return suite

if __name__ == '__main__':
    unittest.main()  


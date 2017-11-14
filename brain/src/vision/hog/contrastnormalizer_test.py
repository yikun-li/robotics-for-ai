'''
Created on Feb 25, 2011

@author: shantia
'''
import unittest
import contrastnormalizer
import cv
import os

class Contrastnomarlizer_test(unittest.TestCase):
    '''
    The tests for the divider
    '''
        
    def setUp(self):
        self.equilizer = contrastnormalizer.ContrastNormalizer(False)

    def tearDown(self):
        pass
    
    def test_empty_Image(self):
        result = self.equilizer.normalize(None)
        self.assertEqual(result, -1, "The divider could not give error on empty image")
        
    def test_contrast_Image(self): 
        image = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/noface.jpg'))
        result = self.equilizer.normalize(image)
        self.assertEqual(self.equilizer.image_check(result),0,"The normalizer function didn't return anything")
    
def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(Contrastnomarlizer_test))
    return suite

if __name__ == '__main__':
    unittest.main()  


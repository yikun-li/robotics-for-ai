'''
Created on Feb 25, 2011

@author: shantia
'''
import unittest
import os
import divider
import cv2.cv as cv

class Divider_test(unittest.TestCase):
    '''
    The tests for the divider
    '''
        
    def setUp(self):
        self.divider = divider.Divider(True)

    def tearDown(self):
        pass
    
    def test_empty_Image(self):
        result = self.divider.divide(None)
        self.assertEqual(result, -1, "The divider could not give error on empty image")
    
    def test_wrong_mn(self):
        image = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/test_img/faces.jpg'))
        result = self.divider.divide(image, 0, 0)
        self.assertEqual(result, -2, "The divider could not give error on wrong width and height")
            
    def test_divider(self):
        image = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/noface.jpg'))
        subRect = (0,0, 250,187)
        subimage = self.divider.crop(image, subRect)
        subimage1 = image
        
        result = self.divider.divide(image, 2,2)
        
        if cv.GetSize(subimage) != cv.GetSize(result[0]):
            self.fail("The subimage sizes are not correct. Either correctly crop the image manually or check divider function")
            
        dif = cv.CreateImage(cv.GetSize(subimage), cv.IPL_DEPTH_8U, 3)
        dif2 = cv.CreateImage(cv.GetSize(subimage), cv.IPL_DEPTH_8U, 3)
        
        
        cv.AbsDiff(subimage, result[0], dif)
        cv.Threshold(dif, dif2, 50,255, cv.CV_THRESH_TOZERO)
        for i in range(3):
            cv.SetImageCOI(dif2, i+1)
            n_nonzero = cv.CountNonZero(dif2)


        if n_nonzero < 400:
            threshold = 0
        else:
            threshold = 1
            self.assertEqual(threshold,0, "The subimages are different")
         
        result = self.divider.divide(image, 4,4, option = "pro")
        print len(result)
        print result
        
        dif = cv.CreateImage(cv.GetSize(subimage1), cv.IPL_DEPTH_8U, 3)
        dif2 = cv.CreateImage(cv.GetSize(subimage1), cv.IPL_DEPTH_8U, 3)
           
        cv.AbsDiff(subimage1, result[0], dif)
        cv.Threshold(dif, dif2, 50,255, cv.CV_THRESH_TOZERO)
        for i in range(3):
            cv.SetImageCOI(dif2, i+1)
            n_nonzero = cv.CountNonZero(dif2)


        if n_nonzero < 400:
            threshold = 0
        else:
            threshold = 1
            self.assertEqual(threshold,0, "The subimages are different")
        
        
        result = self.divider.divide(image, 4,4, option = "pro", overlap = 10)    
    
def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(Divider_test))
    return suite

if __name__ == '__main__':
    unittest.main()  


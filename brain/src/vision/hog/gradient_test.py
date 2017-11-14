'''
Created on Feb 25, 2011

@author: shantia
'''
import unittest
import gradient
import os
import cv2.cv as cv
import math

class Gradient_test(unittest.TestCase):
    '''
    The tests for gradient calclulator
    '''
        
    def setUp(self):
        self.gradient = gradient.Gradient(True)
        self.constant = 180.0/math.pi
    def tearDown(self):
        pass
    
    def test_empty_Image(self): 
        result = self.gradient.cannyGradient(None)
        self.assertEqual(result, -1, "The divider could not give error on empty image")
        
    def test_canny_Image(self): 
        image = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/matrixtest/1320757889.jpg'))
        result = self.gradient.cannyGradient(image)
        
        #size = cv.GetSize(result)
        #for x in range(size[0]):
        #    for y in range(size[1]):
        #        print result[y,x]
        self.assertEqual(self.gradient.image_check(result),0,"The Edge image is not correct")
        
    def test_sobel_Image(self): 
        image = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/matrixtest/1320757889.jpg'))
        result = self.gradient.sobelGradient(image)
        success = 0
        if self.gradient.image_check(result[0]) == 0 and self.gradient.image_check(result[1]) == 0:
            success = 0
        else: 
            success = 1
        self.assertEqual(success,0,"The sobel function didn't return anything")
    
    def test_tangentCanny(self):
        image = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/matrixtest/1320757889.jpg'))
        result = self.gradient.cannyTangent(image)
        success = 0
        if self.gradient.image_check(result[0]) == 0 and self.gradient.image_check(result[1]) == 0:
            success = 0
        else: 
            success = 1
        self.assertEqual(success,0,"The tangentCanny is not working properly")
        
    def test_tangent_vert(self):
        vert = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/vertical edge.jpg'))
        
        image = vert
        (dx, dy) = self.gradient.sobelGradient(image)
        canny = self.gradient.cannyGradient(image)
        tangent = self.gradient.tangent(dx, dy,canny)
        
        size = cv.GetSize(tangent)
        
        for x in range(size[0]):
            for y in range(size[1]):
                if canny[y,x] > 0:
                    tang = math.atan2(dy[y,x], dx[y,x])*self.constant
                    
                        
                    tang = int(tang)
                    
                    if tang < 0 :
                        tang = 360 + tang
                    '''if dx[y,x] < 0 and dy[y,x] >= 0:
                        tang += 180
                    elif dx[y,x] < 0 and dy[y,x] < 0:
                        tang -= 180'''
                else:
                    tang = 0
                if tangent[y,x] != tang:      
                    print 'DX ->column: ', x, ' ,Row: ', y, ' ,Value: ', dx[y,x]
                    print 'DY ->column: ', x, ' ,Row: ', y, ' ,Value: ', dy[y,x]
                    print 'Real tangent:', tang 
                    print 'The tangent from function', tangent[y,x]
                    self.fail("The tangents do not match")
                if tangent[y,x] != 0:
                    self.fail("Vertical edge should not give anything than 0 az tangent")
                    
    def test_tangent_horiz(self):
        horiz = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/horizontal edge.jpg'))      
        image = horiz
        (dx, dy) = self.gradient.sobelGradient(image)
        canny = self.gradient.cannyGradient(image)
        tangent = self.gradient.tangent(dx, dy, canny)
        
        size = cv.GetSize(tangent)
        
        for x in range(size[0]):
            for y in range(size[1]):
                if canny[y,x] > 0:
                    tang = math.atan2(dy[y,x], dx[y,x])*self.constant
                    tang = int(tang)
                    if tang < 0 :
                        tang = 360 + tang
                    '''if dx[y,x] < 0 and dy[y,x] >= 0:
                        tang += 180
                    elif dx[y,x] < 0 and dy[y,x] < 0:
                        tang -= 180'''
                else:
                    tang = 0
                if tangent[y,x] != tang:      
                    print 'DX ->column: ', x, ' ,Row: ', y, ' ,Value: ', dx[y,x]
                    print 'DY ->column: ', x, ' ,Row: ', y, ' ,Value: ', dy[y,x]
                    print 'Real tangent:', tang 
                    print 'The tangent from function', tangent[y,x]
                    self.fail("The tangents do not match")
                    if not (tangent[y,x] == 90 or tangent[y,x] ==-90):
                        self.fail("Horizontal edge should only give either 90 or -90 as gradient vector")
                        
    def test_tangent_45plusdegree(self):
        degree45 = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/45degree.jpg'))               
           
        
        smoothed = cv.CreateImage(cv.GetSize(degree45), degree45.depth, degree45.channels)
        cv.Smooth(degree45, smoothed)
        image = smoothed
        (dx, dy) = self.gradient.sobelGradient(image)
        canny = self.gradient.cannyGradient(image)
        tangent = self.gradient.tangent(dx, dy, canny)
        
        size = cv.GetSize(tangent)
        
        for x in range(size[0]):
            for y in range(size[1]):
                if canny[y,x] > 0:
                    tang = math.atan2(dy[y,x], dx[y,x])*self.constant
                    tang = int(tang)
                    if tang < 0 :
                        tang = 360 + tang
                    '''if dx[y,x] < 0 and dy[y,x] >= 0:
                        tang += 180
                    elif dx[y,x] < 0 and dy[y,x] < 0:
                        tang -= 180'''
                else:
                    tang = 0
                if tangent[y,x] != tang:      
                    print 'DX ->column: ', x, ' ,Row: ', y, ' ,Value: ', dx[y,x]
                    print 'DY ->column: ', x, ' ,Row: ', y, ' ,Value: ', dy[y,x]
                    print 'Real tangent:', tang 
                    print 'The tangent from function', tangent[y,x]
                    self.fail("The tangents do not match")
                
                
                if canny[y,x] > 0 and not ( (tangent[y,x] >= 180 and tangent[y,x] <= 270 ) ):
                    print 'The tangent from function', tangent[y,x], "calculated here is:", tang
                    self.fail("The tangent is not in the range of 225 and  270. (45 degrees)")

        
def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(Gradient_test))
    return suite

if __name__ == '__main__':
    unittest.main()  


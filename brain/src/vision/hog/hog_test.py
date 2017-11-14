'''
Created on Feb 25, 2011

@author: shantia
'''
import unittest
import hog
import os
import cv2.cv as cv
import gradient

class HoG_test(unittest.TestCase):
    '''
    The tests for the divider
    '''
    
    def setUp(self):
        self.hog = hog.HoG(False)
        self.gradient = gradient.Gradient(False)
        
    def tearDown(self):
        pass
    
    def test_empty_Image(self):
        result = self.hog.HoG(None, None)
        self.assertEqual(result, -1, "The divider could not give error on empty image")
    def test_histogram_numpy(self):
        
        tests = ["numpy"]
        vertical = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/vertical edge.jpg'))
        verticalr = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/vertical edge-r.jpg'))
        
        horizontal = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/horizontal edge.jpg'))
        horizontalr = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/horizontal edge-r.jpg'))
        
        degree45 = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/45degree.jpg'))
        degree135 = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/135degree.jpg'))
        degree225 = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/225degree.jpg'))
        degree315 = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/315degree.jpg'))
        
        #ranges = [[0,22,67,112,157,202,247,292,337,359]]
        #hist = cv.CreateHist([9], cv.CV_HIST_ARRAY, ranges,uniform = 0)
        #ranges = [(0,360)]
        #hist = cv.CreateHist(dims = [8], type = cv.CV_HIST_ARRAY, ranges,uniform = 1)
        
        print "Thorough Histogram Test"
        for type in tests:
            
        
            
            (tangent, magnitude) = self.gradient.cannyTangent(vertical,smooth = False, T1 = 20, T2 = 250, eq = False)
            vert = self.hog.HoG(tangent, magnitude, type)
            
            
            result = 1
            if cv.QueryHistValue_1D(vert,0)  > 0:
                result = 0
            
            print "Test Image: Vertical Edge"   
            print "Horizontal edge: ", cv.QueryHistValue_1D(vert,2), cv.QueryHistValue_1D(vert,6)
            print "vertical edge: ", cv.QueryHistValue_1D(vert,0), cv.QueryHistValue_1D(vert,4)
            print "+45 degree: ", cv.QueryHistValue_1D(vert,1),cv.QueryHistValue_1D(vert,5)
            print "-45 degree: ", cv.QueryHistValue_1D(vert,3), cv.QueryHistValue_1D(vert,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for vertical image')
            cv.ClearHist(vert)
            vert=None
            
            (tangent, magnitude) = self.gradient.cannyTangent(verticalr,smooth = False, T1 = 20, T2 = 250, eq = False)
            vert = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(vert,4) > 0 :
                result = 0
            
            print "Test Image: Vertical Edge"   
            print "Horizontal edge: ", cv.QueryHistValue_1D(vert,2), cv.QueryHistValue_1D(vert,6)
            print "vertical edge: ", cv.QueryHistValue_1D(vert,0), cv.QueryHistValue_1D(vert,4)
            print "+45 degree: ", cv.QueryHistValue_1D(vert,1),cv.QueryHistValue_1D(vert,5)
            print "-45 degree: ", cv.QueryHistValue_1D(vert,3), cv.QueryHistValue_1D(vert,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for vertical image')
            cv.ClearHist(vert)
            
            
            (tangent, magnitude) = self.gradient.cannyTangent(horizontal,smooth = False, T1 = 20, T2 = 250, eq = False)
            horiz = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(horiz,6) > 0:
                result = 0
            
            print "Test Image: Horizontal Edge:"  
            print "Horizontal edge: ", cv.QueryHistValue_1D(horiz,2), cv.QueryHistValue_1D(horiz,6)
            print "vertical edge: ", cv.QueryHistValue_1D(horiz,0), cv.QueryHistValue_1D(horiz,4)
            print "+45 degree: ", cv.QueryHistValue_1D(horiz,1),cv.QueryHistValue_1D(horiz,5)
            print "-45 degree: ", cv.QueryHistValue_1D(horiz,3), cv.QueryHistValue_1D(horiz,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for horizontal image')
            cv.ClearHist(horiz)
            
            (tangent, magnitude) = self.gradient.cannyTangent(horizontalr,smooth = False, T1 = 20, T2 = 250, eq = False)
            horiz = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(horiz,2) > 0:
                result = 0
            print "Test Image: Horizontal Edge:"  
            print "Horizontal edge: ", cv.QueryHistValue_1D(horiz,2), cv.QueryHistValue_1D(horiz,6)
            print "vertical edge: ", cv.QueryHistValue_1D(horiz,0), cv.QueryHistValue_1D(horiz,4)
            print "+45 degree: ", cv.QueryHistValue_1D(horiz,1),cv.QueryHistValue_1D(horiz,5)
            print "-45 degree: ", cv.QueryHistValue_1D(horiz,3), cv.QueryHistValue_1D(horiz,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for horizontal image')
            
            (tangent, magnitude) = self.gradient.cannyTangent(degree45,smooth = True, T1 = 20, T2 = 250, eq = False)
            deg45 = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(deg45,5) > 0:
                result = 0
            print "Test Image: 45 Degree:"      
            print "Horizontal edge: ", cv.QueryHistValue_1D(deg45,2), cv.QueryHistValue_1D(deg45,6)
            print "vertical edge: ", cv.QueryHistValue_1D(deg45,0), cv.QueryHistValue_1D(deg45,4)
            print "+45 degree: ", cv.QueryHistValue_1D(deg45,1),cv.QueryHistValue_1D(deg45,5)
            print "-45 degree: ", cv.QueryHistValue_1D(deg45,3), cv.QueryHistValue_1D(deg45,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for 45 degree image')
            
            (tangent, magnitude) = self.gradient.cannyTangent(degree225,smooth = True, T1 = 20, T2 = 250, eq = False)
            deg45 = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(deg45,1) > 0:
                result = 0
            print "Test Image: 45degree Edge:"
            print "Horizontal edge: ", cv.QueryHistValue_1D(deg45,2), cv.QueryHistValue_1D(deg45,6)
            print "vertical edge: ", cv.QueryHistValue_1D(deg45,0), cv.QueryHistValue_1D(deg45,4)
            print "+45 degree: ", cv.QueryHistValue_1D(deg45,1),cv.QueryHistValue_1D(deg45,5)
            print "-45 degree: ", cv.QueryHistValue_1D(deg45,3), cv.QueryHistValue_1D(deg45,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for 45 degree image')
            
            (tangent, magnitude) = self.gradient.cannyTangent(degree135,smooth = True, T1 = 20, T2 = 250, eq = False)
            deg45 = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(deg45,7) > 0:
                result = 0
            print "Test Image: 45 Degree:"  
            print "Horizontal edge: ", cv.QueryHistValue_1D(deg45,2), cv.QueryHistValue_1D(deg45,6)
            print "vertical edge: ", cv.QueryHistValue_1D(deg45,0), cv.QueryHistValue_1D(deg45,4)
            print "+45 degree: ", cv.QueryHistValue_1D(deg45,1),cv.QueryHistValue_1D(deg45,5)
            print "-45 degree: ", cv.QueryHistValue_1D(deg45,3), cv.QueryHistValue_1D(deg45,7)
            
            (tangent, magnitude) = self.gradient.cannyTangent(degree315,smooth = True, T1 = 20, T2 = 250, eq = False)
            deg45 = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(deg45,3) > 0:
                result = 0
            print "Test Image: 45 Degree:"  
            print "Horizontal edge: ", cv.QueryHistValue_1D(deg45,2), cv.QueryHistValue_1D(deg45,6)
            print "vertical edge: ", cv.QueryHistValue_1D(deg45,0), cv.QueryHistValue_1D(deg45,4)
            print "+45 degree: ", cv.QueryHistValue_1D(deg45,1),cv.QueryHistValue_1D(deg45,5)
            print "-45 degree: ", cv.QueryHistValue_1D(deg45,3), cv.QueryHistValue_1D(deg45,7)      
    def test_histogram_cv(self):
        
        tests = ["cv"]
        vertical = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/vertical edge.jpg'))
        verticalr = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/vertical edge-r.jpg'))
        
        horizontal = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/horizontal edge.jpg'))
        horizontalr = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/horizontal edge-r.jpg'))
        
        degree45 = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/45degree.jpg'))
        degree135 = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/135degree.jpg'))
        degree225 = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/225degree.jpg'))
        degree315 = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/315degree.jpg'))
        
        #ranges = [[0,22,67,112,157,202,247,292,337,359]]
        #hist = cv.CreateHist([9], cv.CV_HIST_ARRAY, ranges,uniform = 0)
        #ranges = [(0,360)]
        #hist = cv.CreateHist(dims = [8], type = cv.CV_HIST_ARRAY, ranges,uniform = 1)
        
        print "Thorough Histogram Test"
        for type in tests:
            
        
            
            (tangent, magnitude) = self.gradient.cannyTangent(vertical,smooth = False, T1 = 20, T2 = 250, eq = False)
            vert = self.hog.HoG(tangent, magnitude, type)
            
            
            result = 1
            if cv.QueryHistValue_1D(vert,0)  > 0:
                result = 0
            
            print "Test Image: Vertical Edge"   
            print "Horizontal edge: ", cv.QueryHistValue_1D(vert,2), cv.QueryHistValue_1D(vert,6)
            print "vertical edge: ", cv.QueryHistValue_1D(vert,0), cv.QueryHistValue_1D(vert,4)
            print "+45 degree: ", cv.QueryHistValue_1D(vert,1),cv.QueryHistValue_1D(vert,5)
            print "-45 degree: ", cv.QueryHistValue_1D(vert,3), cv.QueryHistValue_1D(vert,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for vertical image')
            cv.ClearHist(vert)
            vert=None
            
            (tangent, magnitude) = self.gradient.cannyTangent(verticalr,smooth = False, T1 = 20, T2 = 250, eq = False)
            vert = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(vert,4) > 0 :
                result = 0
            
            print "Test Image: Vertical Edge"   
            print "Horizontal edge: ", cv.QueryHistValue_1D(vert,2), cv.QueryHistValue_1D(vert,6)
            print "vertical edge: ", cv.QueryHistValue_1D(vert,0), cv.QueryHistValue_1D(vert,4)
            print "+45 degree: ", cv.QueryHistValue_1D(vert,1),cv.QueryHistValue_1D(vert,5)
            print "-45 degree: ", cv.QueryHistValue_1D(vert,3), cv.QueryHistValue_1D(vert,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for vertical image')
            cv.ClearHist(vert)
            
            
            (tangent, magnitude) = self.gradient.cannyTangent(horizontal,smooth = False, T1 = 20, T2 = 250, eq = False)
            horiz = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(horiz,6) > 0:
                result = 0
            
            print "Test Image: Horizontal Edge:"  
            print "Horizontal edge: ", cv.QueryHistValue_1D(horiz,2), cv.QueryHistValue_1D(horiz,6)
            print "vertical edge: ", cv.QueryHistValue_1D(horiz,0), cv.QueryHistValue_1D(horiz,4)
            print "+45 degree: ", cv.QueryHistValue_1D(horiz,1),cv.QueryHistValue_1D(horiz,5)
            print "-45 degree: ", cv.QueryHistValue_1D(horiz,3), cv.QueryHistValue_1D(horiz,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for horizontal image')
            cv.ClearHist(horiz)
            
            (tangent, magnitude) = self.gradient.cannyTangent(horizontalr,smooth = False, T1 = 20, T2 = 250, eq = False)
            horiz = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(horiz,2) > 0:
                result = 0
            print "Test Image: Horizontal Edge:"  
            print "Horizontal edge: ", cv.QueryHistValue_1D(horiz,2), cv.QueryHistValue_1D(horiz,6)
            print "vertical edge: ", cv.QueryHistValue_1D(horiz,0), cv.QueryHistValue_1D(horiz,4)
            print "+45 degree: ", cv.QueryHistValue_1D(horiz,1),cv.QueryHistValue_1D(horiz,5)
            print "-45 degree: ", cv.QueryHistValue_1D(horiz,3), cv.QueryHistValue_1D(horiz,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for horizontal image')
            
            (tangent, magnitude) = self.gradient.cannyTangent(degree45,smooth = True, T1 = 20, T2 = 250, eq = False)
            deg45 = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(deg45,5) > 0:
                result = 0
            print "Test Image: 45 Degree:"      
            print "Horizontal edge: ", cv.QueryHistValue_1D(deg45,2), cv.QueryHistValue_1D(deg45,6)
            print "vertical edge: ", cv.QueryHistValue_1D(deg45,0), cv.QueryHistValue_1D(deg45,4)
            print "+45 degree: ", cv.QueryHistValue_1D(deg45,1),cv.QueryHistValue_1D(deg45,5)
            print "-45 degree: ", cv.QueryHistValue_1D(deg45,3), cv.QueryHistValue_1D(deg45,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for 45 degree image')
            
            (tangent, magnitude) = self.gradient.cannyTangent(degree225,smooth = True, T1 = 20, T2 = 250, eq = False)
            deg45 = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(deg45,1) > 0:
                result = 0
            print "Test Image: 45degree Edge:"
            print "Horizontal edge: ", cv.QueryHistValue_1D(deg45,2), cv.QueryHistValue_1D(deg45,6)
            print "vertical edge: ", cv.QueryHistValue_1D(deg45,0), cv.QueryHistValue_1D(deg45,4)
            print "+45 degree: ", cv.QueryHistValue_1D(deg45,1),cv.QueryHistValue_1D(deg45,5)
            print "-45 degree: ", cv.QueryHistValue_1D(deg45,3), cv.QueryHistValue_1D(deg45,7)
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for 45 degree image')
            
            (tangent, magnitude) = self.gradient.cannyTangent(degree135,smooth = True, T1 = 20, T2 = 250, eq = False)
            deg45 = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(deg45,7) > 0:
                result = 0
            print "Test Image: 45 Degree:"  
            print "Horizontal edge: ", cv.QueryHistValue_1D(deg45,2), cv.QueryHistValue_1D(deg45,6)
            print "vertical edge: ", cv.QueryHistValue_1D(deg45,0), cv.QueryHistValue_1D(deg45,4)
            print "+45 degree: ", cv.QueryHistValue_1D(deg45,1),cv.QueryHistValue_1D(deg45,5)
            print "-45 degree: ", cv.QueryHistValue_1D(deg45,3), cv.QueryHistValue_1D(deg45,7)
            
            (tangent, magnitude) = self.gradient.cannyTangent(degree315,smooth = True, T1 = 20, T2 = 250, eq = False)
            deg45 = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if cv.QueryHistValue_1D(deg45,3) > 0:
                result = 0
            print "Test Image: 45 Degree:"  
            print "Horizontal edge: ", cv.QueryHistValue_1D(deg45,2), cv.QueryHistValue_1D(deg45,6)
            print "vertical edge: ", cv.QueryHistValue_1D(deg45,0), cv.QueryHistValue_1D(deg45,4)
            print "+45 degree: ", cv.QueryHistValue_1D(deg45,1),cv.QueryHistValue_1D(deg45,5)
            print "-45 degree: ", cv.QueryHistValue_1D(deg45,3), cv.QueryHistValue_1D(deg45,7)
            
    def test_histogram_hist(self):
        tests = ["hist"]
        vertical = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/vertical edge.jpg'))
        horizontal = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/horizontal edge.jpg'))
        degree45 = cv.LoadImage(os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/45degree.jpg'))
        for type in tests:
            
            
            (tangent, magnitude) = self.gradient.cannyTangent(vertical,smooth = True, T1 = 20, T2 = 250, eq = False, method="fast")
            vert = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if vert[0] > 0 or vert[4] > 0:
                result = 0
                
            print "Horizontal edge: ", vert[2], vert[6]
            print "vertical edge: ", vert[0], vert[4]
            print "+45 degree: ", vert[1], vert[5]
            print "-45 degree: ", vert[3], vert[7]
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for vertical image')
            
            (tangent, magnitude) = self.gradient.cannyTangent(horizontal,smooth = True, T1 = 20, T2 = 250, eq = False, method="fast")
            horiz = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if horiz[2] > 0 or horiz[6] > 0:
                result = 0
            print "Horizontal edge: ", horiz[2], horiz[6]
            print "vertical edge: ", horiz[0], horiz[4]
            print "+45 degree: ", horiz[1], horiz[5]
            print "-45 degree: ", horiz[3], horiz[7]
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for horizontal image')
            
            (tangent, magnitude) = self.gradient.cannyTangent(degree45,smooth = True, T1 = 20, T2 = 250, eq = False, method="fast")
            deg45 = self.hog.HoG(tangent, magnitude, type)
            
            result = 1
            if deg45[3] > 0 or deg45[7] > 0:
                result = 0
            print "Horizontal edge: ", deg45[2], deg45[6]
            print "vertical edge: ", deg45[0], deg45[4]
            print "+45 degree: ", deg45[1], deg45[5]
            print "-45 degree: ", deg45[3], deg45[7]
            self.assertEqual(result,0, 'The histogram couldnt correctly calculate the orientation for 45 degree image')
    
def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(HoG_test))
    return suite

if __name__ == '__main__':
    unittest.main()  


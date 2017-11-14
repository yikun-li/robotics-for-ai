'''
Created on Sep 21, 2011

@author: shantia
'''
import cv2.cv as cv
import math
import logging
import util.nullhandler
import numpy
import time
logging.getLogger('Borg.Brain.Vision.hog.Gradient').addHandler(util.nullhandler.NullHandler())

class Gradient(object):
    def __init__(self, visualize = False):
        self.logger = logging.getLogger("Borg.Brain.Vision.hog.Gradient")
        self.visualize = visualize
        self.constant = 180.0/math.pi

    def cannyGradient(self, image, t1 = 20, t2 = 250):
        '''Returns the canny gradient'''
        #Checks whether inputs are correct
        if self.image_check(image) < 0: 
            return -1
        
        #Converts the image if it is not B&W
        gsimage = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 1)  
        if image.channels > 1 :
            temp = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 1)
            cv.CvtColor(image, temp, cv.CV_BGR2GRAY)      
            gsimage = temp
        else:
            gsimage = image
            
        
        
        
        #Gets the edges from the image
        edges = cv.CreateImage(cv.GetSize(gsimage), cv.IPL_DEPTH_8U, 1)
        
        #Warning: the threshold 1 and threshold 2 should be selected by experiment
        cv.Canny(gsimage, edges, threshold1 = t1, threshold2 = t2)
        
        if self.visualize:
            while True:
                cv.NamedWindow("Original")
                cv.ShowImage("Original", gsimage)
                cv.NamedWindow("Edges")
                cv.ShowImage("Edges", edges)
                c = cv.WaitKey(5)
                if c > 0:
                    
                    break
        cv.DestroyAllWindows()    
        return edges
    
    def sobelGradient(self, image):
        '''Calculates the gradient in x and y direction using Sobel mask using default 3x3 filter'''
        if self.image_check(image) < 0: 
            return -1
        
        gsimage = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 1)  
        if image.channels > 1 :
            temp = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 1)
            cv.CvtColor(image, temp, cv.CV_BGR2GRAY)      
            gsimage = temp
        else:
            gsimage = image
        #smoothing image 
        
        #Creating empty images for dx,dy and temporary 16 bit dx16 and dy16
        dy16 =  cv.CreateImage(cv.GetSize(gsimage), cv.IPL_DEPTH_16S, 1)
        dx16 =  cv.CreateImage(cv.GetSize(gsimage), cv.IPL_DEPTH_16S, 1)
        dx =  cv.CreateImage(cv.GetSize(gsimage), cv.IPL_DEPTH_8U, 1)
        dy =  cv.CreateImage(cv.GetSize(gsimage), cv.IPL_DEPTH_8U, 1)
        
        #Convolving sobel mask to get gradient of dx and dy
        cv.Sobel(gsimage, dy16, 0, 1,apertureSize = 3)
        cv.Sobel(gsimage, dx16, 1, 0,apertureSize = 3)
        cv.ConvertScaleAbs(dx16, dx)
        cv.ConvertScaleAbs(dy16, dy)  
                
        if self.visualize:
            while True:
                cv.NamedWindow("Original")
                cv.ShowImage("Original", gsimage)
                cv.NamedWindow("Sobelx")
                cv.ShowImage("Sobelx", dx)
                cv.NamedWindow("Sobely")
                cv.ShowImage("Sobely", dy)
                c = cv.WaitKey(5)
                if c > 0:
                    
                    break
        cv.DestroyAllWindows()
        
        return (dx16, dy16)
    
    def smooth(self, image, param1 = 5, param2 = 16):
        
        smoothed = cv.CreateImage(cv.GetSize(image), image.depth, image.channels)
        cv.Smooth(image, smoothed, smoothtype=cv.CV_GAUSSIAN_5x5, param1 = 5, param2 = 16)
        
        return smoothed
    
    def tangent(self, dx,dy, Mask = None, method="cv"):
        '''This function calculates the gradient orientation of each pixel 
        that is in Mask'''
        
        tangent = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_8U, 1)
        divSize = cv.GetSize(dx)
        tangent16U = cv.CreateImage(divSize, cv.IPL_DEPTH_32F, 1)
        cv.SetZero(tangent16U)
        if method == "slow":
            for x in range(divSize[0]):
                for y in range(divSize[1]):
                    if Mask == None:

                        tang = math.atan2(dy[y,x], dx[y,x])*self.constant
                        tangent16U[y,x] = int(tang)
                        
                    elif Mask[y,x] > 0:
                        tang = math.atan2(dy[y,x], dx[y,x])*self.constant
                        tangent16U[y,x] = int(tang)
                    elif Mask[y,x] == 0:
                        tangent16U[y,x] = 0
        elif method == "cv":
            #Calculated the arctan2 which give -pi to pi range
            #I create numpy arrays then reshape them in to 1 Dimesnion.
            #Next, I calculated arctan2 and change the range to 0-2pi and make it in degrees
            #I reshape back to picture format and return the picture
            
            #Numpy formatting
            (width,height) = cv.GetSize(dx)
            matdx = cv.CreateMat(height,width,cv.CV_16SC1)
            matdy = cv.CreateMat(height,width,cv.CV_16SC1)
            cv.SetZero(matdx)
            cv.SetZero(matdy)
            cv.Copy(dx,matdx, Mask)
            
            cv.Copy(dy,matdy, Mask)
            a = numpy.asarray(matdx)
            b = numpy.asarray(matdy)
            
            #Reshaping to one dimension
            ar = numpy.reshape(a, a.size)
            br = numpy.reshape(b, b.size)
            
            #Calculating Arc Tangent with quadrant information
            c = numpy.arctan2(br,ar)
            #Turning it to -180 to 180 range
        
            z = numpy.multiply(c,self.constant)
            result = z.astype(numpy.int32)
            

            result[result < 0] += 360
            tang = numpy.reshape(result, (height,width))
            tang = tang.astype(numpy.float32)
            mat = cv.fromarray(tang)
            cv.Copy(mat,tangent16U)
        else:
            dxTemp = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_16S, 1)
            dyTemp = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_16S, 1)
            zero = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_16S, 1)
            
            cv.Add(zero, dx, dxTemp, Mask)
            cv.Add(zero, dy, dyTemp, Mask)
            
            dx = dxTemp
            dy = dyTemp
            

            
            for x in range(divSize[0]):
                for y in range(divSize[1]):
                    if Mask[y,x] == 0:
                        tangent16U[y,x] = 0
                        continue
                    tang = math.atan2(dy[y,x], dx[y,x])*self.constant
                    tangent16U[y,x] = int(tang)
                    
         
         
        
        
        if self.visualize:
            #tangent2 = cv.CreateImage(cv.GetSize(dy), cv.CV_16SC1, 1)
            cv.ConvertScaleAbs(tangent16U, tangent)
            cv.EqualizeHist(tangent, tangent)
            while True:
                cv.NamedWindow("Tangent")
                cv.ShowImage("Tangent", tangent)
                c = cv.WaitKey(5)
                if c > 0:
                    break
        cv.DestroyAllWindows()
        

        return tangent16U
    
    def Magnitude(self, dx,dy, Mask = None ,precise = True, method="cv"):
        '''Calculates the magnitude of the gradient using precise and fast approach'''

        dxconv = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_32F, dx.channels)
        dyconv = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_32F, dx.channels)
        dxdest = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_32F, dx.channels)
        dydest = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_32F, dx.channels)
        magdest = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_32F, dx.channels)
        magnitude = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_32F, dx.channels)
        magnitudetemp = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_32F, dx.channels)
        zero = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_32F, dx.channels)
        
        
        cv.Convert(dx, dxconv)
        cv.Convert(dy, dyconv)
       
        if precise:
            cv.Pow(dxconv,dxdest,2)
            cv.Pow(dyconv,dydest,2)
            cv.Add(dxdest, dydest, magdest)
            cv.Pow(magdest, magnitude, 1./2)
        else:
            #Add the |dx| + |dy|
            return None
        
        if method == "slow":
            size = cv.GetSize(magnitude)
            
            for x in range(size[0]):
                for y in range(size[1]):
                    if Mask == None:
                        pass
                    elif Mask[y,x] > 0:
                        pass
                    else:
                        magnitude[y,x] = 0
            
            final = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_8U, dx.channels)
            cv.ConvertScaleAbs(magnitude, final)
        else:
            cv.Add(zero, magnitude, magnitudetemp, Mask)     
            final = cv.CreateImage(cv.GetSize(dx), cv.IPL_DEPTH_8U, dx.channels)
            cv.ConvertScaleAbs(magnitudetemp, final)
         
        if self.visualize:
            magnitude2 = cv.CreateImage(cv.GetSize(dy), cv.IPL_DEPTH_8U, 1)
            cv.EqualizeHist(final, magnitude2)      
            while True:
                cv.NamedWindow("Magnitude")
                cv.ShowImage("Magnitude", magnitude2)
                c = cv.WaitKey(5)
                if c > 0:
                    
                    break
        cv.DestroyAllWindows()
        
        
        return final
    
    def cannyTangent(self, image, smooth = True, T1 = 20, T2 = 250, eq = False, method = "cv", method2 = "fast"):
        '''Gets the thresholded edge from opencv canny, and use the locations to take out the magnitude and gradient
        from manually calculated gradient using sobel mask'''
        gsimage = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 1)  
        smoothed = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 1) 
        final = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 1) 
        
        if image.channels > 1 :
            temp = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 1)
            cv.CvtColor(image, temp, cv.CV_BGR2GRAY)         
        else:
            temp = image
            
        if eq:
            cv.EqualizeHist(temp, gsimage)
        else:
            cv.Copy(temp, gsimage)
        
        if smooth:
            cv.Smooth(gsimage, smoothed)
            final = smoothed
        else:
            cv.Copy(gsimage,final)
        
        
        tempo = self.visualize
        self.visualize = False
        #a = time.time()
        gradient = self.cannyGradient(final, t1 = T1, t2 = T2 )
        #print "single canny time:", time.time() - a
        
        #a = time.time()
        (dx, dy) = self.sobelGradient(final)
        #print "single sobel time:", time.time() - a
        #a = time.time()
        tangent = self.tangent(dx, dy, gradient, method = method)
        #print "single tangent time:", time.time() - a
        #a = time.time()
        magnitude = self.Magnitude(dx, dy, gradient, method = method2)
        #print "single magnitude time:", time.time() - a
        self.visualize = tempo
        if self.visualize:
            jinjer = 0
            while True:
                tan = cv.CreateImage(cv.GetSize(tangent), cv.IPL_DEPTH_8U, 1)
                timp = cv.CreateImage(cv.GetSize(tangent), cv.IPL_DEPTH_8U, 1)
                cv.Convert(tangent, tan)
                cv.EqualizeHist(tan, timp)  
                cv.NamedWindow("Original")
                cv.MoveWindow("Original", 0, 0)
                cv.ShowImage("Original", final)
                #cv.NamedWindow("TangentCanny")
                #cv.ShowImage("TangentCanny", timp)
                cv.NamedWindow("magnitude")
                cv.MoveWindow("magnitude", 640, 0)
                cv.ShowImage("magnitude", magnitude)
                c = cv.WaitKey(5)
                jinjer += 1
                if c > 0 or jinjer > 1000000:
                    break
        cv.DestroyAllWindows()
        
        return (tangent, magnitude)
    
    def image_check(self, image):
        try:
            nchannels = image.channels
            if nchannels <= 0:
                self.logger.error("The image has no channels")
                return -1
            if nchannels > 1:
                self.logger.warning("The image should be grayscale, it will be transformed")
        except: 
            self.logger.error("The image is not an iplimage")  
            return -1
        return 0

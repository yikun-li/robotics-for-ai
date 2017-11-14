'''
Created on Sep 21, 2011

@author: shantia
'''

import cv2.cv as cv
import math
import logging
import util.nullhandler
import contrastnormalizer

logging.getLogger('Borg.Brain.Vision.hog.Divider').addHandler(util.nullhandler.NullHandler())

class Divider(object):
    '''
    This function divides an image in to m by n cells and returns a list of images.
    '''
    def __init__(self, visualize = False):
        
        self.logger = logging.getLogger("Borg.Brain.Vision.hog.Divider")
        self.visualize = visualize
        self.equilizer = contrastnormalizer.ContrastNormalizer(False)
    def divide(self, image, column = 8, row = 8, option = "normal", overlap = 0):
        
        #Checks whether inputs are correct
        if self.image_check(image) < 0: 
            return -1
        if self.mn_check(column, row) < 0: 
            return -2
        
        if option == "normal":
            #sizeTuple is a tuple as follow: (width, height)
            sizeTuple = cv.GetSize(image)
            
            widthSlice = math.floor(sizeTuple[0]/column) 
            heightSlice = math.floor(sizeTuple[1]/row)
            
            widthReminder = sizeTuple[0]%column
            heightReminder = sizeTuple[1]%column
            
            List = []
            
            rowIndex = 0
            columnIndex = 0
            
            #In these two while loops we make every sub image, crop it from original image and add it to a list
            #if there are reminders, the width and height of the final row/columns will differ
            while columnIndex < sizeTuple[0]:
                if columnIndex == sizeTuple[0] - widthSlice - widthReminder :
                    width = widthSlice + widthReminder
                else:   
                    width = widthSlice
                while rowIndex < sizeTuple[1]:
                    if rowIndex == sizeTuple[1] - heightSlice - heightReminder :
                        height = heightSlice + heightReminder
                    else:
                        height = heightSlice
                    
                    if columnIndex - overlap <= 0: #Overlapping the rectangles.
                        column = columnIndex
                    else:
                        column = columnIndex - overlap
                        
                    if rowIndex - overlap <= 0:
                        row = rowIndex
                    else:
                        row = rowIndex - overlap
                            
                    subRect = (column, row, width, height)
                    List.append(self.crop(image, subRect))
                    rowIndex += height
                rowIndex = 0    
                columnIndex += width
                
            if (self.visualize):
                windows = range(len(List))
                for x in windows:
                    cv.NamedWindow(str(x))
                while True:
                        for x in windows:
                            cv.ShowImage(str(x),List[x])
                        c = cv.WaitKey(5)
                        if c > 0:
                            print c
                            break
                cv.DestroyAllWindows()
                    
                
            return List
        else:
            #sizeTuple is a tuple as follow: (width, height)
            columnstep = 1
            rowstep = 1
            maxcolumn = column
            maxrow = row
            List = []
            while columnstep <= maxcolumn and rowstep <= maxrow:
                
                column = columnstep
                row = rowstep
                 
                sizeTuple = cv.GetSize(image)
                
                widthSlice = math.floor(sizeTuple[0]/column) 
                heightSlice = math.floor(sizeTuple[1]/row)
                
                widthReminder = sizeTuple[0]%column
                heightReminder = sizeTuple[1]%column
                
                
                
                rowIndex = 0
                columnIndex = 0
                
                #In these two while loops we make every sub image, crop it from original image and add it to a list
                #if there are reminders, the width and height of the final row/columns will differ
                while columnIndex < sizeTuple[0]:
                    if columnIndex == sizeTuple[0] - widthSlice - widthReminder :
                        width = widthSlice + widthReminder
                    else:   
                        width = widthSlice
                    while rowIndex < sizeTuple[1]:
                        if rowIndex == sizeTuple[1] - heightSlice - heightReminder :
                            height = heightSlice + heightReminder
                        else:
                            height = heightSlice
                            
                        if columnIndex - overlap <= 0: #Overlapping the rectangles.
                            column = columnIndex
                        else:
                            column = columnIndex - overlap
                            
                        if rowIndex - overlap <= 0:
                            row = rowIndex
                        else:
                            row = rowIndex - overlap
                        subRect = (int(column), int(row), int(width), int(height))
                        List.append(self.crop(image, subRect))
                        rowIndex += height
                    rowIndex = 0    
                    columnIndex += width
                    
                if (self.visualize):
                    windows = range(len(List))
                    for x in windows:
                        cv.NamedWindow(str(x))
                    while True:
                        for x in windows:
                            tan = cv.CreateImage(cv.GetSize(List[x]), cv.IPL_DEPTH_8U, 1)
                            final = cv.CreateImage(cv.GetSize(List[x]), cv.IPL_DEPTH_8U, 1)
                            cv.CvtColor(List[x], tan, cv.CV_RGB2GRAY)
                            cv.EqualizeHist(tan, final)  
                            cv.NamedWindow(str(x))
                            cv.ShowImage(str(x),final)
                        c = cv.WaitKey(5)
                        if c > 0:
                            print c
                            break
                    cv.DestroyAllWindows()
                    
                columnstep *= 2
                rowstep *= 2
                        
                    
            return List    
    
    def image_check(self, image):
        try:
            nchannels = image.channels
            if nchannels <= 0:
                self.logger.error("The image has no channels")
                return -1
        except: 
            self.logger.error("The image is not an iplimage")  
            return -1
        return 0
    
    def mn_check(self,column = 8, row = 8):
        if row <= 0 or column <= 0 :
            self.logger.error("The width and height are not logical")
            return -2
        return 0
    
    def crop(self, image, subRect):

        cv.SetImageROI(image, subRect);
        img2 = cv.CreateImage(cv.GetSize(image), image.depth, image.nChannels);
        #img3 = cv.CreateImage(cv.GetSize(image), image.depth, image.nChannels);
        cv.Copy(image, img2);
        #img3 = self.equilizer.normalize(img2)
        cv.ResetImageROI(image);
        return img2

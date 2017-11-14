import sys
import cv
import os
import Image
import time

import util.vidmemreader

class FaceDetector():
    """A module that returns detected faces.
    
    Faces are detected on a webcam feed or on images in sub-directories
    in 'brain/data/whoiswho/', using the OpenCV Haarface implementation.
    The sub-directories should have the name of the person the images
    in it are taken from, and should have two sub-dirs themselves,
    'train/' and 'test/'.
    An example image would be '/brain/data/whoiswho/alice/train/IMG_01.jpg'.
    """
    
    def __init__(self, cascade = 'haarcascade_frontalface_default',\
                    video_source = 'webcam'):
        cascadedir = os.environ['BORG'] + '/brain/data/haarcascades/' + cascade + '.xml'
        self.__cascade = cv.Load(cascadedir)
        if video_source == 'webcam':
            self.__vmreader = util.vidmemreader.VidMemReader([video_source])
        self.__source = video_source
        self.__imgdir = os.environ['BORG'] + '/brain/data/who_is_who/'
        self.__dirlen = 0
        self.__img_idx = 0
        self.__do_once = False
        self.__image_scale = 2.4
        self.__haar_scale = 1.2
        self.__min_neighbors = 2
        self.__min_size = (7, 7)
    
    def getFaces(self, train, name, source):
        """Returns detected faces, if any.
        
        Parameters:
        - boolean train: specifies if 'train/' or 'test/' folder is used
        - string name: the name of the current person
        - string source: 'webcam' or 'folder'
        If source is 'webcam', the train and name params can be ignored.
        """
        #print 'Facedetect getting faces, index:', self.__img_idx
        img = self.__getImage(train, name, source)
        if img is not None:
            faces = self.__findFaces(img)
            return faces
        else:
            return None
    
    def getIndex(self):
        """Returns the index of the image currently used.
        This is only applicable if working with a folder."""
        return self.__img_idx
    
    def getDir(self, train, name):
        """Returns a list, and its length, of all .jpg files
        in the directory specified by train and name."""
        try:
            dirlist = os.listdir(self.__imgdir + name + ['/test/','/train/'][train])
        except:
            print 'Cannot make dirlist'
            return None
        
        for obj in dirlist:
            if obj[-4:] != '.jpg':
                del dirlist[dirlist.index(obj)]
        #print 'Facedetect dirlist is', dirlist, 'length:', len(dirlist)
        self.__dirlen = len(dirlist)
        return dirlist, self.__dirlen
    
    def folder_exists(self, train, name):
        """Returns True if the '/brain/data/<name>/<train>/' folder exists,
        False otherwise."""
        if self.getDir(train, name) is None:
            print 'Directory does not exist'
            return False
        return True
    
    def finished(self):
        """Returns False if training should continue and
        True if the whole folder has been checked."""
        if self.__source == 'folder':
            if self.__dirlen == 0:
                return True
            if not self.__do_once:
                self.__do_once = True
                return False
            elif self.__do_once:
                if self.__img_idx < self.__dirlen:
                    return False
                elif self.__img_idx >= (self.__dirlen - 1):
                    return True
        elif self.__source == 'webcam':
            return False
    
    def reset(self):
        """Resets the image index, so the module can start in a new folder."""
        self.__img_idx = 0
        self.__do_once = False
    
    def __findFaces(self, image):
        """Returns cropped grayscale images of detected faces."""
        faces = self.__detectFaces(image)
        iplFaces = []
        if faces:
            for (rectangle, n) in faces:
                rectangle = self.__resizeRect(rectangle)
                resized = cv.CreateImage((320, 320), 8, 1)
                src_region = cv.GetSubRect(image, rectangle)
                cv.Resize(src_region, resized, cv.CV_INTER_LINEAR)
                cv.EqualizeHist(resized, resized)               
                iplFaces.append(resized)
        return iplFaces
    
    def __detectFaces(self, grayscaled):
        """Returns a list of detected faces."""
        smallImg = cv.CreateImage((cv.Round(grayscaled.width / self.__image_scale),
                                    cv.Round(grayscaled.height / self.__image_scale)), 8, 1)
        cv.Resize(grayscaled, smallImg, cv.CV_INTER_LINEAR)
        cv.EqualizeHist(smallImg, smallImg)
        
        store = cv.CreateMemStorage(1024)
        faces = cv.HaarDetectObjects(smallImg, self.__cascade, store,\
                                    self.__haar_scale, self.__min_neighbors,\
                                    cv.CV_HAAR_DO_CANNY_PRUNING, self.__min_size)
        return faces
    
    def __getImage(self, train, name, source):
        """Returns images taken from the specified source, converted to grayscale.
        Returns None if there is no image left to detect on."""
        if source == 'webcam':
            #print "Getting image from webcam"
            img = self.__vmreader.get_latest_image()[0]
        elif source == 'folder':
            try:
                #print "Getting image from folder"
                dirlist = self.getDir(train, name)[0]
                img = cv.LoadImage(self.__imgdir + name +\
                        ['/test/','/train/'][train] + dirlist[self.__img_idx])
                #print 'Facedetect current image is:', self.__imgdir +\
                #            name + ['/test/','/train/'][train] +\
                #            dirlist[self.__img_idx], ':', img
                if self.__img_idx < len(dirlist):
                    self.__img_idx += 1
            except:
                return None
            
        #cv.ShowImage('Facedetect', img)
        #cv.WaitKey(10)
        grayscaled = cv.CreateImage((img.width,img.height), 8, 1)
        cv.CvtColor(img, grayscaled, cv.CV_BGR2GRAY)
        return grayscaled
    
    def __resizeRect(self, rect):
         (x,y,w,h) = rect
         x = cv.Round(x * self.__image_scale)
         y = cv.Round(y * self.__image_scale)
         w = cv.Round(w * self.__image_scale)
         h = cv.Round(h * self.__image_scale)
         rect = (x,y,w,h)
         return rect

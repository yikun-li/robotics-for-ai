import cv
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Util.TakeImages').addHandler(util.nullhandler.NullHandler())

class TakeImages(object):
    '''
    The operation related to taking images from multiple cameras, regardless of the platform. It is possible to
    store on harddisk. All Images will be written in shared memory and there will be a file for each camera that
    points out to the last image. This file is a shared variable therefore, while it is being written no one will
    have access to it
    '''
    def __init__(self, cameranumber=-1, source='camera', resolution='verylow', filename=None):
        '''
        Constructor
        @param cameranumber - Represents the index of the camera you want to use. By default it is -1 (first cam)
        @param source - The source you want to use, by default it's camera, other option is file.
        @param resolution - The resolution of the captured images.
        @param filename - Specify the file you want to use with the source=file option.
        '''
        self.logger = logging.getLogger('Borg.Brain.Util.TakeImages')
        self.source = source
        self.filename = filename
        self.Nocamera = False
        if self.source == 'camera':
            # 200 is the V4L (or more general: system native) video device
            # category of OpenCV
            if cameranumber != -1:
                self.cameranumber = 200 + cameranumber
            else:
                self.cameranumber = 200
            self.CAM = cv.CaptureFromCAM(self.cameranumber)
            if str(self.CAM) == "<Capture (nil)>":
                self.Nocamera = True
            if True:
                cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_WIDTH, 1024)
                cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_HEIGHT, 768)
            elif resolution == 'low':
                cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_WIDTH, 640)
                cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
            # else use default settings
            elif resolution == 'verylow':
                cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_WIDTH, 1280)
                cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_HEIGHT, 960)
            elif resolution == 'veryverylow':
                cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_WIDTH, 160)
                cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_HEIGHT, 120)
            else:
                cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_WIDTH, 1280)
                cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_HEIGHT, 960)
            # http://opencv.willowgarage.com/documentation/python/reading_and_writing_images_and_video.html#setcaptureproperty
            cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_WIDTH, 1280)
            cv.SetCaptureProperty(self.CAM, cv.CV_CAP_PROP_FRAME_HEIGHT, 960)
            # QUOTE: This function currently does nothing when using 
            # the latest CVS download on linux with FFMPEG 
            # (the function contents are hidden if 0 is used and returned).
            temp = cv.QueryFrame(self.CAM)
            if temp:
                self.logger.debug("%s Image size:%sx%s" % (self.CAM, temp.width, temp.height))
            
    def get_image(self):
        if self.source == 'camera':
            if self.Nocamera == True:
                return "Error! No camera Detected"
            else:
                temp = cv.QueryFrame(self.CAM)
                if temp == None:
                    return "No Feed from Camera"
                else:
                    return temp

        elif self.source == 'file':
            if self.filename == None:
                self.logger.error("Filename not specified.")
                return Error
            self.logger.debug("File detected")
            iscolor = cv.CV_LOAD_IMAGE_UNCHANGED
            self.logger.debug("File returned")
            return cv.LoadImage(self.filename,iscolor)
    
    def store_harddisk(self,filename,image):
        cv.SaveImage(filename, image)

if __name__ == "__main__":
    ##### Capture from 2 cameras: #####
#    takeImages_1 = TakeImages(cameranumber=1, source='camera')
#    cv.NamedWindow("1")
#
#    takeImages_2 = TakeImages(cameranumber=0, source='camera')
#    cv.NamedWindow("2")
#
#    handle1 = takeImages_1.get_image()
#    while True:
#        handle1 = takeImages_1.get_image()
#        handle2 = takeImages_2.get_image()
#        cv.ShowImage('1', handle1)
#        cv.WaitKey(5)
#        cv.ShowImage('2', handle2)
#        cv.WaitKey(5)

    ##### Capture from 1 camera: #####
    cameranumber = 0
    takeImages = TakeImages(cameranumber, 'camera')

    import time

    frametime = 0.0
    frames = 0

    while True:
        start = time.time()
        image = takeImages.get_image()
        print image
        cv.ShowImage("Camera index: " + str(cameranumber), image)
        
        k = cv.WaitKey(5)
        frametime += (time.time() - start)
        frames += 1 
        if frames > 0:
            avgframe = frametime / frames
            print "FPS: %f" % (1.0 / avgframe)
        if k == 0x1b:
            print 'ESC pressed. Exiting ...'
            break

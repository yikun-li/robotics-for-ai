#import cv2.cv as cv
import cv
import numpy as np
import freenect

import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Util.KinectVideo').addHandler(util.nullhandler.NullHandler())
logger = logging.getLogger('Borg.Brain.Util.KinectVideo')


def DoNiceConvert8(depth):

    np.clip(depth, 0, 2**10 - 1, depth)
    depth >>= 2
    depth = depth.astype(np.uint8)

    image = cv.CreateImageHeader((depth.shape[1], depth.shape[0]),
                                 cv.IPL_DEPTH_8U,
                                 1)
    cv.SetData(image, depth.tostring(),
               depth.dtype.itemsize * depth.shape[1])
    return image

def DoUsefulConvert8(depth):

    #np.clip(depth, 0, 2**10 - 1, depth)
#    depth >>= 1
    depth = depth - 400
    depth >>= 3
    depth = depth.astype(np.uint8)


    image = cv.CreateImageHeader((depth.shape[1], depth.shape[0]),
                                 cv.IPL_DEPTH_8U,
                                 1)
    cv.SetData(image, depth.tostring(),
               depth.dtype.itemsize * depth.shape[1])
    return image



def DoNiceConvert11(depth):

    image = cv.CreateImageHeader((depth.shape[1], depth.shape[0]),
                                 cv.IPL_DEPTH_16U,
                                 1)
    cv.SetData(image, depth.tostring(),
               depth.dtype.itemsize * depth.shape[1])
    return image
  
def DoNiceConvertRGB(video):

    video = video[:, :, ::-1]  # RGB -> BGR
    image = cv.CreateImageHeader((video.shape[1], video.shape[0]),
                                 cv.IPL_DEPTH_8U,
                                 3)
    cv.SetData(image, video.tostring(),
               video.dtype.itemsize * 3 * video.shape[1])
    return image


def GetDepth8():
    depth = freenect.sync_get_depth()[0]

    return DoUsefulConvert8(depth)


def GetDepth11():
    depth = freenect.sync_get_depth()[0]

    return DoNiceConvert11(depth)

 

def GetRGB():
    video = freenect.sync_get_video()[0] 

    return DoNiceConvertRGB(video)
	
def GetBoth():

#    freenect.stop_depth()
#    freenect.stop_video()
    depth = freenect.sync_get_depth()[0]
    video = freenect.sync_get_video()[0] 
#    freenect.start_depth()
#    freenect.start_video()



    return (DoUsefulConvert8(depth),DoNiceConvertRGB(video))

def StopKinect():
    freenect.sync_stop()

if __name__ == "__main__":
    while True:
        image = GetDepth8()
        if image:
            cv.ShowImage("Kinect view", image)
            cv.SaveImage("/home/borg/freenect.png", image)  
        else:
            print "invalid image %s" % repr(image)
        
        k = cv.WaitKey(5)
        if k == 0x1b:
            print 'ESC pressed. Exiting ...'
            break

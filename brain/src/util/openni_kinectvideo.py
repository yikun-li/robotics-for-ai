import cv
import numpy
import time

import logging
import util.nullhandler

################################################################################
## OPENNI/OPENCV MODULE TO ACCESS KINECT                                      ##
################################################################################
## This module implements the same functionality as the kinectvideo module,   ##
## but is uses OpenNI through OpenCV to access the kinect instead of the      ##
## libfreenect library.                                                       ##
##                                                                            ##
## Opencv Python do not currently publish the OpenNI constants. Therefore,    ##
## they are added to the module here. The values are taken from highgui_c.h   ##
## file from the OpenCV includes.                                             ## 
################################################################################
# OpenCV Capture Sources
cv.CV_CAP_ANY = 0
cv.CV_CAP_V4L = 200
cv.CV_CAP_VFW = 200
cv.CV_CAP_OPENNI = 900

# OpenNI map generators
cv.CV_CAP_OPENNI_DEPTH_GENERATOR = 0
cv.CV_CAP_OPENNI_IMAGE_GENERATOR = -1 << 31
cv.CV_CAP_OPENNI_GENERATORS_MASK = -1 << 31

# Properties of cameras available through OpenNI interfaces
cv.CV_CAP_PROP_OPENNI_OUTPUT_MODE      = 100
cv.CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH  = 101 # in mm
cv.CV_CAP_PROP_OPENNI_BASELINE         = 102 # in mm
cv.CV_CAP_PROP_OPENNI_FOCAL_LENGTH     = 103 # in pixels
cv.CV_CAP_PROP_OPENNI_REGISTRATION_ON  = 104 # flag

# CV_CAP_PROP_OPENNI_REGISTRATIONS is a flag that synchronizes the remapping
# depth map to image map by changing depth generator's view point (if the flag
# is "on") or sets this view point to its normal one (if the flag is "off").
cv.CV_CAP_PROP_OPENNI_REGISTRATION     = cv.CV_CAP_PROP_OPENNI_REGISTRATION_ON 
                                                                               
                                                                               
cv.CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE =                                 \
    cv.CV_CAP_OPENNI_IMAGE_GENERATOR + cv.CV_CAP_PROP_OPENNI_OUTPUT_MODE
cv.CV_CAP_OPENNI_DEPTH_GENERATOR_BASELINE =                                    \
    cv.CV_CAP_OPENNI_DEPTH_GENERATOR + cv.CV_CAP_PROP_OPENNI_BASELINE
cv.CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH =                                \
    cv.CV_CAP_OPENNI_DEPTH_GENERATOR + cv.CV_CAP_PROP_OPENNI_FOCAL_LENGTH
cv.CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON =                             \
    cv.CV_CAP_OPENNI_DEPTH_GENERATOR + cv.CV_CAP_PROP_OPENNI_REGISTRATION_ON

# Data given from depth generator
cv.CV_CAP_OPENNI_DEPTH_MAP = 0          # Depth values in mm
cv.CV_CAP_OPENNI_POINT_CLOUD_MAP = 1    # XYZ in meters (CV_32FC3)
cv.CV_CAP_OPENNI_DISPARITY_MAP = 2      # Disparity in pixels
cv.CV_CAP_OPENNI_DISPARITY_MAP_32F = 3  # Disparity in pixels (CV_32FC1)
cv.CV_CAP_OPENNI_VALID_DEPTH_MASK = 4   # CV_8UC1

# Data given from RGB image generator.
cv.CV_CAP_OPENNI_BGR_IMAGE = 5
cv.CV_CAP_OPENNI_GRAY_IMAGE = 6

# Supported output modes of OpenNI image generator
cv.CV_CAP_OPENNI_VGA_30HZ = 0
cv.CV_CAP_OPENNI_SXGA_15HZ = 1

def cv2array(im):
    """
    Convert a OpenCV image to a numpy array. From the opencv FAQ
    """
    depth2dtype = {
          cv.IPL_DEPTH_8U: 'uint8',
          cv.IPL_DEPTH_8S: 'int8',
          cv.IPL_DEPTH_16U: 'uint16',
          cv.IPL_DEPTH_16S: 'int16',
          cv.IPL_DEPTH_32S: 'int32',
          cv.IPL_DEPTH_32F: 'float32',
          cv.IPL_DEPTH_64F: 'float64',
      }


    arrdtype = im.depth
    a = numpy.fromstring(
           im.tostring(),
           dtype=depth2dtype[im.depth],
           count=im.width*im.height*im.nChannels)
    a.shape = (im.height,im.width,im.nChannels)
    return a

def doUsefulConvert8(depth):
    """
    Copy from the freenect kinectvideo implementation. Convert the
    depth map values into usable values between 0 and 255 in the
    useful minimum range of the kinect
    """
    depth = depth - 400
    depth >>= 3
    depth = depth.astype(numpy.uint8)

    image = cv.CreateImageHeader((depth.shape[1], depth.shape[0]),
                                 cv.IPL_DEPTH_8U,
                                 1)
    cv.SetData(image, depth.tostring(), depth.dtype.itemsize * depth.shape[1])
    return image

logging.getLogger('Borg.Brain.Util.OpenNIKinectVideo').addHandler(util.nullhandler.NullHandler())
logger = logging.getLogger('Borg.Brain.Util.OpenNIKinectVideo')

NI_cameranumber = None
NI_camera = None
NI_grabtime = None

class OpenNIKinect(object):
    """
    This class wraps the OpenNI interface to the Kinect using OpenCV. It
    does require a small patch to the Python wrappers of OpenCV that includes
    the image index in the RetrieveFrame method exposed to Python. This way,
    the type of image (RGB/Depth/PointCloud) can be specified.

    Currently, it uses a conversion to a numpy array and back to convert the
    depth image into something we can use. This of course means a performance
    degradation, so we need to find out if it can be done in an easier way.

    Since OpenCV allows only one use of a single Kinect camera, it is stored
    as a global variable in the openni_kinectvideo module. Each instance of
    this class uses the global variable to access the image data, and obtains
    the correct image (RGB, Depth or PointCloud) from OpenCV.
    """
    def __init__(self, img_type="depth", update_frequency=10, cameranumber=0):
        self.logger = logging.getLogger('Borg.Brain.Util.OpenNIKinectVideo')
        self.Nocamera = False

        global NI_cameranumber
        global NI_camera
        global NI_grabtime

        # Initialize the global OpenCV capture object, if it is not yet
        # available. 
        if not NI_cameranumber:
            NI_cameranumber = cv.CV_CAP_OPENNI + cameranumber
        if not NI_camera:
            NI_camera = cv.CaptureFromCAM(NI_cameranumber)

        self.width = 640
        self.height = 480
        if img_type != "depth" and img_type != "rgb" and img_type != "pcl":
            img_type = "depth"

        self.img_type = img_type
        
        # Check if camera initialization worked
        if str(NI_camera) == "<Capture (nil)>":
            self.Nocamera = True

        # Set capture mode to VGA @ 30Hz
        cv.SetCaptureProperty(NI_camera,                                       \
                              cv.CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE,    \
                              cv.CV_CAP_OPENNI_VGA_30HZ)

        temp = None
        try:
            temp = cv.QueryFrame(NI_camera)
        except:
            self.Nocamera = True

        if temp:
            self.logger.debug("%s Image size:%sx%s"                            \
                              % (NI_camera, temp.width, temp.height))
            self.width = temp.width
            self.height = temp.height

        if not NI_grabtime:
            NI_grabtime = time.time() - 10

        self.grab_interval = 1.0 / update_frequency # 30 fps
            
    def get_image(self):
        """
        Retrieve an image of the correct type from the Kinect, depending on the
        type that was passed to the constructor.

        Since the classes share a OpenNI camera instance, only obtain the image
        at the set update frequency.
        """
        global NI_grabtime
        global NI_camera

        if time.time() > NI_grabtime + self.grab_interval:
            cv.GrabFrame(NI_camera)
            NI_grabtime = time.time()

        if self.img_type == "depth":
            depth = cv.RetrieveFrame(NI_camera, cv.CV_CAP_OPENNI_DEPTH_MAP)
            temp = cv.CreateImage(cv.GetSize(depth),cv.IPL_DEPTH_8U,1)
            cv.ConvertScale(depth,temp,0.0625,0.0)
#            temp = doUsefulConvert8(cv2array(depth))
        elif self.img_type == "rgb":
            temp = cv.RetrieveFrame(NI_camera, cv.CV_CAP_OPENNI_BGR_IMAGE)
        elif self.img_type == "pcl":
            temp = cv.RetrieveFrame(NI_camera, cv.CV_CAP_OPENNI_POINT_CLOUD_MAP)

        if temp == None:
            raise Exception("Unable to start Kinect, check connection")
        return temp

def GetDepth8():
    """
    A compatibility wrapper for the GetDepth8 function from the kinectvideo
    module. This method uses the class above to get a depth image and return it.
    It uses a new object for each invocation but since they all share the same
    global camera instance, it should not degrade performance that much.
    """
    obj = OpenNIKinect('depth')
    return obj.get_image()

def GetRGB():
    """
    A compatibility wrapper for the GetRGB function from the kinectvideo
    module. This method uses the class above to get a depth image and return it.
    It uses a new object for each invocation but since they all share the same
    global camera instance, it should not degrade performance that much.
    """
    obj = OpenNIKinect('rgb')
    return obj.get_image()

################################################################################
## DRIVER                                                                     ##
################################################################################
## This program will open the Kinect using opencv and OpenNI and tries to     ##
## grab a depth image as often as possible, displaying it using OpenCV.       ##
################################################################################
if __name__ == "__main__":
    import math
    kinect = OpenNIKinect("rgb")
    kinect_rgb = OpenNIKinect("depth")

    ticktimes = []
    a = 5
    while True:
        ticktimes = ticktimes[-100:]
        start = time.time()
        image = kinect.get_image()
        image2 = kinect_rgb.get_image()
        print image2.depth
        if image and image2:
            cv.ShowImage("Kinect view", image)
            cv.ShowImage("Kinect vrgb iew", image2)
            cv.SaveImage("/home/borg/Openni.png",image2)
        else:
            print "invalid image %s" % repr(image)
            
        if a == 0:
            a = 5 
            avg = 1.0 / (math.fsum(ticktimes) / len(ticktimes))
            print "%.2f fps" % avg
        k = cv.WaitKey(5)
        if k == 0x1b:
            print 'ESC pressed. Exiting ...'
            break
        ticktimes.append(time.time() - start)
        a -= 1

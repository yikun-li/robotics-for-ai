
import kinectvideo as kv
import cv2.cv as cv
import os
import time
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Util.KinectMemWriter').addHandler(util.nullhandler.NullHandler())

class KinectDepthSource(object):
    def __init__(self): 
        self.logger = logging.getLogger('Borg.Brain.Util.KinectDepthSource')
        self.scale = 1

    def get_image(self):
        depth = kv.GetDepth8()
        temp = cv.CreateImage((cv.GetSize(depth)[0] / self.scale,
                               cv.GetSize(depth)[1] / self.scale),
                               cv.IPL_DEPTH_8U, 1)        
        cv.Resize(depth,temp)
        depth = temp
        
        return depth

        temp = cv.CreateImage((cv.GetSize(depth)[0]/self.scale,cv.GetSize(depth)[1]/self.scale),cv.IPL_DEPTH_8U,1)        
        cv.Resize(depth,temp)
        depth=temp
        self.current_idx = (self.current_idx + 1) % self.max_n_images
        
class KinectRGBSource(object):
    def __init__(self):
        self.logger = logging.getLogger('Borg.Brain.Util.KinectRGBSource')
        self.scale = 1

    def get_image(self):
        rgb = kv.GetRGB()
        temp = cv.CreateImage((cv.GetSize(rgb)[0] / self.scale,
                               cv.GetSize(rgb)[1] / self.scale),
                              cv.IPL_DEPTH_8U, 3)        
        cv.Resize(rgb, temp)
        rgb = temp
        return kv.GetRGB()

class KinectMemWriter():
    def __init__(self):
        self.logger = logging.getLogger('Borg.Brain.Util.KinectMemWriter')
        self.cleanup()
        self.max_n_images = 50
        os.system("mkdir /dev/shm/images -m 0777 2>/dev/null")
	os.system("rm -r /dev/shm/images/kinect_rgb/*")
	os.system("rm -r /dev/shm/images/kinect_depth/*")
        os.system("mkdir /dev/shm/images/kinect_rgb -m 0777 2>/dev/null")
        os.system("mkdir /dev/shm/images/kinect_depth 0777 2>/dev/null")
        self.current_idx = 0
        self.scale = 1

    def update(self):
        (depth, rgb) = kv.GetBoth()
        temp = cv.CreateImage((cv.GetSize(rgb)[0]/self.scale,cv.GetSize(rgb)[1]/self.scale),cv.IPL_DEPTH_8U,3)        
        cv.Resize(rgb,temp)
        rgb = temp
        temp = cv.CreateImage((cv.GetSize(depth)[0]/self.scale,cv.GetSize(depth)[1]/self.scale),cv.IPL_DEPTH_8U,1)        
        cv.Resize(depth,temp)
        depth=temp
        self.current_idx = (self.current_idx + 1) % self.max_n_images

        #write the image for RGB
        dirname = "/dev/shm/images/kinect_rgb"
        filename = dirname + "/" + str(self.current_idx) + ".png"
        try:
            cv.SaveImage(filename, rgb)
        except:
            raise Exception("No valid image was received from source: kinect_rgb")
        os.system("chmod a+rwx " + filename)
        #write the counter
        #f = open(dirname+"/lastimage", 'w')
        #f.write(str(self.current_idx))
        #f.close()
        #os.system("chmod a+rwx " + dirname + "/lastimage")

        #write the image for Depth
        dirname = "/dev/shm/images/kinect_depth"
        filename = dirname + "/" + str(self.current_idx) + ".png"
        try:
            cv.SaveImage(filename, depth)
        except:
            raise Exception("No valid image was received from source: kinect_depth")
        os.system("chmod a+rwx " + filename)
        #write the counter
        #f = open(dirname+"/lastimage", 'w')
        #f.write(str(self.current_idx))
        #f.close()
        #os.system("chmod a+rwx " + dirname + "/lastimage")

    def __del__(self):
        """
        Clean up if the program is terminated. In that case, files in shared
        memory should be deleted. However, if the program is stopped
        prematurely this method might not be executed.
        """
        pass
#self.cleanup()

    def cleanup(self):
        """Remove the folder containing the images from /dev/shm"""
        os.system("rm -rf /dev/shm/images/kinect_rgb")
        os.system("rm -rf /dev/shm/images/kinect_depth")


if __name__ == "__main__":

    kmw = KinectMemWriter()

    while (True):
        #time.sleep(0.2);
        kmw.update()

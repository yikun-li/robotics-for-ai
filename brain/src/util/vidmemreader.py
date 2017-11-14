import cv2 as cv
import logging
import sys
import os
import glob
import time
#import base64

import util.nullhandler
from util.binarysocket import BinarySocket

#Used to compare to ros image topics:
import re
ros_pattern = re.compile("ros_")

logging.getLogger('Borg.Brain.Util.VidMemReader').addHandler(util.nullhandler.NullHandler())

class VidMemReader():
    """Reads video frames from shared memory to be used by vision modules"""

    def __init__(self, namelist):
        """initialize VidMemReader
	@param namelist - the list of the sources where to read from
	"""
        self.logger = logging.getLogger('Borg.Brain.Util.VidMemReader')
        self.namelist = namelist
        self.last_images = []
        self.last_status = []
        self.fail_counter = [0] * len(namelist)
        for s in self.namelist:
            self.last_images.append((None, None))
            self.last_status.append(False)
        self.img = None
        self.conn = BinarySocket("localhost", "VideoManager", server=False, bufsize=256*1024)

    def request_image(self, source):
        # Request the new image
        start = time.time()
        self.conn.sendall({"get_image": source})

        data = self.conn.wait_data(0.15)
        for meta, img_string in data:
            if 'error' in meta:
                self.logger.warn("Obtaining image failed. Error message: %s" % meta['error'])
                continue
            shape = meta['shape']
            nchannels = meta['nChannels']
            depth = meta['depth']
            mtime = meta['time']
            img = cv.CreateImageHeader(shape, depth, nchannels)
            cv.SetData(img, img_string)
            took = time.time() - start
            return mtime, img

        took = time.time() - start
        return None, None

    def get_latest_image(self, just_filename=False, mtimes=False):
        """
        Get a list of the latest images of each of the names in namelist
        """
        images = []
        mod_times = []
        vm_sources = ['kinect_rgb', 'kinect_depth', 'webcam', 'naovideo', 'ros_']
        for idx, source in enumerate(self.namelist):
            if source in vm_sources or ros_pattern.match(source):
                mtime, img = self.request_image(source)
            else:
                print "Loading image from file"
                mtime, img = self.get_latest_image_file(source)

            if not img:
                print "No image received"
                self.last_status[idx] = False
                self.fail_counter[idx] += 1
                last_time, last_img = self.last_images[idx]
                if last_img:
                    images.append(last_img)
                    mod_times.append(last_time)
                else:
                    images.append(self.empty_image(source))
                    mod_times.append(time.time())
                continue

            self.last_status[idx] = True
            self.last_images[idx] = (mtime, img)
            images.append(img)
            mod_times.append(mtime)
            
        if mtimes:
            return zip(images, mod_times)
        return images

    def get_status(self, source):
        try:
            idx = self.namelist.index(source)
            return self.last_status[idx]
        except:
            return False

    def get_latest_image_file(self, source):
        dirname = "/dev/shm/images/" + source

        if not os.path.exists(dirname):
            self.logger.error("Directory %s does not exist; returning empty image" % dirname)
            return None, None

        files = glob.glob(os.path.join(dirname, "*.png")) + glob.glob(os.path.join(dirname, "*.jpg"))
        if not files:
            self.logger.error("No images in directory %s; returning empty image" % dirname)
            return None, None

        #Load the actual file
        linkname = os.path.join(dirname, "lastimage")
        try:
            filename = os.path.realpath(linkname)
        except OSError as e:
            if e.errno != 2: 
                raise 
            # OSError 2 = File not found; use fallback
            filename = linkname
        if linkname == filename or not os.path.exists(filename):
            # if anybody knows how to suppress warning when there's nothing in the folder please add it here...
            second_last_file_name = os.popen("ls -tr " + dirname + " | head -n $(expr $(ls " + dirname + " | wc -l) - 1) | tail -n 1").read().replace("\n","")
            #Load the actual file
            filename = os.path.join(dirname, second_last_file_name)
        try:
            img = cv.LoadImage(filename)
            mtime = os.path.getmtime(filename)
            return mtime, img
        except:
            return None, None

    def empty_image(self, source):
        if source == "kinect_depth":
            return cv.CreateImage((640,480), cv.IPL_DEPTH_8U, 1)
        else:
            return cv.CreateImage((640,480), cv.IPL_DEPTH_8U, 3)

    def get_latest_ascii(self):
        """ls " + dirname + " | wc -l)
        Get a list of the latest ascii of each of the names in namelist
        """
        
        for name in range(len(self.namelist)):
            #Read the index of the latest file
            dirname = "/dev/shm/ascii/" + self.namelist[name]

            #Load the actual file
            # if anybody knows how to suppress warning when there's nothing in the folder please add it here...
            second_last_file_name = os.popen("ls -tr " + dirname + " | head -n $(expr $(ls " + dirname + " | wc -l) - 1) | tail -n 1").read().replace("\n","")
            #Load the actual file
            if (len(second_last_file_name) == 0):
                return None
            filename = dirname + "/" + second_last_file_name
            
        return filename

    def get_resolution(self):
        """ Returns the resolution of the image """
        if self.img != None:
            return ({'width':self.img.width, 'height':self.img.height})
        else:
            self.logger.info("No images captured yet ...")
            return

if __name__ == "__main__":
    if len(sys.argv) == 1:
        vmr = VidMemReader(["webcam"])
    else:
        sources = sys.argv
        sources.pop(0)
        print sources
        vmr = VidMemReader(sources)
    while(True):
        image = vmr.get_latest_image()[0]
        cv.ShowImage("vidmemreader", image)
        cv.WaitKey(5)


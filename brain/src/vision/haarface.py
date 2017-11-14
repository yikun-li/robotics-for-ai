#!/usr/bin/python
#TODO: Clean up observation properties (test/verify angle/distance).
#TODO: Clean up code.
#TODO: Test if it also works on video sources other than webcam


"""example stand-alone vision module"""
import sys
import cv
import time
import os
import logging
import util.nullhandler
import util.loggingextra

import util.naovideo
import util.sendsocket
import configparse
import util.vidmemreader
import util.speed_angle
from abstractvisionmodule import AbstractVisionModule

logging.getLogger('Borg.Brain.Vision.Haarface').addHandler(util.nullhandler.NullHandler())

class HaarFace(AbstractVisionModule):
    """example vision class. Detects faces using the openCV face detector"""

    def __init__(self, host, port, update_frequency, source="webcam", cascade="haarcascade_frontalface_default", verbose=False):
        """Initialize vision system with update frequency, ip adress, and Haar cascade"""
        self.logger = logging.getLogger("Borg.Brain.Vision.Haarface")
        super(HaarFace, self).__init__(host, port) 
        self.update_frequency = update_frequency
        cascadedir = os.environ['BORG'] + "/brain/data/haarcascades/" + cascade + ".xml"
        self.verbose = verbose
        self.cascade = cv.Load(cascadedir)
        self.source = [source]
        self.vid_mem_reader = util.vidmemreader.VidMemReader(self.source)
        # set the type of objects that someone can detect
        self.set_known_objects(['face'])
        # get new image and see what's the resolution for the coordonate to angle/speed converter
        self.get_new_image()
        self.logger.info("Using haar cascade: " + cascadedir)
        
    def train(self):
        pass

    def run(self):
        """start loop which gets a new image, then processes it"""

        im = None
        while im == None:
            im = self.vid_mem_reader.get_latest_image()
            if im == None:
                print "not receiving images yet..."
                time.sleep(0.2)

        #Wait for video source to be ready:
        #TODO: Shoud use vidmemreader, but this one never seem to return a resolution (at time of writing):
        #res = self.vid_mem_reader.get_resolution()
        print "image is"
        #TODO: This should work, but it doesn't because OpenCV keeps on complaining about that im is not a IPL image 
        #(while if you print it, it seems to be a IPL image).
        #print im
        #size = cv.GetSize(im)
        #print size
        self.res = ({'width':640, 'height':480})
        res = self.res

        self.transformer = util.speed_angle.SpeedAngle(None, res['width'], res['height'])
        
        while True:
            start_time = time.time()
            img = self.get_new_image()
            self.get_faces(img)
            self.update()
            time_spent = time.time() - start_time
            sleep_time = 1 / float(self.update_frequency) - time_spent
            self.logger.debug("Time: " + str(sleep_time))
            time.sleep(max(sleep_time, 0))
        
    def get_new_image(self):
        """ Get new image from video shared memory """
        return self.vid_mem_reader.get_latest_image()[0]

    def get_faces(self, img):
        """ Detect faces in image """
        results = self.detect_and_draw(img, False)
        (width, height) = cv.GetSize(img)
        for result in results:
            self.add_property('name', 'face')
            self.add_property('pixel_location', result[0])
            self.add_property('relative_location', (float(result[0][0]) / width, float(result[0][1]) / height))
            self.add_property('angle_and_distance', result[1])
            self.add_property('face_area', result[2])
            self.add_property('width', result[4])
            self.add_property('height', result[5])
            self.add_property('relative_face_area', (float(result[2]) / (width * height)))
            self.add_property('confidence', result[3])
            self.add_property('dimensions', (width, height))
            self.store_observation()
            
    def get_no_faces(self):
        return

    def detect_and_draw(self, img, videosettings):
        """
        This program is demonstration for face and object detection using haar-like features.
        The program finds faces in a camera image or video stream and displays a red box around them.

        Original C implementation by:  ?
        Python implementation by: Roman Stanchak, James Bowman
        
        Draws faces detected in img using cascade
        and returns list of tuples ((x, y, width, height), nNeighbours)"""
        if videosettings:
            haar_scale = 1.2
            min_neighbors = 2
            haar_flags = cv.CV_HAAR_DO_CANNY_PRUNING
        else:
            haar_scale = 1.1
            min_neighbors = 3
            haar_flags = 0
        min_size = (7, 7)
        image_scale = 2.4

        # allocate temporary images
        gray = cv.CreateImage((img.width,img.height), 8, 1)
        small_img = cv.CreateImage((cv.Round(img.width / image_scale),
                                   cv.Round (img.height / image_scale)), 8, 1)

        # convert color input image to grayscale
        cv.CvtColor(img, gray, cv.CV_BGR2GRAY)

        # scale input image for faster processing
        cv.Resize(gray, small_img, cv.CV_INTER_LINEAR)

        cv.EqualizeHist(small_img, small_img)

        if self.cascade:
            store = cv.CreateMemStorage(1024)
            faces = cv.HaarDetectObjects(small_img, self.cascade, store, haar_scale, min_neighbors, haar_flags, min_size)
            normfaces = []
            if faces:
                for ((x, y, w, h), n) in faces:
                    # the input to cv.HaarDetectObjects was resized, so scale the
                    # bounding box of each face and convert it to two CvPoints
                    pt1 = (int(x * image_scale), int(y * image_scale))
                    pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
                    cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)
                    #normfaces.append(((int(round(x*image_scale)), int(round(y*image_scale)), int(round(w*image_scale)), int(round(h*image_scale))),n))

                    center_x = (int(round(w*image_scale)/2) + int(round(x*image_scale)))
                    center_y = (int(round(h*image_scale)/2) + int(round(y*image_scale)))
                    angle_x = self.transformer.get_x_angle(center_x)
                    # hardcoded speed according to the size of the face in the image
                    speed_y = self.transformer.get_speed(w, h)
                    normfaces.append( ( (center_x, center_y), (speed_y, angle_x), (w*h), n, w, h) )
        if self.verbose:
            cv.ShowImage("Haar Detector", img)
            cv.WaitKey(10)
        return normfaces
    
    def get_predator_distance(self, bb, depth):
        self.logger.debug("Bounding Box: "+ str(bb))
        if bb[0] < 0:
            bb[0] = 0
        if bb[2] >= self.res['width']:
            bb[2] = self.res['width'] - 1
        if bb[1] < 0:
            bb[1] = 0
        if bb[3] >= self.res['height']:
            bb[3] = self.res['height'] - 1
        dist_rect = cv.CreateImage((bb[2]-bb[0],bb[3]-bb[1]), cv.IPL_DEPTH_8U, 1)
        dist_rect = cv.GetSubRect(depth, (bb[0], bb[1] , bb[2]-bb[0], bb[3]-bb[1]))
        return cv.Avg(dist_rect)[0]

def usage():
    print "You should add the configuration options on the command line."
    print ""
    print "Usage: ", sys.argv[0], " host=<controller_host> port=<controller_port> updatefreq=<update frequency>"
    print ""

if __name__ == "__main__":
    if len(sys.argv) < 3:
        usage()
        exit()

    sec = "haarface" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)

    update_frequency = config_dict.get_option(sec, "updatefreq")
    source = config_dict.get_option(sec, "video_source")
    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    use_cascade = config_dict.get_option(sec, "cascade", "haarcascade_frontalface_default")
    use_verbose = config_dict.get_option(sec, "verbose", "False")
    sect = config_dict.get_section(sec)
    print sect

    if not (update_frequency and controller_ip and controller_port):
        usage()
        exit()

    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)

    vision = HaarFace(controller_ip, controller_port, update_frequency, source=source, cascade=use_cascade, verbose=use_verbose)
    vision.connect()
    if (vision.is_connected()):
        vision.run()

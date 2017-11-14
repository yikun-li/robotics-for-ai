#!/usr/bin/python

import cv2.cv as cv
import sys
import time
import configparse
import logging
import util.vidmemreader

from grabbingutil.filter import TiltFilter, DummyFilter
from grabbingutil.tabledetector import TableDetector
from grabbingutil.segmentizer import HistogramSegmentizer, ConstantSegmentizer
from grabbingutil.config import config

from abstractvisionmodule import AbstractVisionModule

class TableModule(AbstractVisionModule):
    '''The table module keeps track of the table (or some elevated
    approximately flat surface with an enge) and writes the position and angle
    to memory. It was made with straight tables in mind, and might bork and
    go rampage when it encounters something... round. This module is not open
    minded at all! It should be taught something about respecting other types
    of tables. Ignorant module...'''
    
    def __init__(self, host, port, update_frequency, source="webcam", verbose=False, threshold=None, gradient=None):
        super(self.__class__, self).__init__(host, port)
        
        self.vid_mem_reader = util.vidmemreader.VidMemReader([source])
        depth = self.get_last_observation()
        while not depth.nChannels == 1:
            depth = self.get_last_observation()
            print "not depth file"
            
        
        self.res = {'width':depth.width,'height':depth.height}
        # filter, to filter out (compensate) the kinect image tilt
        self.filter = TiltFilter()
        
        # gradient is an empty image or an image of a flat surface from which
        # the TiltFilter can distill the differences in the image caused by
        # the tilt of the kinect camera.
        if not gradient:
            self.filter = DummyFilter() # well, if there is nothing to calibrate the filter on, don't filter.
        elif gradient == 'source':
            self.filter.calibrate(self.get_last_observation())
        else:
            self.filter.calibrate(cv.LoadImage(gradient, cv.CV_LOAD_IMAGE_GRAYSCALE))
        
        # Segmentizer: extracts only the pixels that contain the table from the image
        self.segmentizer = HistogramSegmentizer()
        
        # Detector: finds the distance and angle of the table edge
        self.detector = TableDetector()
        self.detector.verbose = verbose
        
        # Threshold: when should we drop a point, how far does it have to be
        # away from the fitted regression line to be called an outlier?
        if threshold:
            self.detector.outlier_threshold = int(threshold)
        
        self.update_frequency = update_frequency
        
        self.set_known_objects(['table'])

    def get_last_observation(self):
        '''Get last observation from the camera'''
        return self.vid_mem_reader.get_latest_image()[0]

    def observe(self):
        '''Grab the latest image, detect the table on it, and write the observation to memory'''
        temp = self.get_last_observation()
        if temp == None or not temp.nChannels == 1:
            return
        
        

        img = cv.CreateImage(cv.GetSize(temp), temp.depth , temp.nChannels);

        SE = cv.CreateStructuringElementEx(50, 50, 25, 25, cv.CV_SHAPE_RECT)
        cv.MorphologyEx(temp, img, temp, SE, cv.CV_MOP_CLOSE, iterations=1)
        # only look at the pixels that are interesting, just cut off the Nao and stuff.
        cv.SetImageROI(img, config["roi"])

        # remove tilt
        self.filter.filter(img)
        
        # cut the image into layers of interest
        layers = self.segmentizer.segmentize(img)
      
        # the HistogramSegmentizer can fail when no peaks are ever found
        if not layers:
            return
        
        # find arms and objects in the table layer of the image
        observation = self.detector.detect(layers['table'])
        
        if observation:
            # write the observation to memory
            self.emit(observation)
        
    def emit(self, observation):
        '''Writes the last observation to memory'''
        self.add_property("name", "table")
        
        # angle of the line that goes right-angled from the table to the robot.
        # if the table is exactly straight aligned in front of the robot it
        # should be zero, if it is turned a bit to the left (the edge of the
        # table on the left is closer to the robot than the edge on the right)
        # this value should be negative. It is in radiants. It is calculated
        # in detectors.py/TableDetector.Observation.angle.
        self.add_property("angle", observation.angle)
        
        # distance between the robot and the closest point to the table.
        self.add_property("distance", observation.distance)
        
        # internaly the tableedge is encoded as an y = ax + b formula. It is
        # quite easy to do magic with this, and one of the approachobject
        # behaviors does this, I think it was approachobject_52.py
        self.add_property("f_a", observation.a)
        self.add_property("f_b", observation.b)
        
        # distance in pixels isn't that useful if we just want to know how
        # safe it is to move forward. Easy to multiply with max_speed, and tada!
        self.add_property("relative_distance", observation.distance / self.res['height'])
        self.store_observation()

    def train(self):
        pass

    def run(self):
        '''Work your magic baby!'''
        while True:
            start_time = time.time()
            self.observe()
            self.update()
            time_spent = time.time() - start_time
            sleep_time = 1 / float(self.update_frequency) - time_spent
            time.sleep(max(sleep_time, 0))


def usage():
    print "You should add the configuration options on the command line."
    print ""
    print "Usage: ", sys.argv[0], " host=<controller_host> port=<controller_port> updatefreq=<update frequency> [source=<source>] [verbose=<True|False>] [threshold=50]"
    print ""

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print sys.argv
        usage()
        exit()

    sec = "tabler" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)

    update_frequency = config_dict.get_option(sec, "updatefreq")
    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    
    # when verbose is on, it shows the image it uses in a window
    use_verbose = config_dict.get_option(sec, "verbose", False)
    image_source = config_dict.get_option(sec, "source", "webcam")
    outlier_threshold = config_dict.get_option(sec, "threshold", None)
    
    # compensate the tilt of the kinect camera by subtracting an image of an empty surface.
    path_to_gradient = config_dict.get_option(sec, "gradient", config["gradient"])
    
    sect = config_dict.get_section(sec)
    print sect

    if not (update_frequency and controller_ip and controller_port):
        usage()
        exit()

    logging.getLogger('Borg.Brain').addHandler(logging.StreamHandler(sys.stdout))
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)

    tabler = TableModule(controller_ip, controller_port, update_frequency, source=image_source, verbose=use_verbose, threshold=outlier_threshold,gradient=path_to_gradient)
    tabler.connect()
    tabler.run()

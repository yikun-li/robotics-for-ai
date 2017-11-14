#!/usr/bin/python


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
from util.kinect_depth import convert_16to8

from util.ticker import Ticker
from abstractvisionmodule import AbstractVisionModule

logging.getLogger('Borg.Brain.Vision.SnapShotSaver').addHandler(util.nullhandler.NullHandler())

class SnapShotSaver(AbstractVisionModule):
    """
    Simple vision module, used to store images from specified video source.
    """

    def __init__(self, host, port, update_frequency, prefix = "", destination = None, source = "webcam", verbose = False):
        self.logger = logging.getLogger("Borg.Brain.Vision.SnapShotSaver")
        super(SnapShotSaver, self).__init__(host, port) 
        self.update_frequency = update_frequency
        self.__ticker = Ticker(frequency=update_frequency)
        self.verbose = verbose
        self.source = [source]
        if destination == None:
            raise Exception("Destination should be specified!")
        if not os.path.exists(destination):
            raise Exception("Specified destination does not exist!")
        self.destination = destination
        self.prefix = prefix
        self.vid_mem_reader = util.vidmemreader.VidMemReader(self.source)

    def train(self):
        pass

    def run(self):
        """start loop which gets a new image, then processes it"""
                
        count = 0
        while True:
            self.__ticker.tick()

            self.update()
            img = self.get_new_image()
            if img == None:
                print "not receiving images yet..."
            else:
                if self.verbose:
                    cv.ShowImage("SnapShotSaver", img)
                    cv.WaitKey(10)
                cv.SaveImage("%s/%s_%d.png" % (self.destination, self.prefix, count), img)
                count += 1

    def get_new_image(self):
        """ Get new image from video shared memory """
        img = self.vid_mem_reader.get_latest_image()
        if not img: 
            return None
        img = img[0]
        return convert_16to8(img)

def usage():
    print "You should add the configuration options on the command line."
    print ""
    print "Usage: ", sys.argv[0], " host=<controller_host> port=<controller_port> updatefreq=<update frequency>"
    print ""

if __name__ == "__main__":
    if len(sys.argv) < 3:
        usage()
        exit()

    sec = "snapshotsaver" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)

    update_frequency = config_dict.get_option(sec, "updatefreq")
    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    destination = config_dict.get_option(sec, "destination")
    video_source = config_dict.get_option(sec, "video_source")
    use_verbose = config_dict.get_option(sec, "verbose", "False")
    prefix = config_dict.get_option(sec, "prefix", "")
    sect = config_dict.get_section(sec)
    print sect

    if not (update_frequency and controller_ip and controller_port):
        usage()
        exit()

    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)

    vision = SnapShotSaver(controller_ip, controller_port, update_frequency, prefix=prefix, destination=destination,source=video_source, verbose=use_verbose)
    vision.connect()
    if (vision.is_connected()):
        vision.run()

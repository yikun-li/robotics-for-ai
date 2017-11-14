#!/usr/bin/python
import sys
import cv2.cv as cv
import os
import logging
import util.nullhandler
import util.loggingextra
import time
import datetime
import pickle

import configparse
import util.vidmemreader
from abstractvisionmodule import AbstractVisionModule



logging.getLogger('Borg.Brain.Vision.Observer').addHandler(util.nullhandler.NullHandler())

class Observer(AbstractVisionModule):
    """
    Class that segments objects and returns by default the location and distance to first object.
    It uses the Kinect to get the objects in an array that are ordered according to size, based on a threshold value.
    """

    def __init__(self, host, port, address, source = 'webcam'):
        """
        Observer Constructor
        @param host - hostname or ip of the destination to send data (controller ip)
        @param port - port on which to send data to destination (controller port)
        @param preview - set to true if you want to see the segmented object
        """
        self.logger = logging.getLogger("Borg.Brain.Vision.Observer")
        super(Observer, self).__init__(host, port)
        self.address = address    
        if source == 'webcam': 
            self.vmr = util.vidmemreader.VidMemReader(['webcam'])
        else:
            self.vmr = util.vidmemreader.VidMemReader(['kinect_depth', 'kinect_rgb'])
        self.command = False
        today = datetime.date.today()
        path = self.address + "/" + str(today) + '/'
        try:
            os.makedirs(path)
        except:
            pass
        self.statefile = None
        try:
            self.statefile = open(path + str(today) + '.sa', 'a')
            self.statefile.close()
        except:
            print "something wrong with making files"
            
    def train(self):
        pass

    def run(self):
        """Start loop which gets a new image, then processes it"""
        while True:         
            rgb = self.get_new_images()
            
            if self.command:
                self.store(rgb)
            self.update()
            
    

    def get_new_images(self):
        """ Gets new depth and rgb images from video shared memory """
        rgb = self.vmr.get_latest_image()[0]
        return rgb

    def store(self, rgb):
        
        today = datetime.date.today()
        path = self.address + "/" + str(today) + '/'
        try:
            os.makedirs(path)
        except:
            pass
        
        now = int(time.time())
        
        filename =  str(now) + '.png'
        extraIndex = 1
        
        if os.path.isfile(filename):
            temp = ""
            while os.path.isfile(filename):
                temp = filename + '-' + str(extraIndex)
                extraIndex += 1
            filename = temp
        cv.SaveImage(path + filename , rgb)
        self.add_property('name', 'observer')       
        self.add_property('picturename', filename)
        self.add_property('picturedir', path)
        self.add_property('observe', "True")
        self.command = False
        
    def stateaction_store(self, action, observation):
        today = datetime.date.today()
        path = self.address + "/" + str(today) + '/'
        statefile = open(path + str(today) + '.sa', 'a')
        
        object = None
        object = (action, observation)
        pickle.dump(object, statefile)
        
        statefile.close()
        

    def handle_custom_commands(self, entry):
        
        if entry['command'] == "take_observation":
            print "taking observation"
            self.command = True
            return True
        if entry['command'] == "store":
            print "storing"
            params = entry['params']
            action = params['action']
            observation = params['observation']
            self.stateaction_store(action, observation)
            
    def __del__(self):
        self.stop()


def usage():
    print "You should add the cbbonfiguration options on the command line.\n"
    print "Usage: ", sys.argv[0], " host=<controller_host> port=<controller_port> preview=<True>\n"

if __name__ == "__main__":
    if len(sys.argv) < 2:
        usage()
        exit()

    sec = "observer" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)

    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    address = config_dict.get_option(sec, "address")
    source = config_dict.get_option(sec, "video_source")
    if address == None:
        address = os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/data/')
        
    if source == None:
        source = "webcam"
    sect = config_dict.get_section(sec)
    print sect

    if not (controller_ip and controller_port):
        usage()
        exit()

    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)

    segmenter = Observer(controller_ip, controller_port, address, source)
    segmenter.connect()
    segmenter.set_socket_verbose(True, True)
    if (segmenter.is_connected()):
        segmenter.run()

#!/usr/bin/python

import pygame
import time
import pickle
import sys

import glob 
import os 

import cv2.cv as cv

import os
import logging
import util.nullhandler
import util.loggingextra

import random

import configparse
import util.vidmemreader
import util.speed_angle
from abstractvisionmodule import AbstractVisionModule
import SocketServer
import shutil
import time
logging.getLogger('Borg.Brain.Vision.BlobDetector').addHandler(util.nullhandler.NullHandler())

import vision.colorblob.colorblob as colorblob
import vision.colorblob.colorlist as colorlist
import vision.colorblob.gui as gui


class BlobDetector(AbstractVisionModule):

    def __init__(self, host, port, blob_dir, source="webcam", do_preview=False, do_train = None):
    
        self.logger = logging.getLogger("Borg.Brain.Vision.BlobDetector")
        super(BlobDetector, self).__init__(host, port)

        self.__preview = do_preview
        #TODO; not sure about this part:
        self.__vmr = util.vidmemreader.VidMemReader([source])
        self.__blob_dir = blob_dir
        self.__colorlist_list = colorlist.load_color_file_list(self.__blob_dir)

        self.__display_surface = None
        self.__do_train = not do_train == None
        self.__last_surface = None

        if self.__do_train:
            print "In training procedure..."
            self.__gui = gui.Gui(self.__colorlist_list)


    def get_colorlist(self, filename):
        for colorlist in self.__colorlist_list:
            if colorlist.get_file_basename() == filename:
                return colorlist
        return None


    def handle_custom_commands(self, entry):
        if entry['command'] == 'train':
            box = entry['params']['box']  
            sample_count = entry['params']['sample_count']  
            name = entry['params']['name']  
            if not self.__last_surface == None:
                colorlist = self.get_colorlist(name)
                if colorlist == None:
                    raise Exception("Specified colorlist with filename %s not found!" % name)
                colorlist.empty_list()
                for i in range(sample_count):
                    x_range = box[1][0] - box[0][0]
                    y_range = box[1][1] - box[0][1]
                    x = int(random.random() * x_range) + box[1][0]
                    y = int(random.random() * y_range) + (box[1][1] - y_range)

                    #Only add if pixel location is valid:
                    if x > self.__last_surface.get_width() or x < 0:
                        continue
                    if y > self.__last_surface.get_height() or y < 0:
                        continue
                    colorlist.add_color(self.__last_surface.get_at((x, y)))
                colorlist.filter_outliers()
                colorlist.save()
                

    def train(self):
        #We do not use this
        pass

        
    def run(self):
        """Start loop which gets a new image, then processes it"""
        while True:         

            cv_im = self.get_new_image()[0]
            surface = colorblob.convert_cv_to_pg(cv_im)
            self.__last_surface = surface

            if self.__do_train: 
                self.__gui.run_iterate(surface)

            for colorlist in self.__colorlist_list:
                blob = colorblob.ColorBlob(colorlist)
                blob.detect(surface)

                x = blob.get_centroid()[0]
                y = blob.get_centroid()[1]
                size = blob.get_size()
                name = colorlist.get_file_basename()

                if size > 0:
                    self.add_property('name', name)       
                    self.add_property('x', x)
                    self.add_property('y', y)
                    self.add_property('size', size)
                    self.store_observation()

            self.update()
            

    def get_new_image(self):
        #TODO: Get image from vmr:
        return self.__vmr.get_latest_image()


def usage():
    print "You should add the cbbonfiguration options on the command line.\n"
    print "Usage: ", sys.argv[0], " host=<controller_host> port=<controller_port> preview=<True>\n"

if __name__ == "__main__":
    if len(sys.argv) < 2:
        usage()
        exit()

    sec = "blobdetector" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)

    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    do_preview = config_dict.get_option(sec, "preview")
    video_source = config_dict.get_option(sec, "video_source")
    do_train = config_dict.get_option(sec, "train")
    blob_dir = config_dict.get_option(sec, "blob_dir")
    if do_preview != None and do_preview != 'True':
        do_preview = None
    if do_train != None and do_train != 'True':
        do_train = None

    sect = config_dict.get_section(sec)
    print sect

    if (not (controller_ip and controller_port)) or blob_dir == None:
        usage()
        exit()

    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)

    detector = BlobDetector(controller_ip, controller_port, blob_dir, source = video_source, do_train = do_train, do_preview = do_preview)
    detector.connect()
    detector.set_socket_verbose(True, True)
    if (detector.is_connected()):
        detector.run()


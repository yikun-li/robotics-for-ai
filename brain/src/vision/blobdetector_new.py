#!/usr/bin/python
from __future__ import division

import pygame
import time
import pickle
import sys
import numpy
import copy

import glob
import os

import cv2.cv as cv
import cv2

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

#import vision.colorblob.colorblob as colorblob
import vision.colorblob.colorblob_new as colorblob_new
import vision.colorblob.colorlist as colorlist
import vision.colorblob.gui as gui

from util.ticker import Ticker


class BlobDetector(AbstractVisionModule):

    def __init__(self, host, port, blob_dir, source="webcam", tbd=None, trainable=False, denoise_value = 5, frequency = 10):
        self.logger = logging.getLogger("Borg.Brain.Vision.BlobDetector")
        super(BlobDetector, self).__init__(host, port)

        self.ticker = Ticker(frequency=frequency, name="BlobDetector", ignore_delay=True)

        #TODO; not sure about this part:
        self.__vmr = util.vidmemreader.VidMemReader([source])
        self.__blob_dir = blob_dir
        #self.__colorlist_list = colorlist.load_color_file_list(self.__blob_dir)

        #self.__display_surface = None
        #self.__last_surface = None
        if not tbd:
            print "Using default empty boundaries."
            self.to_be_detected = [("default",[])]
        else:
            input_data = open(tbd)
            self.to_be_detected = pickle.load(input_data)
            input_data.close()
        print trainable
        if trainable:
            self._trainable = True
            self.add_from_next_image = False
            self.add_x = None
            self.add_y = None
        else:
            self._trainable = False
        print "Number of target colors: " + str(len(self.to_be_detected))
        self.current_target = 0

        self.denoise_value = int(denoise_value)

        # Variables to tune the size of the colorcubes added by clicking:
        self.cube_size = 5 # height/width of the square area used to sample pixels.
        self.increase_h = 1 # How much the hue of the cube is widened on each side.
        self.increase_s = 5 # How much the saturation of the cube is widened on each side.
        self.increase_v = 5 # How much the value of the cube is widened on each side.

        #TODO DEBUG VALUES!
        self.most_recent_button = "Startup"


    def run(self):
        """Start loop which gets a new image, then processes it"""

        # Create display window.
        cv2.namedWindow("Blob Detector")
        cv2.moveWindow("Blob Detector",100,100)
        if self._trainable:
            print "Using mouse clicking."
            cv.SetMouseCallback("Blob Detector", self.onmouse)
        else:
            print "Not using mouse clicking."
        while True:

            self.ticker.tick(True)
            # Get the image
            cv_im = self.get_new_image()[0]
            if cv_im:

                # Pre-process the image
                HSV_image = self.preprocess_image(cv_im)

                # Create a copy image on which found contours are shown.
                self.draw_image = copy.deepcopy(HSV_image)

                if self._trainable and self.add_from_next_image:
                    # Try to add the area specified by the most recent click.
                    self.add_new_cube(HSV_image)

                self.detection_process(HSV_image)

                self.image_display(self.draw_image)

            else:
                # If no image is recieved, nothing can be done.
                print "No image recieved!"
                print "-------------------------------"

            self.update()

    def train(self):
        #We do not use this
        pass

    def get_new_image(self):
        #TODO: Get image from vmr:
        return self.__vmr.get_latest_image()

    def onmouse(self, event, x, y, flags, param):

        #print "Flags: " + str(flags)
        if not ( flags == cv.CV_EVENT_FLAG_CTRLKEY or flags == cv2.EVENT_FLAG_SHIFTKEY or flags == cv2.EVENT_FLAG_ALTKEY):
            self.handle_nokey_click(event, x, y)
            self.most_recent_button = "None"
            return

        if flags == cv.CV_EVENT_FLAG_ALTKEY:
            # Hold Alt to store/delete trained data
            self.handle_alt_click(event)
            self.most_recent_button = "Alt"
            return

        if flags == cv.CV_EVENT_FLAG_SHIFTKEY:
            # Hold Shift to increase HSV width.
            self.handle_shift_click(event)
            self.most_recent_button = "Shift"
            return

        if flags == cv.CV_EVENT_FLAG_CTRLKEY:
            # Hold Ctrl to decrease HSV width.
            self.handle_ctrl_click(event)
            self.most_recent_button = "Ctrl"
            return

        #print "No usable event, so doing nothing with the click."
        return

    def handle_custom_commands(self, entry):
        """This function will handle commands to add cubes from a behavior"""
        print entry
        if self._trainable and entry['command'] == 'train':
            print "Trying to train by clicking."
            # Get the target points and image
            points = entry['params']
            image = self.get_new_image()[0]
            processed_image = self.preprocess_image(image)
            for point in points:
                # Set the internal variables
                self.add_x = point['x']
                self.add_y = point['y']
                print "Clicking on: " + str(point['x']) + ", " + str(point['y'])

                # Add the cube
                self.add_new_cube(processed_image)

        # Then save the additions
        self.handle_alt_click(None)

############ Subfunctions

######## Image processing

    def preprocess_image(self, image):
        """This function converts the image to a Numpy array and transfers it to HSV space."""

        # Convert the image to a Numpy array.
        arr = numpy.asarray(image[:,:])
        #cv2.imshow("Blob Detector", arr)
        #print cv_im
        #print arr

        # Convert to HSV
        return cv2.cvtColor(arr,cv2.COLOR_BGR2HSV)

    def add_new_cube(self, image):
        """"This function takes care of adding a new colorcube to the range of detection cubes."""
        # Try to add the area specified by the most recent click.

        # Determine the area to use.
        # This is a square area of self.cube_size pixels.
        area_to_be_added = cv2.getRectSubPix(image, (self.cube_size,self.cube_size), (float(self.add_x), float(self.add_y)))

        # Determine the min and max values in that area.
        # Split the image
        (hue, saturation, value) = cv2.split(area_to_be_added)
        # Determine the min and max in the area.
        # These min and max are increased/decreased to improve detection.
        hue_max = numpy.amin([180, numpy.amax(hue) + self.increase_h])
        hue_min = numpy.amax([0, numpy.amin(hue) - self.increase_h])
        print hue
        if (hue_max - hue_min) > 100:
            #Ok, probably it is some kind of a red color (around 0):
            #So we need to determin the min and max around 0:
            #TODO: Make more efficient using numpy or so:
            hue_min = 180
            hue_max = 0
            for hue_val_line in hue:
                for hue_val in hue_val_line:
                    print hue_val
                    if hue_val > 90:
                        if hue_val < hue_min:
                            hue_min = hue_val
                    if hue_val <= 90:
                        if hue_val > hue_max:
                            hue_max = hue_val

        saturation_max = numpy.amin([255, numpy.amax(saturation) + self.increase_s])
        saturation_min = numpy.amax([0, numpy.amin(saturation) - self.increase_s])

        value_max = numpy.amin([255, numpy.amax(value) + self.increase_v])
        value_min = numpy.amax([0, numpy.amin(value) - self.increase_v])

        # Add these to a dict.
        new_cube = {'h_lower':hue_min, 'h_upper':hue_max, 's_lower':saturation_min, 's_upper':saturation_max, 'v_lower':value_min, 'v_upper': value_max}

        print "Made new dict."
        print new_cube

        # Add the dict to the detection dicts.
        ((self.to_be_detected[self.current_target])[1]).append(copy.deepcopy(new_cube))

        # And it's done!
        self.add_from_next_image = False

    def store_contour(self, contour, tag):
        """"This function will store a found image contour in the central memory."""

        # Determine the bounding box
        x, y, w, h = cv2.boundingRect(contour)

        # Determine the area
        area = cv2.contourArea(contour)

        # Then, store that in memory.
        self.add_property('name', tag)
        self.add_property('x', x)
        self.add_property('width', w)
        self.add_property('y', y)
        self.add_property('height', h)
        self.add_property('surface', area)
        self.store_observation()

    def store_combined_contours(self, contours, tag):
        """This will store the combined contours in a list sorted by area."""
        contour_list = []
        # Determine the dict values for each contour.
        for contour in contours:
            # Determine x, y, w and h
            x, y, w, h = cv2.boundingRect(contour)
            # And the area.
            area = cv2.contourArea(contour)

            # Put this data into a dict
            contour_dict = {'x': x, 'width': w, 'y': y, 'height': h, 'surface': area}
            # And add it to the list.
            contour_list.append(copy.deepcopy(contour_dict))

        if len(contour_list) <1:
            return
        sorted_contours = sorted(contour_list, key=lambda k: k['surface'], reverse=True)
        new_tag = "combined_" + str(tag)
        self.add_property('name', new_tag)
        self.add_property("sorted_contours", copy.deepcopy(sorted_contours))
        self.store_observation()

    def detection_process(self, image):
        """"This function runs the detection process for the recieved image."""
        # Now, go over the various colors to be detected.
        for target in self.to_be_detected:
            # Detect any blobs
            #print target
            tag, cubes = target
            found = colorblob_new.process_HSV(image, cubes, self.denoise_value)
            collection = []

            if not found:
                # No contours found.
                pass
            else:
                #Determine HSV color of contours:
                h = 0
                target_1 = cubes[0]
                if target_1['h_upper'] < target_1['h_lower']:
                    h = target_1['h_lower'] + (((180 - target_1['h_lower']) + target_1['h_upper']) / 2)
                    if h > 180:
                        h -= 180
                else:
                    h = target_1['h_lower'] + ((target_1['h_upper'] - target_1['h_lower']) / 2)

                # Draw the contours into the draw_image
                cv2.drawContours(self.draw_image, found, -1, (h, 150, 150), thickness = -1)

                for contour in found:
                    self.store_contour(contour, tag)
                    collection.append(contour)
            self.store_combined_contours(collection, tag)

    def image_display_line(self, image, position, text):
        cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0), lineType=cv2.CV_AA)
        cv2.putText(image, text, (position[0] - 1, position[1] - 1), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), lineType=cv2.CV_AA)

    def image_display_line_list(self, image, position, line_list):
        line_left_offset = position[0]
        line_height = 10
        line_y_position = position[1] + line_height
        for line in line_list:
            self.image_display_line(image, (line_left_offset, line_y_position), line)
            line_y_position += line_height


    def image_display(self, image):
        """"This will display the image with the found contours and relevant text on it."""
        # Convert to BGR for display.
        disp_image = cv2.cvtColor(image,cv2.COLOR_HSV2BGR)
        if self._trainable:
            line_list = []

            # Display the colour clicks will be added to.
            line_list.append("Target: " + str((self.to_be_detected[self.current_target])[0]))
            # And the area used when clicking to add.
            line_list.append("Area: " + str(self.cube_size) + " x " + str(self.cube_size) + " px")
            # And the degree to which the H, S and V values found by clicking are widened.
            line_list.append("HSV wideing values: " + str(self.increase_h) + " " + str(self.increase_s) + " " + str(self.increase_v))
            #TODO DEBUG CODE!
            line_list.append(self.most_recent_button)

            self.image_display_line_list(disp_image, (3, 0), line_list)
        # Display the actual image.
        cv2.imshow("Blob Detector", disp_image)
        cv2.waitKey(10)

######## Mouse input

    def handle_ctrl_click(self, event):
        """This will handle a ctrl-click when training."""
        print "Control pressed."
        if event == cv.CV_EVENT_LBUTTONDOWN:
            # Click left to decrease hue
            print "Narrowing hue edges of color cube"
            self.increase_h -= 1
            if self.increase_h < 0:
                self.increase_h = 180-self.increase_h
            return
        if event == cv.CV_EVENT_MBUTTONDOWN:
            # Click middle to decrease saturation
            print "Narrowing saturation edges of color cube"
            self.increase_s -= 5
            if self.increase_s < 0:
                self.increase_s = 0
            return
        if event == cv.CV_EVENT_RBUTTONDOWN:
            # Click right to decrease value
            print "Narrowing value edges of color cube"
            self.increase_v -= 5
            if self.increase_v < 0:
                self.increase_v = 0
            return

    def handle_alt_click(self, event):
        """This will handle a alt-click when training."""
        print "Alt pressed."
        # Save all added data
        #if (event == cv.CV_EVENT_LBUTTONDOWN):
        print "Saving added data."
        output=open(tbd, 'wb')
        pickle.dump(self.to_be_detected, output)
        output.close()
        return
        # Click right to remove last added cube
        #if (event == cv.CV_EVENT_LBUTTONDOWN) and tbd:
            #TODO Not working ATM.
            #return

    def handle_shift_click(self, event):
        """This will handle a shift-click when training."""
        print "Shift pressed."
        if event == cv.CV_EVENT_LBUTTONDOWN:
            # Click left to increase hue
            print "Widening hue edges of color cube"
            self.increase_h += 1
            if self.increase_h > 180:
                self.increase_h = 0
            return
        if event == cv.CV_EVENT_MBUTTONDOWN:
            # Click middle to increase saturation
            print "Widening saturation edges of color cube"
            self.increase_s += 5
            if self.increase_s > 255:
                self.increase_s = 0
            return
        if event == cv.CV_EVENT_RBUTTONDOWN:
            # Click right to increase value
            print "Widening value edges of color cube"
            self.increase_v += 5
            if self.increase_v > 255:
                self.increase_v = 0
            return

    def handle_nokey_click(self, event, x, y):
        """This will handle a keyless click when training."""
        #print "No CTRL, ALT or SHIFT pressed!"

        # Will add a blob to the current color and save it on a left-click.
        if event == cv.CV_EVENT_LBUTTONDOWN:
            print "Adding pixel cube for current click to the color."
            self.add_from_next_image = True
            self.add_x = copy.copy(x)
            self.add_y = copy.copy(y)
            # This is checked for in the processing if trainable is True.
            return
        # Will cycle through the colors on a right-click.
        if event == cv.CV_EVENT_RBUTTONDOWN:
            print "Switching colors."
            self.current_target += 1
            if self.current_target > len(self.to_be_detected) -1:
                self.current_target = 0
            return
        if (event == cv.CV_EVENT_MBUTTONDOWN) and tbd:
            #TODO: switch this to cycling through selection square sizes.
            print "Cycling cube size."
            self.cube_size += 1
            if self.cube_size > 10:
                self.cube_size = 1

############ Administrative functions

def usage():
    print "You should add the configuration options on the command line.\n"
    print "Usage: ", sys.argv[0], " host=<controller_host> port=<controller_port>\n"


if __name__ == "__main__":
    if len(sys.argv) < 2:
        usage()
        exit()

    sec = "blobdetector" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)

    print config_dict

    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    video_source = config_dict.get_option(sec, "video_source")
    blob_dir = config_dict.get_option(sec, "blob_dir")
    tbd = config_dict.get_option(sec, "targets")
    trainable = config_dict.get_option(sec, "trainable")
    denoise_value = config_dict.get_option(sec, "denoise_value")
    frequency = config_dict.get_option(sec, "frequency")

    print "To be detected: " + str(tbd)

    controller_port = config_dict.get_option(sec, "port")

    sect = config_dict.get_section(sec)
    print sect

    if (not (controller_ip and controller_port)):
        usage()
        exit()

    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)

    detector = BlobDetector(controller_ip, controller_port, blob_dir, source = video_source, \
            tbd = tbd, trainable = trainable, denoise_value = denoise_value, frequency = frequency)
    detector.connect()
    detector.set_socket_verbose(True, True)
    if (detector.is_connected()):
        detector.run()


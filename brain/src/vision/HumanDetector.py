#!/usr/bin/env python
import cv2.cv as cv
import numpy as np
import cv2
import sys
import time
import math
import logging

import util.nullhandler
import util.loggingextra
import util.vidmemreader

from util.ticker import Ticker

from abstractvisionmodule import AbstractVisionModule
import configparse

import roslib
roslib.load_manifest('borg_pioneer')
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

logging.getLogger('Borg.Brain.Vision.HumanDetector').addHandler(util.nullhandler.NullHandler())

class HumanDetector(AbstractVisionModule):

    def __init__(self, host, port, freq, source="camera/depth/image_raw", transform=[[0, 0, 0.63]]):
        super(HumanDetector, self).__init__(host, port)
        self.logger = logging.getLogger("Borg.Brain.Vision.HumanDetector")
        print "Human detection running"
        self.bridge = CvBridge()
        self.cv_image = None
        self.FPS_counter = 0
        self.FPS_time = time.time()
        self.camera_to_base_transform = transform
        self.source = source
        self.freq = freq

        #Parameters
        self.__body_line            = 120
        self.__leg_line             = 320
        self.__line_height          = 10
        self.__max_depth            = 2000
        self.__depth_search_limit   = 1000
        self.__depth_step           = 500
        self.__min_width_body       = 60
        self.__max_width_body       = 250
        self.__min_width_leg        = 15
        self.__max_width_leg        = 100
        self.__search_depth_leg     = 500
        
        self.__hori_fov = 57.0      # Horizontal field of view 
        self.__vert_fov = 43.0      # Vertical
        self.__hori_res = 640.0     # Horizontal resolution
        self.__vert_res = 480.0     # Vertical

        #self.vid_mem_reader = util.vidmemreader.VidMemReader([self.source])
        self.Operators = []
        rospy.Subscriber(source, Image, self.newImageCB)
        print "Subscribing to %s" % source

    def train(self):
        pass

    def run(self):
        """Start loop which simple sends a random fake observation every 5 secondes."""
        t = Ticker(self.freq)
        while True:         
            t.tick()
            #Name, used to identify the observation in memory:
            Operators = self.Operators
            if Operators:
                self.add_property('name', 'HumanDetector')
                self.add_property('Operators', Operators)
                self.store_observation()

            self.update()

    def computeFPS(self):
        self.FPS_counter += 1
        if (time.time() - self.FPS_time > 1):
            print "FPS: ", self.FPS_counter
            self.FPS_time = time.time()
            self.FPS_counter = 0

 
    #Filter depth in the image
    def filterDepth(self, depth_image, minimum, maximum):
        depth_image[depth_image > maximum] = 0
        depth_image[depth_image < minimum] = 0
        depth_image[depth_image > minimum] = 50000
        return depth_image


    #Select image line from image
    def selectLine(self,image,line,height):
        return image[line:line+height, 0:640]


    #Remove noise from the image.
    #If there are 1 or more zeros in one column -> Entire column is set to zero 
    def removeNoise(self,image):
        image[:, sum(image == 0) >= 1] = 0        
        return image


    #Remove blobs from the list
    def removeBlobsFromList(self, Blobs, key, comparison,  value):
        if not Blobs == None:
            if comparison == '<':
                return [x for x in Blobs if x[str(key)] > value]
            elif comparison == '>':
                return [x for x in Blobs if x[str(key)] < value]
            else:
                print "Wrong comparison"    
        else:
            return []

    #Compute the depth value of the blob. Remove outliers
    def computeDepth(self, image_depth, x, size):
        temp = np.copy(image_depth[0:10, x:x+size])
        return np.mean(temp[abs(temp - np.mean(temp)) < np.std(temp)])


    #Sort the list with blobs
    def sortBlobs(self, blobs):
        return sorted(blobs, key=lambda k: k['width'], reverse=True) 


    #Detect blobs in the image
    def detectBlobs(self, image, image_depth):
        detections = np.argwhere(image[0,:])
        if np.size(detections) > 5:
            blobs = []
            start_x = detections[0]
            for i in range(0, np.size(detections) - 5):
                if detections[i+1] - detections[i] > 2:
                    blobs.append({'x' : int(start_x), 'width' : int(detections[i] - start_x), 'depth' : self.computeDepth(image_depth, int(start_x),int(detections[i] - start_x))})
                    start_x = detections[i+1]
                    
            blobs.append({'x' : int(start_x), 'width' : int(detections[i+1] - start_x) , 'depth' : self.computeDepth(image_depth, int(start_x),int(detections[i+1] - start_x))})
            
            return self.sortBlobs(blobs)
        return None
        
        #Return a list with the blobs


    def detectOperator(self, body, legs):
        if len(legs) >= 2:
            legs = self.sortBlobs(legs)
            if abs(legs[0]['width'] - legs[1]['width']) < 20:
                return True
            else:
                #print "Legs are different"
                return False

        else:
            #print "Not enough possible legs detected"
            return False

    def newImageCB(self, data):
        self.computeFPS()
        Operators = []
        #Create an array with depth information from the image
        temp = self.bridge.imgmsg_to_cv(data)
        image_temp = np.asarray(temp[:,:])

        depth = 500

        while depth < self.__max_depth:
            ##########################
            # Body Detection         #
            ##########################
            #Make a copy of the original image
            image = np.copy(image_temp)
            #Select a line at x = self.__body_line with a height of self.__line_height
            image_body = self.selectLine(image,self.__body_line,self.__line_height)
            #Create an array with the depth values
            image_body_depth = np.copy(image_body)
            #Filter on depth
            image_body = self.filterDepth(image_body,depth,depth + self.__depth_search_limit)
            #Remove noise from the image
            image_body = self.removeNoise(image_body)
            #Detect blobs in image
            body_blobs = self.detectBlobs(image_body,image_body_depth)

            #Remove small blobs
            body_blobs = self.removeBlobsFromList(body_blobs, 'width', '<', self.__min_width_body)

            if depth >= 4000:
                body_blobs = self.removeBlobsFromList(body_blobs, 'width', '>', 150)
            else:
                body_blobs = self.removeBlobsFromList(body_blobs, 'width', '>', self.__max_width_body)

            #If there are possible bodies detected
            if not body_blobs == None:
                #Make a copy of the original image
                #image = np.copy(image_temp)
                #Select a line for detecting legs
                image_legs = self.selectLine(image,self.__leg_line,self.__line_height)
                #Create an array with the depth values
                image_legs_depth = np.copy(image_legs)

                for body_blob in body_blobs:
                    #Minimum and maximum depth values for detecting legs
                    (min_depth, max_depth) = (body_blob['depth'] - self.__search_depth_leg, body_blob['depth'] + self.__search_depth_leg)
                    #Minimum and maximum position of legs
                    (min_leg_pos, max_leg_pos) = (body_blob['x'] - 50, body_blob['x'] + body_blob['width'] + 50)
                    #Filter on depth
                    image_legs = self.filterDepth(image_legs, min_depth, max_depth)
                    #Remove noise
                    image_legs = self.removeNoise(image_legs)

                    #Detect blobs in image
                    leg_blobs = self.detectBlobs(image_legs, image_legs_depth)

                    leg_blobs = self.removeBlobsFromList(leg_blobs, 'x', '<', min_leg_pos)
                    leg_blobs = self.removeBlobsFromList(leg_blobs, 'x', '>', max_leg_pos)
                    leg_blobs = self.removeBlobsFromList(leg_blobs, 'width', '<', self.__min_width_leg)
                    leg_blobs = self.removeBlobsFromList(leg_blobs, 'width', '>', self.__max_width_leg)

                    if self.detectOperator(body_blob, leg_blobs):
                        Operator = {'x' : (body_blob['x'] + (0.5 * body_blob['width'])), 'depth' : body_blob['depth'], 'width' : body_blob['width'], 'time': time.time()}
                        Match = False
                        
                        #Convert x, y values to real-world x, y values (in meters)
                        depth_dist = body_blob['depth'] / 1000

                        #Angle difference per pixel
                        hori_pp = self.__hori_fov / self.__hori_res
                        vert_pp = self.__vert_fov / self.__vert_res
                        
                        #Locate x
                        x_from_center = self.__hori_res / 2.0 - body_blob['x']
                        x_alpha = (x_from_center * hori_pp) / 180.0 * 3.14195
                        x_actual = depth_dist * math.sin(x_alpha)
                        
                        #Locate y
                        y_from_center = self.__vert_res / 2.0 - self.__body_line
                        y_alpha = y_from_center * vert_pp / 180.0 * 3.14195
                        y_actual = depth_dist * math.sin(y_alpha)
                        
                        #Transform to base_link
                        Operator['y_actual'] = x_actual + self.camera_to_base_transform[0][1]
                        Operator['z_actual'] = y_actual + self.camera_to_base_transform[0][2]
                        Operator['x_actual'] = math.sqrt(Operator['y_actual']*Operator['y_actual'] + depth_dist*depth_dist)
                        
                        for i in range(0, len(Operators)):
                            if abs(Operators[i]['x'] - (body_blob['x'] + (0.5 * body_blob['width']))) < 30:     
                                Match = True

                        if not Match:
                            Operators.append(Operator)

            depth += self.__depth_step

        #image_temp[self.__body_line:(self.__body_line + self.__line_height),0:640] = 10000
        #image_temp[self.__leg_line:(self.__leg_line + self.__line_height),0:640] = 10000
        #cv2.imshow("Human detector 1.0", image_temp)
        #cv2.waitKey(3)

        self.Operators = Operators
               

def main(args):
    sec = "humandetector"
    args = sys.argv[1:]
    option_dict = configparse.parse_args_to_param_dict(args, sec)

    rospy.init_node('HumanDetector')
    #READ PARAMETERS:
    host   = option_dict.get_option(sec,'host')
    port   = option_dict.get_option(sec,'port')
    source = option_dict.get_option(sec,'video_source')
    freq   = option_dict.get_option(sec,'update_frequency')

    # Transform is fixed
    transform = [[0, 0, 0.63]]
    detector = HumanDetector(host, port, freq, source, transform)
    detector.connect()
    detector.run()
    
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


import numpy
import math
import os
import cv
import time
import sys
import logging
import util.nullhandler

import vision.surfdetect
from abstractvisionmodule import AbstractVisionModule as avm
import util.vidmemreader
import configparse

logging.getLogger('Borg.Brain.Vision.ObjectDetector_AVM').addHandler(util.nullhandler.NullHandler())

class ObjectDetector(avm):
    '''
    classdocs
    '''

    def __init__(self, controller_ip, controller_port, video_source, update_frequency, training_dirs, min_matches, visualize, vision_window, detection_window):
        '''
        Constructor
        '''
        self.logger = logging.getLogger("Borg.Brain.Vision.ObjectDetector_AVM")
        super(ObjectDetector, self).__init__(controller_ip, controller_port)
        ### PARAMETERS: ###
        self.update_frequency = update_frequency

        ### INITIALIZERS: ###
        self.vidmemreader = util.vidmemreader.VidMemReader([video_source])
        self.surf = vision.surfdetect.SurfDetect(update_frequency, None, video_source, vision_window_name=vision_window, detection_window_name=detection_window)
        #Object with the update() function to be called (TODO: Cleanup this code):
        self.surf.update_object = self
        self.surf.set_destination_name('objectDetector')
        self.min_matches = min_matches
        self.visualize = visualize
        self.training_dirs = training_dirs

    def train(self):
        ### TRAIN SYSTEM: ###
        print '\nSTARTING TRAINING PROCEDURE: ObDet\n'
        time_1 = time.time()
        dirs = self.training_dirs.split(':')
        for dir in dirs:
            print 'TRAINING ON: ' + os.environ['BORG'] + dir
            self.surf.train_system(os.environ['BORG'] + dir)
            print 'TRAINED: ', dir[19:]
        time_2 = time.time()
        print '\nTRAINING PROCEDURE FINISHED: ObDet'
        print '\t' + str(time_2-time_1) + ' seconds'
        self.add_property('name', 'training_finished')
        self.store_observation()

    def run(self):
        """start loop which gets a new image, then processes it"""
        while self.is_connected():
            start_time = time.time()
            ########## UPDATE ##########
            try: #This block avoids crashing if vidmemreader fails to read last_image_index file
                detectionsList = []
                images_list = self.vidmemreader.get_latest_image()
                for image in images_list:
                    detectionsList = self.surf.process_image(image, min_matches=self.min_matches, visualize=self.visualize)
                    #If there is any detection:
                    if detectionsList:
                        detectionsList = self.process_detection(image, detectionsList)
                        for dic in detectionsList:
                            self.add_property('name', dic['name'])
                            self.add_property('time', dic['time'])
                            for key, value in dic['property_dict'].iteritems():
                                self.add_property(key, value)
                            self.store_observation()
            except:
                raise
                pass

            ############################
            # Send data
            self.update()

            #Sleep left time if any:
            time_spent = time.time() - start_time
            sleep_time = 1 / float(self.update_frequency) - time_spent
            time.sleep( max(sleep_time,0) )

    def process_detection(self, image, detectionsList):
        tempList = detectionsList
        for object in range(len(tempList)):
            if 'pixel_location' in tempList[object]['property_dict'].keys():
                xSize = (cv.GetSize(image)[0]) / 2
                ySize = (cv.GetSize(image)[1]) / 2
                hypot = math.hypot(xSize, ySize)
                #29 according to documentation but much better performance with 25 :)
                #angle_h = math.radians(25)
                #Angle calculated for the logitech HD-c510 (aprox 48.45 degrees on the horizontal):
                angle_h = math.radians(27.307594906640663)
                focalDist = hypot / math.tan(angle_h)
                xPixel = tempList[object]['property_dict']['pixel_location'][0]
                yPixel = tempList[object]['property_dict']['pixel_location'][1]
                #Angles in RADIANS:
                angle_x = math.atan2(xPixel-xSize, focalDist)
                angle_y = math.atan2(yPixel-ySize, focalDist)
                tempList[object]['property_dict']['image_x'] = angle_x
                tempList[object]['property_dict']['image_y'] = angle_y

                tempList[object]['property_dict']['image_size'] = cv.GetSize(image)

                #Calculate covariance matrix of transformed keypoints:
                transKeypoints = tempList[object]['property_dict']['transKeypoints']
                cov = numpy.cov(transKeypoints, rowvar=0)
                tempList[object]['property_dict']['cov_x']  = cov[0][0]
                tempList[object]['property_dict']['cov_y']  = cov[1][1]
                tempList[object]['property_dict']['cov_xy'] = cov[0][1]

                #Add object corners:
                corners = tempList[object]['property_dict']['corners']
                tempList[object]['property_dict']['corner_BL'] = corners[0]
                tempList[object]['property_dict']['corner_BR'] = corners[1]
                tempList[object]['property_dict']['corner_TR'] = corners[2]
                tempList[object]['property_dict']['corner_TL'] = corners[3]

                #Remove keypoints before sending detections on TC/IP to reduce file size:
                del(tempList[object]['property_dict']['transKeypoints'])

        return tempList

if __name__ == "__main__":
    print 'OPENED: ObDet --'
    logging.getLogger('Borg.Brain').addHandler(logging.StreamHandler(sys.stdout))
    #logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)
    logging.getLogger('Borg.Brain').setLevel(logging.INFO)

    #PARSE CONFIG FILE:
    section = "objectdetector" # in config_dict
    arguments = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(arguments, section)

    #READ PARAMETERS:
    vision_window    =     config_dict.get_option(section,'vision_window')
    detection_window =     config_dict.get_option(section,'detection_window')
    training_dirs    =     config_dict.get_option(section,'training_dirs')
    update_frequency = int(config_dict.get_option(section,'update_frequency'))
    min_matches      = int(config_dict.get_option(section,'min_matches'))
    visualize        = int(config_dict.get_option(section,'visualize'))
    video_source     =     config_dict.get_option(section,'video_source')

    controller_ip    =     config_dict.get_option(section,'host')
    controller_port  =     config_dict.get_option(section,'port')

    #START:
    objectdetector = ObjectDetector(controller_ip, controller_port, video_source, update_frequency, training_dirs, min_matches, visualize, vision_window, detection_window)
    objectdetector.connect()
    objectdetector.train()
    objectdetector.run()

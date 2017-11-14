import sys
import cv2.cv as cv

import os
import logging
import util.nullhandler
import util.loggingextra
import util.predator_socket
import configparse

import random

import util.speed_angle
import util.openni_kinectvideo as kv
from abstractvisionmodule import AbstractVisionModule
import SocketServer
import atexit

import time
logging.getLogger('Borg.Brain.Vision.PredatorSegment').addHandler(util.nullhandler.NullHandler())


class PredatorSegment(AbstractVisionModule):
    """
    This class is used to communicate with openTLD(Predator) software in Matlab. It should send a bounding box to matlab,
    and matlab will return the tracked bounding box.
	Note that Openni is used instead of freenect. Matlab should be using openniKinect version. 
    """

    def __init__(self, host, port):
        """
        PredatorSegment Constructor
        @param host - hostname or ip of the destination to send data (controller ip)
        @param port - port on which to send data to destination (controller port)
        
		@self.transformer - Based on image resolution and object location, it suggest a rotation angle for the pioneer
        """
 
        self.logger = logging.getLogger("Borg.Brain.Vision.PredatorSegment")
        super(PredatorSegment, self).__init__(host, port)

        
        self.predator_ready = False
        self.predator_on = False
        
		#Enable these lines if you want to also use openni kinect images in this module.
		#Note that there are some unstability problems when matlab is using openni. Somethind
		#related to Matlab's openni version

        #self.depth = kv.OpenNIKinect("depth")
        #self.rgb = kv.OpenNIKinect("rgb")
        
		#Makes sure that openni is retreiving images before conitnuing.
        #depth = None
        #while not depth:
        #    depth = self.depth.get_image()

		#Width and Height of the image being processed (resolution), select manually if
		#you are not using openni kinect
        #(width, height) = cv.GetSize(depth)
        width, height = 320, 240
        self.transformer = util.speed_angle.SpeedAngle(None, width, height)
        self.area = self.transformer.get_width() * self.transformer.get_height()
        
		#Creating path to make required files for OpenTLD startup.
        path = "/dev/shm/images"
        try:
            os.mkdir(path)
            os.mkdir(path+ "/predator")      
        except:
            self.logger.warn("Cannot create directory for predator in " + path)
        
    def train(self):
        pass

    def run(self):
        """Start loop which gets a new image, then processes it"""
        while True:         
        	self.compute_x_z(None)
            self.update()


    def compute_x_z(self,depth):
        """
		If the OpenTLD matlab socket is ready, then it reads the bounding box.
		Next, it calculates the estimated angle of the tracked object to the pioneer.
		Finally, it return an angle, distance, and confidence.

		The distance and standard deviation are now selected as zero. If you want
		to calculate distance, you have to enable using openni, and should manually
		read the average distance from the received bounding box.
 
        @param depth - the depth image
        @param rgb - the rgb image
        @param which_object default=0, the object to be considered, the default is closest
        """
                
    
        if self.predator_on:
            util.predator_socket.resetMessage()
            self.predator_handler.handle_request()
        	
			#If matlab has new data, read it.
            if util.predator_socket.observed():
                self.logger.info("getting bounding box from Opentld matlab"
				#Receiving bounding box
                (X, Y, XMAX, YMAX) = util.predator_socket.getBoundingBox()
				
                #Uncomment this to calculate approximate distance of the object
                #(dist_predator, std) = self.get_predator_distance((X, Y, XMAX, YMAX), depth)
                angle_predator = self.transformer.get_x_angle(CoM_predator)
                conf_predator = util.predator_socket.getConfidence()
                
                if conf_predator > 0.4:
                    self.add_property('name', 'predator_obj')
                    self.add_property('index', "first")
                    self.add_property('angle', angle_predator)
                    self.add_property('distance', 0)
                    self.add_property('confidence', conf_predator)
                    self.add_property('std', 0)
                    self.store_observation()

        if not self.predator_on and self.predator_ready:
			#Writing the bounding box for Matlab. At this moment it is manual.
            self.write_bounding_box([155,120,180,160], (320,240))
            self.predator_on = True
            
    def write_bounding_box(self, bb, imagesize):
        
		'''
		Writes Bounding box for OpenTLD in matlab.
		'''
        init = open("/dev/shm/images/init_bb.txt","w")
        self.logger.info("Bounding box written")
        init.write(str(bb[0])+","+str(bb[1])+","+str(bb[2])+","+str(bb[3]))
        init.close()
    
    def get_predator_distance(self, bb, depth):
        
        '''
		This function gives back the average depth and standard deviation based on a bounding box
		and a depth image.
		@param bb - Bounding box in (X,Y, XMAX, YMAX) format
		@param depth - the depth image

		'''
        #dist_rect = cv.CreateImage((bb[2]-bb[0],bb[3]-bb[1]), cv.IPL_DEPTH_8U, 1)
        if bb[0] <= 0:
            temp0 = 1   
        else:
            temp0 = bb[0]
            
        if bb[1] <= 0:
            temp1 = 1   
        else:
            temp1 = bb[1]
        if bb[2] >= cv.GetSize(depth)[0]:
            temp2 = cv.GetSize(depth)[0] - 2
        else:
            temp2 = bb[2]
        if bb[3] >= cv.GetSize(depth)[1]:
            temp3 = cv.GetSize(depth)[1] - 2
        else:
            temp3 = bb[3]    

        self.logger.debug("old bb " + bb)
        bb = (temp0, temp1,temp2,temp3)
        self.logger.debug( "new bb" + bb)
        dist_rect = (bb[0], bb[1], bb[2]-bb[0], bb[3]-bb[1])
        cv.SetImageROI(depth, dist_rect)
        depththresh = cv.CreateImage(cv.GetSize(depth), depth.depth, depth.nChannels)
        depththresh2 = cv.CreateImage(cv.GetSize(depth), depth.depth, depth.nChannels)
        depththresh3 = cv.CreateImage(cv.GetSize(depth), depth.depth, depth.nChannels)
        
        cv.Threshold(depth,depththresh,254, 255, cv.CV_THRESH_BINARY_INV)
        cv.Threshold(depth,depththresh2,10, 255, cv.CV_THRESH_BINARY_INV)
        cv.And(depththresh, depththresh2, depththresh3)
        (mean,std) = cv.AvgSdv(depth,depththresh3)
        cv.ResetImageROI(depth)
        return mean[0], std[0]


    def handle_custom_commands(self, entry):
		'''
			This part reads commands from comming from the behaviors.
		'''

		#Starting the socket server. Selecting a working socket and writing it in a file so OpenTLD can read it.
        if entry['command'] == "start_predator":
            self.predator_ready = True
            for portnumber in xrange(25000, 34999):
                try:
                    self.predator_handler = SocketServer.UDPServer(('127.0.0.1', portnumber), util.predator_socket.PredatorHandler)
                    open("/dev/shm/predatorsocket.txt", "w").write("%d" % portnumber)
                    print "USING PORT %d" % portnumber
                    break
                except:
                    pass
            atexit.register(self.predator_handler.close_request)
            self.predator_handler.timeout = 0.8
            print "Predator Ready"
        return True 
    def __del__(self):
        try:
            if self.predator_handler:
                self.predator_handler.close()
        except:
            pass            
def usage():
    print "You should add the cbbonfiguration options on the command line.\n"
    print "Usage: ", sys.argv[0], " host=<controller_host> port=<controller_port> preview=<True>\n"

if __name__ == "__main__":
    if len(sys.argv) < 2:
        usage()
        exit()

    sec = "predatorsegment" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)

    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    do_preview = config_dict.get_option(sec, "preview")
    if do_preview != None and do_preview != 'True':
        do_preview = None
    sect = config_dict.get_section(sec)
    print sect

    if not (controller_ip and controller_port):
        usage()
        exit()

    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)

    segmenter = PredatorSegment(controller_ip, controller_port)
    segmenter.connect()
    segmenter.set_socket_verbose(True, True)
    if (segmenter.is_connected()):
        segmenter.run()

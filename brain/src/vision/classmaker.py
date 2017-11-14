#!/usr/bin/python
import sys
import cv2.cv as cv
import os
import logging
import util.nullhandler
import util.loggingextra
import time
import termios
import util.ticker
import configparse
import util.vidmemreader
import datetime
import pickle
from abstractvisionmodule import AbstractVisionModule
logging.getLogger('Borg.Brain.Vision.ClassMaker').addHandler(util.nullhandler.NullHandler())


TERMIOS = termios
def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 0
    new[6][TERMIOS.VTIME] = 1

    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c




class ClassMaker(AbstractVisionModule):
    """
    This module is usually used by Manualgathering behavior. It sequentially takes pictures in a specified timing windows and 
    will class them into one class.
    """

    def __init__(self, host, port, address, source = 'webcam', autoNaming = True):
        """
        Observer Constructor
        @param host - hostname or ip of the destination to send data (controller ip)
        @param port - port on which to send data to destination (controller port)
        @param preview - set to true if you want to see the segmented object
        """
        self.logger = logging.getLogger("Borg.Brain.Vision.Observer")
        super(ClassMaker, self).__init__(host, port)
        self.address = address    
        if source == 'webcam': 
            self.vmr = util.vidmemreader.VidMemReader(['webcam'])
        else:
            self.vmr = util.vidmemreader.VidMemReader(['kinect_depth', 'kinect_rgb'])
        self.command = False
        
        self.currentCluster = ""
        self.currentClusterNum = self.clusterNum()
        self.__clusterSelected = False
        self.__make_scan_now = False
        self.__save_map = False    
        self.rgbList = []
        self.ticker = util.ticker.Ticker(4)
        self.autoNaming = autoNaming
        self.lastFileName = None
        self.lastDir = None
        
        self.__prevObs = None
        self.__curObs = None
        
    def clusterNum(self):
        """Reutrn the number of the states"""
        finalNum = 0
        tempNum = 0
        for root, _ , _ in os.walk(self.address):
            base = os.path.basename(root)
            try:
                tempNum = int(base)
                if tempNum > finalNum:
                    finalNum = tempNum
            except:
                continue
        return finalNum + 1
            
        
    def train(self):
        pass

    def run(self):
        """Start loop which gets a new image, then processes it"""
        while True:
            self.ticker.tick()         
            rgb = self.get_new_images()  
            
                      
            if self.__make_scan_now:
                self.rgbList.append(rgb)

                    
            if len(self.rgbList) > 0 and not self.__make_scan_now:
                
                self.__curObs = self.store(self.rgbList)  
                self.currentCluster = ""
                self.rgbList = []
                self.__clusterSelected = False
            self.update()
            
    

    def get_new_images(self):
        """ Gets new depth and rgb images from video shared memory """
        rgb = self.vmr.get_latest_image()[0]
        return rgb

    def store(self, rgbList):
        """Stores pictures in the directory"""
        temp = time.time()
        retName = None
        for rgb in rgbList:
            extraIndex = 1
            if time.time() - temp > 3:
                    self._AbstractVisionModule__send_heartbeat()
                    temp = time.time()
            path = self.address + "/" + self.currentCluster + '/'
            try:
                os.makedirs(path)
            except:
                pass
                   
            now = "%10.2f" % time.time()
            filename =  now + '.png'

            if os.path.isfile(filename):
                strtemp = ""
                while os.path.isfile(filename):
                    strtemp = filename + '-' + str(extraIndex)
                    extraIndex += 1
                filename = strtemp
            if not retName:
                retName = filename
            cv.SaveImage(path + filename , rgb)
            self.lastFileName = filename
            self.lastDir = path
        return retName
            
    def stateaction_store(self, action, observation, odometry):
        
        """Adds one row of state-action pair into the action file"""
        today = datetime.date.today()
        path = os.path.join(self.address,str(today))
        try:
            os.mkdir(path)
        except:
            pass
        statefile = open(os.path.join(path, str(today) + '.sa'), 'a')
        
        pickle.dump((action, observation, odometry), statefile)
        
        statefile.close()
    
    def stateActionTriple(self, observation1, action, observation2, odometry):
        """Adds one row of state-action-state into the action file"""
        today = datetime.date.today()
        path = os.path.join(self.address,str(today))
        try:
            os.mkdir(path)
        except:
            pass
        statefile = open(os.path.join(path, str(today) + '-triple' + '.sa'), 'a')
        
        pickle.dump((action, observation1, observation2, odometry), statefile)
        self.logger.debug("STATEACTION STORE: action: " + str(action) + ", obs1: " + str(observation1) + ", obs2: " +str(observation2))
        
        statefile.close()
    def handle_custom_commands(self, entry):
        """Handles commands received from the brain"""
        
        passed = True
        if entry['command'] == "make_scan":
            if self.__make_scan_now:
                return
            self.__make_scan_now = True
            temp = time.time()
            if not self.autoNaming:
                while not self.__clusterSelected:
                    if time.time() - temp > 3:
                        self._AbstractVisionModule__send_heartbeat()
                        temp = time.time()
    
                    c = getkey()
                    if c:
                        if c == "\n":
                            print "Cluster Name is:", self.currentCluster
                            self.__clusterSelected = True
                            self.add_property("name", "module_status")
                            self.add_property("module_status", "cluster_recieved")
                            break
                        else:
                            self.currentCluster += str(c)
            else:
                self.currentCluster = str(self.currentClusterNum)
                print "Cluster Name is:", self.currentCluster
                self.__clusterSelected = True
                self.add_property("name", "module_status")
                self.add_property("module_status", "cluster_recieved")
                self.currentClusterNum += 1
                        
            print "scanning is True"
            
        if entry['command'] == "stop_scan":
            print "scanning is False"
            self.add_property('name', 'module_picture')       
            self.add_property('picturename', self.lastFileName)
            self.add_property('picturedir', self.lastDir)
            self.__make_scan_now = False
            
        elif entry['command'] == "save_map":
            self.__save_map = True
            passed = True
            
        elif entry['command'] == "store":
            params = entry['params']
            action = params['action']
            odometry = params['odometry']
            
            
            self.stateaction_store(action, self.__curObs)
            
            if not self.__prevObs:
                self.__prevObs = (self.__curObs, action, odometry)
            else:        
                self.stateActionTriple(self.__prevObs[0], self.__prevObs[1], self.__curObs, odometry)
                self.__prevObs = (self.__curObs, action)
        else:
            print repr(entry)
        return passed
            
    def __del__(self):
        self.stop()


def usage():
    print "You should add the configuration options on the command line.\n"
    print "Usage: ", sys.argv[0], " host=<controller_host> port=<controller_port> preview=<True>\n"

if __name__ == "__main__":
    if len(sys.argv) < 2:
        usage()
        exit()

    #Reading from config file
    sec = "classmaker" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)

    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    address = config_dict.get_option(sec, "address")
    source = config_dict.get_option(sec, "video_source")
    logLevel = config_dict.get_option(sec, "log_level")
    if address == None:
        address = os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/data/')
        
    if source == None:
        source = "webcam"
    
    #Setting up logging
    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())    
    if logLevel == None:   
        logging.getLogger('Borg.Brain').setLevel(logging.INFO)
    elif logLevel.lower() == "warning":
        logging.getLogger('Borg.Brain').setLevel(logging.WARNING)
    elif logLevel.lower() == "debug":
        logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)
    
          
    sect = config_dict.get_section(sec)
    print sect

    if not (controller_ip and controller_port):
        usage()
        exit()

    classMaker = ClassMaker(controller_ip, controller_port, address, source)
    classMaker.connect()
    classMaker.set_socket_verbose(True, True)
    if (classMaker.is_connected()):
        classMaker.run()

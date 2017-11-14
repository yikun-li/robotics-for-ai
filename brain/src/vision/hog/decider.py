import cv2.cv as cv
import numpy
import hog

import divider
import logging
import gradient
import util.nullhandler
import time
import sys
import configparse
import os
import math
import random
import scipy.io as sio

from vision.abstractvisionmodule import AbstractVisionModule

logging.getLogger('Borg.Brain.Vision.hog.Decider').addHandler(util.nullhandler.NullHandler())

class Decider(AbstractVisionModule):
    def __init__(self, host, port, path, opath, discountfactor=0.99, method = "cv"):
        self.logger = logging.getLogger("Borg.Brain.Vision.hog.Decider")
        super(Decider, self).__init__(host, port)
        self.path = path
        self.opath = opath
        self.writing_method = method
        self.divider = divider.Divider()
        self.gradient = gradient.Gradient()
        self.hog = hog.HoG("hist")
        self.column = 5
        self.row = 5
        
        self.goal_location = None
        self.current_location = None
        self.currentFeature = None
        
        try:
            self.kcenters = self.load_center()
            self.logger.info("Center file loaded successfully")
        except:
            self.logger.error("Cannot Load Center File")
        self.values = None
        self.qvalues = None
        self.discountValue = discountfactor
        
        try:
            self.tp_matrix = self.load_tp_matrix()
            self.logger.info("Loading TP matrix... Ok")
        except:
            self.logger.error("Cannot Open TP_matrix file")
        
        self.reward= 0 #fornow
        self.stopping_criterion = 0.0001
        
        self.goal = False
        self.action = False
        self.name = ""
        
        self.temperature = 100
        
        self.numberofactions = 7
        
        self.finish = False
        
    def train(self):
        pass

    def run(self):
        """Start loop which gets a new image, then processes it"""
        self.logger.info("starting Run Loop")
        while True:    
            if self.goal:
                self.set_goal_location(self.name)
                self.value_iteration()
                self.goal = False
            if self.action:
                self.get_current_location(self.name)
                if self.finish:                 
                    self.add_property('name', 'goal_location')        
                    self.add_property('finish', True)
                    self.action = False
                else:
                    self.select_action()
                    self.action = False
            self.update()
                
    
    def value_iteration(self):

        try:
            self.values = numpy.loadtxt(self.path + 'statevalues-location-' + str(self.goal_location) + '.txt', delimiter=',', unpack = False)
            self.qvalues = numpy.loadtxt(self.path + 'qvalues-location-' + str(self.goal_location)+ '.txt', delimiter=',', unpack = False)
            
            if self.values != None and self.qvalues != None:
                self.logger.info( str(self.values.shape) )
                self.logger.info( str(self.qvalues.shape) )
                self.logger.info( "State and Q-values loaded from file")
                self.add_property('name', 'goal_location')        
                self.add_property('iteration', True)
                self.add_property('goal_location', True)
                return
        except:
            pass 
        self.values = numpy.zeros(self.kcenters.shape[0])
        old = numpy.zeros(self.kcenters.shape[0])
        self.qvalues = numpy.zeros((self.kcenters.shape[0],self.numberofactions))
        self.values[self.goal_location] = 100
        temp = time.time()
        timestart = time.time()
        stop = 1000
        while stop > self.stopping_criterion:
            
            old = self.values.copy() 
            range1 = range(self.values.shape[0])
            range2 = range(self.qvalues.shape[1])
            for i in range1:
                currenttime = time.time()
                if currenttime - temp >= 5:
                    self.logger.debug("Value iteration calculating...")
                    temp = currenttime
                    self._AbstractVisionModule__send_heartbeat()
                if i == self.goal_location:
                    continue
                for j in range2:
                    temp1 = 0
                    prob = 0

                    for z in range1:
                        temp1 += self.tp_matrix[i,z,j]*((self.reward - 1) + self.discountValue*self.values[z])
                        prob += self.tp_matrix[i,z,j]
                        
                    self.qvalues[i,j] = temp1
                    if not (prob > 0.99 or prob < 0.0000001):
                        self.logger.error("Probability sum higher than 1")
                        
                        
                maxindex = self.qvalues[i,:].argmax()
                self.values[i] = self.qvalues[i,maxindex]
                
            subtract = numpy.subtract(old,self.values)
            power = numpy.power(subtract,2)
            stop = power.sum(axis=0)
                
        timeend = time.time() - timestart
        self.logger.info("Value Iteration Finished. Duration was: "+ str(timeend))
        numpy.savetxt(self.path + 'statevalues-location-' + str(self.goal_location)+ '.txt', self.values, delimiter=',')
        numpy.savetxt(self.path + 'qvalues-location-' + str(self.goal_location) + '.txt', self.qvalues, delimiter=',')
        
        
        #send shit to memory
        self.add_property('name', 'goal_location')        
        self.add_property('iteration', True)
        self.add_property('goal_location', True)
    
    def select_action(self, temperature = False):
        self.logger.info("Qvalues:" + str( self.qvalues[self.current_location,:]) )
        firstvalue = -1
        firstaction = None
        
        if temperature:
            EQ = numpy.zeros([self.numberofactions])
            P = numpy.zeros([self.numberofactions])
            for x in range(self.numberofactions):
                EQ[x] = math.exp(self.qvalues[self.current_location,x]/self.temperature)
                
            sum_EQ = numpy.sum(EQ)
            
            for x in range(self.numberofactions):
                P[x] = EQ[x]/sum_EQ
                
            
            
            Padd = P.copy()
            
            for x in range(self.numberofactions - 1):
                Padd[x+1] = Padd[x] + Padd[x+1]
                
            pointer = random.random()
            
            print "Probabilities: ", P , " ","Accumulated Probabilities: ", Padd
            
            for x in range(self.numberofactions):
                if pointer <= Padd[x]:
                    firstaction = x
                    break;
                      
            
            
        else:  
            for x in range(3):
                temp = self.qvalues[self.current_location,x]
                if temp > firstvalue:
                    firstvalue = temp
                    firstaction = x   
            
            randomness = random.random()
            
            if randomness < 0.1 or firstaction == 3:
                firstaction = random.randint(0,2)
              
            
        action = firstaction
        #action = self.qvalues[self.current_location,:].argmax()
        if action >= 0:
            self.add_property('name', 'action')
        else:
            self.logger.error("Ambigous location")
            return
        if action == 0 or action == 4:  
            self.add_property('action', 'move')
        elif action == 1 or action == 5:
            self.add_property('action', 'left')
        elif action == 2 or action == 6:   
            self.add_property('action', 'right')
        elif action == 3:
            self.add_property('action', 'move')
            
    def set_goal_location(self,name):
        self.goal_location = self.get_cluster(name, False)
        
              
        
        
    def get_current_location(self, name):
    
        temp = self.get_cluster(name, True)
        self.logger.info( "Goal Cluster=" + str(self.goal_location + 1) )
        self.logger.info( "Current Cluster=" + str(temp + 1) )
        if temp == self.goal_location:
            print "Are we Really There?"
            self.finish = True
            self.current_location = temp
            return
        
        self.current_location = temp
    def get_feature(self, name , opath = False):
        currenttime = time.time()
        
        writing_method = "cv"
        divisionOption = "pro"
        column = 5
        row = 5

        
        tempPicture = None
        if writing_method == "cv" or writing_method == "numpy":
            currentHog = []
        else:
            currentHog = numpy.array([])
            
        if not opath:      
            tempPicture = cv.LoadImage(os.path.join(self.path, name))
        else:
            tempPicture = cv.LoadImage(os.path.join(self.opath, name))
            
        gsimage = cv.CreateImage(cv.GetSize(tempPicture), cv.IPL_DEPTH_8U, 1)
        currentPicture = cv.CreateImage(cv.GetSize(tempPicture), cv.IPL_DEPTH_8U, 1)
        if tempPicture.channels > 1 :
            cv.CvtColor(tempPicture, gsimage, cv.CV_RGB2GRAY)
            cv.Copy(gsimage,currentPicture)
        else:
            cv.Copy(tempPicture, currentPicture)
        
        (dx, dy) = self.gradient.sobelGradient(currentPicture)
        Tangent = self.gradient.tangent(dx, dy)
        Magnitude = self.gradient.Magnitude(dx, dy)
                        
        tangentList = self.divider.divide(Tangent, column, row, option = divisionOption)
                
        MagnitudeList = self.divider.divide(Magnitude, column, row, option = divisionOption)
        List = zip(tangentList, MagnitudeList)
                
        for tangent, magnitude in List:
                    
            if writing_method == "cv" or writing_method == "numpy":
                currentHog.append(self.hog.HoG(tangent, magnitude, writing_method))
            else:

                tempHist = 0
                tempHist = self.hog.HoG(tangent, magnitude, writing_method)
                currentHog = numpy.hstack((currentHog, tempHist))
        temp = time.time()
        if self.writing_method == "cv":
            index = 0
            matrix = numpy.zeros((1, len(currentHog) * 8))
            for histogram in currentHog: 
                if currenttime - temp > 2:
                    temp = currenttime
                    self._AbstractVisionModule__send_heartbeat()
                for x in range(8):
                    matrix[0, index * 8 + x] = cv.QueryHistValue_1D(histogram, x)
                index += 1
            currentHog = 0
            currentHog = matrix.copy()
        self._AbstractVisionModule__send_heartbeat()
        
        

        #(a,z) = currentHog.shape
        #print "Length:", a, " Width:", z
            
        return currentHog 
    
    def get_cluster(self,name, opath = False):
        
        currentHog = self.get_feature(name, opath)  
        self.currentFeature = currentHog  
        
        hog = currentHog.copy()
        
        (r,_) = self.kcenters.shape
        for _ in range(r - 1) :
            hog = numpy.vstack((hog,currentHog))
        #(a,z) = hog.shape
        #print "Length:", a, " Width:", z
        distance = numpy.power( numpy.subtract(self.kcenters, hog) , 2)
        distance = distance.sum(axis=1)
        numpy.savetxt( "/home/borg/distance.txt", distance,delimiter = ',')
        minindex = distance.argmin()
        
        
        
        return minindex
    
    def load_tp_matrix(self):
        return numpy.load(self.path + 'tp_matrix.npy')
        
                
    def load_center(self, loadFromMatlab = True):
        if loadFromMatlab:
            data = sio.loadmat(self.path + 'centers.mat')
            centers = data['Centers']
        else:
            centers = numpy.loadtxt(self.path + 'centers.txt', delimiter=',')
        
        return centers
        
    
    def set_var(self, column, row):
        self.column = column
        self.row = row
        
    def handle_custom_commands(self, entry):
        
        if entry['command'] == "select_goal_location":
            print "Select Goal location command received"
            params = entry['params']
            self.name = params['name']
            print "Goal Picture", self.name
            self.goal = True
            
        if entry['command'] == "select_action":
            print "Select action command received"
            params = entry['params']
            self.name = params['name']
            print "Current picture", self.name
            self.action = True     
            
            
    def __del__(self):
        self.stop()
        
    
def usage():
    print "You should add the cbbonfiguration options on the command line.\n"
    print "Usage: ", sys.argv[0], " host=<controller_host> port=<controller_port> preview=<True>\n"

if __name__ == "__main__":
    if len(sys.argv) < 2:
        usage()
        exit()

    sec = "decider" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)

    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    path = config_dict.get_option(sec, "path")
    opath = config_dict.get_option(sec, "opath")
    discountfactor = config_dict.get_option(sec, "discountf")
    
    if path == None:
        path = os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/matrixtest/') + '/'

    if opath == None:
        opath = path
        
    if discountfactor == None:
        discountfactor = 0.98
        

    sect = config_dict.get_section(sec)
    print sect

    if not (controller_ip and controller_port):
        usage()
        exit()

    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)

    decider = Decider(controller_ip, controller_port, path, opath, discountfactor)
    decider.connect()
    decider.set_socket_verbose(True, True)
    if (decider.is_connected()):
        decider.run()

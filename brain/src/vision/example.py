#!/usr/bin/python

import random

import time
import pickle
import sys

import glob 
import os 

import os
import logging
import util.nullhandler
import util.loggingextra

import configparse
from abstractvisionmodule import AbstractVisionModule
import SocketServer
import shutil
logging.getLogger('Borg.Brain.Vision.Example').addHandler(util.nullhandler.NullHandler())


class Example(AbstractVisionModule):
    """
    Example (vision) module, to be used for testing, does not use any hardware.
    """

    def __init__(self, host, port):
        """Initialization of the module, do not forget to call the constructor of the super class."""
        self.logger = logging.getLogger("Borg.Brain.Vision.Example")
        super(Example, self).__init__(host, port)

        self.__last_send_time = 0

    def train(self):
        #Mandatory, but we do not use this function in this example.
        pass

    def run(self):
        """Start loop which simple sends a random fake observation every 5 secondes."""
        while True:         
            #Send memory update every 5 seconds:
            if time.time() - self.__last_send_time > 1.0:
                #Name, used to identify the observation in memory:
                self.add_property('name', 'example')       
                #One or more field in the dictionary:
                self.add_property('value_1', random.random())
                self.add_property('value_2', random.random())
                self.store_observation()

                self.__last_send_time = time.time()

            self.update()
            

def usage():
    print "You should add the configuration options on the command line.\n"
    print "Usage: ", sys.argv[0], " host=<controller_host> port=<controller_port> preview=<True>\n"

if __name__ == "__main__":
    if len(sys.argv) < 2:
        usage()
        exit()

    sec = "example" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)

    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")

    sect = config_dict.get_section(sec)
    print sect

    if not (controller_ip and controller_port):
        usage()
        exit()

    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)

    #Create module and connect to visioncontroller.
    detector = Example(controller_ip, controller_port)
    detector.connect()
    detector.set_socket_verbose(True, True)
    if (detector.is_connected()):
        detector.run()


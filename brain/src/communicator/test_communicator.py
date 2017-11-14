# imports
import unittest
import time
import socket
import os
import sys
import signal
import subprocess
import logging
import random

import util.nullhandler
import util.loggingextra
from util.threadedsocket import ThreadedSocket

import communicator
import configparse

class CommunicatorTestCase(unittest.TestCase):
    def setUp(self):
        if not hasattr(self, "random_seed"):
            random.seed()
            self.random_seed = True
        self.testport = random.randint(49152, 65535)
        self.communicator = communicator.Communicator(port = self.testport)
        self.params = configparse.ParameterDict()
        self.params.add_option('vision_controller', 'video_source_name', 'webcam')
        self.TestCommand = 'this is a command to start xterm'

    def tearDown(self):
        self.communicator.close()
        del self.communicator
        self.communicator = None

    def test_startvidmemwriter(self):
        """Test if starting the vidmemwriter works.
           This test could cause problems when no camera is connected to the system."""
        if (os.path.exists("/dev/video0")):
            self.communicator.start_video_source("webcam")
            shm_path = "/dev/shm/images/webcam"
            self.assertEqual(os.listdir(shm_path),[])
        else:
            print "\n!WARNING! !WARNING! !WARNING!"
            print "     NO CAMERA CONNECTED     "
            print "!WARNING! !WARNING! !WARNING!"
            self.assertTrue(True)

    def test_getCommand(self):
        """Test wether getting commands from the network works.
           Sends command through TCP/IP and check wether the same command is received on the other end."""
        self.assertTrue(self.communicator.wait_listen(2), "Communicator not listening")
        CommandSocket = ThreadedSocket('localhost', self.testport, giveup=0, retry_timeout=0.1)
        self.assertTrue(CommandSocket.wait_connect(4), "Could not connect to communicator")
        CommandSocket.send(self.TestCommand)
        time.sleep(0.1) # Wait a bit for the socket to be available
        received, conn_id = self.communicator.get_command()
        self.assertEqual(received, self.TestCommand) #,"Oh no, we received something else than we sent!")
        
    def test_startProcess(self):
        """Tests whether it can start a seperate process"""
        args = os.environ['BORG'] + "/Brain/data/commtest/test"
        self.communicator._Communicator__processList["0"] = []
        process = self.communicator.start_process(args, 0)

        x = process.poll()
        
        process.terminate()
        process.kill()
        self.assertEqual(x,None, "Either the process is not started or it was terminated unexpectadly")
        
    def test_ProcessList(self):
        process1 = os.environ['BORG'] + "/Brain/data/commtest/test1"
        process2 = os.environ['BORG'] + "/Brain/data/commtest/test"
         
        self.communicator._Communicator__processList["0"] = []
        p1 = self.communicator.start_process(process1, 0)
        p2 = self.communicator.start_process(process2, 0)
        
        processList = self.communicator._Communicator__processList["0"]

        x = range(len(processList))
        
        counter = 0
        for i in x:
            if processList[i][1] == process1:
                self.assertEqual(processList[i][1], process1, "The ProcessList returned wrong")
                #self.communicator.process_kill(process1)
                #print p1.pid
                time.sleep(0.1)
                os.killpg(p1.pid, signal.SIGINT)
                counter += 1 

            if processList[i][1] == process2:
                self.assertEqual(processList[i][1], process2, "The ProcessList returned wrong")
                #self.communicator.processKill(process2)
                #print p2.pid
                time.sleep(0.1)
                os.killpg(p2.pid, signal.SIGINT)
                counter += 1  
        
        self.assertEqual(counter, 2, "One or both of the processes were not found")
        
    def test_ProcessKill(self):
        result = 0
        process = os.environ['BORG'] + "/Brain/data/commtest/test"
        
        self.communicator._Communicator__processList["0"] = []
        p1 = self.communicator.start_process(process, 0)
        pid = p1.pid
        time.sleep(0.2)
        failed = False
        try:
            self.communicator.process_kill(process, 0)
        except:
            failed = True
        
        self.assertEqual(failed, False, "The kill process is failing")
        
    def test_copyModules2SHM(self):
        self.communicator.copyModules2SHM()
        repoList = os.listdir(os.environ['BORG'] + '/Brain/src/vision')
        shmList = os.listdir('/dev/shm/visionmodules')
        repoList.sort()
        shmList.sort()
        #print shmList
        #print repoList
        self.assertEqual(shmList,repoList,"The list of files from the vision folder and the shared memory visionmodules folder were not equal.")
        
    def test_processCommand(self):
        cmd = "Not a command"
        result = self.communicator.process_command(cmd, 0)
        expected = (-1, "Wrong Communication. It should be a dictionary.")
        #print result
        #print expected
        self.assertEqual(expected, result, "The communicator is mistakenly reading a non dictionary")
        
        #cmd={'command':'start-module','port':'49152','module':'video_source=None'}
        
    def test_multipleProcess(self):
        self.communicator._Communicator__processList["0"] = []
        self.communicator.commands = [{"command": "start_module", "port": 45999, "module": "../util/testProcess video_source=None"},
                                      {"command": "start_module", "port": 46000, "module": "../util/testProcess video_source=None"}]
        self.communicator.process_command(self.communicator.commands[0], 0)
        self.communicator.process_command(self.communicator.commands[1], 0)

        processList = self.communicator._Communicator__processList["0"]
        
        self.assertEqual(len(processList), 2, "Did not start two processes!")

        self.communicator.kill_all(0)

        
def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(CommunicatorTestCase))
    return suite

if __name__ == '__main__':
    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)
    unittest.main()

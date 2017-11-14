import socket
import unittest
import time
import cPickle as pickle
import random
import sys
import logging

import util.loggingextra
import visionmodule_listener
import visioncontroller

class VisionModuleListenerHandlerTestCase(unittest.TestCase):
    """
    TestCase for the VisionModuleListener and VisionModuleConnectionHandler classes
    that listen and handle connections from vision modules
    """

    def setUp(self):
        if not hasattr(self, "random_seed"):
            random.seed()
            self.random_seed = True
        self.port = random.randint(49152, 65535)

    def test_connection(self):
        listener = None
        try:
            listener = visionmodule_listener.VisionModuleListener(self.port, "test_listener", None)
            listener.start()
            
            self.assertTrue(listener.wait_listen(), "Listener is not listening")
            time.sleep(0.2)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(('localhost', self.port))
            s.settimeout(1)

            # Receive initial command
            data = pickle.loads(s.recv(1024))
            self.assertEquals(data, {"command": "send_capabilities", "params": {}},
                                     "Command not properly sent")

            # Send test
            s.send(pickle.dumps([{"command": "test"}]))
            s.close()

            listener.close()
            listener.join()
        except:
            if listener:
                listener.close()
                listener.join()
            raise 

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(VisionModuleListenerHandlerTestCase))
    return suite

if __name__ == '__main__':
    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)
    unittest.main()

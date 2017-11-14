import unittest
import random
import time
import logging
import sys

import communicator_connector
import util.threadedsocket
import util.loggingextra

class CommunicatorConnectorTestCase(unittest.TestCase):
    """
    TestCase for the CommunicatorConnector class that connects
    with a vision communicator
    """

    def setUp(self):
        # Set up listening socket
        if not hasattr(self, "random_seed"):
            random.seed()
            self.random_seed = True
        self.port = random.randint(49152, 65535)
        self.listener = util.threadedsocket.ThreadedSocket("", self.port, server=True)
        self.maxWait = 1

    def tearDown(self):
        # Close listening socket
        self.listener.close()
        self.listener = None

    def test_connection(self):
        # This is run in a try/except structure because the threads need
        # to be shut down if anything fails, otherwise the interpreter hangs
        conn = None
        try:
            conn = communicator_connector.CommunicatorConnector('localhost', self.port, {})
            self.assertTrue(conn.wait_connect(5.0))
            conn.close()
            while conn.connected():
                pass
            self.assertTrue(not conn.connected())
        except:
            # Stop threads
            if conn:
                conn.close()
            self.listener.close()
            raise

    def test_data_transfer(self):
        # This is run in a try/except structure because the threads need
        # to be shut down if anything fails, otherwise the interpreter hangs
        conn = None
        try:
            modules = [{"port": 50000, "command": "module1"}, {"port": 50001, "command": "module2"}]
            conn = communicator_connector.CommunicatorConnector('localhost', self.port, modules)
            self.assertTrue(conn.wait_connect())
            # Wait for data to come in
            data = self.listener.wait_data()
            if len(data) < 2:
                data2 = self.listener.wait_data()
                data.extend(data2)
            self.assertTrue({'command': 'start_module', 
                             'module': 'module2',
                             'port': "Vision_50001"} in data)
            self.assertTrue({'command': 'start_module',
                             'module': 'module1',
                             'port': "Vision_50000"} in data)
            conn.send_command("test_command")
            data = self.listener.wait_data()
            self.assertEquals(data, [{'command': 'test_command'}])
            conn.close()
            self.assertTrue(conn.wait_connect(disconnect=True))
        except:
            # Stop threads
            if conn:
                conn.close()
            self.listener.close()
            raise

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(CommunicatorConnectorTestCase))
    return suite

if __name__ == '__main__':
    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.INFO)
    unittest.main()

# imports
import unittest
import speechcontroller
import util.testsocket
import time
import configparse

class  SpeechControllerTest(unittest.TestCase):
    def setUp(self):
        params = configparse.ParameterDict()
        params.add_option('speech_controller', 'modules', '')

        self.SC = speechcontroller.SpeechController(params)
        self.test_socket = util.testsocket.TestSocket()
        self.SC.receive_sockets = [self.test_socket]

    def tearDown(self):
        self.SC = None
        self.test_socket = None        

    def test_update(self):
        """test update function of speechcontroller test"""
        example_strings = ["hi bob", "how are you bob", "get me a coffee and clean my room", "let's go for tequila"]
        for string in example_strings:
            self.test_socket.set_data(string)
            before_time = time.time()
            readcommand = self.SC.update()
            after_time = time.time()
            self.assertEqual(string, readcommand[0]['property_dict']['message'])
            assert(readcommand[0]['time'] > before_time)
            assert(readcommand[0]['time'] < after_time)
            self.assertEqual(readcommand[0]['name'], 'voice_command')

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(SpeechControllerTest))
    return suite

if __name__ == '__main__':
    unittest.main()



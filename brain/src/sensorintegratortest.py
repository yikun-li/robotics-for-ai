# imports
import unittest
import sensorintegrator
import time
import logging
import util.loggingextra
import configparse
import memory
import util.testsocket

class  SensorintegratorTestCase(unittest.TestCase):
    def setUp(self):
        params = configparse.ParameterDict()
        self.IP = '192.168.0.1'
        params.add_option('general', 'robot_ip', self.IP )
        params.add_option('vision_controller', 'video_source_name', 'test')

        params.add_option('general', 'starting_behavior', 'test1')
        params.add_option('speech_controller', 'modules', '') #no speech modules, we'll add a test socket later on 
        params.add_option('speech_controller', 'start_speech', 'true') #no speech modules, we'll add a test socket later on 

        params.add_option('body','number_of_naos','0')
        params.add_option('body','number_of_pioneers','0')
        self.mem = memory.Memory()
        self.mem.clear()
        self.si = sensorintegrator.SensorIntegrator(params)

    def tearDown(self):
        #self.si.dispose()
        self.si = None

    def test_initialization(self):
        """Initializing sensors"""
        self.assertEqual(self.si.memory.n_items(), 0, "Memory not initialized empty");

    def test_add_to_memory(self):
        """Adding sensordata to memory"""
        current_time = time.time()
        self.si.add_to_memory([{'name':'example', 'time':current_time, 'property_dict':{}}])
        self.assertEqual(self.si.memory.n_items(),1,'there should be only one item in memory')
        self.assertEqual(self.si.memory.n_occurs('example'),1,'item not in memory')
        self.assertEqual(self.si.memory.get_last_observation('example'),
            (current_time, {}),'item not correct in memory')

        current_time = time.time()
        self.si.add_to_memory([{'name':'example', 'time':current_time, 'property_dict':{}}])
        self.assertEqual(self.si.memory.n_items(),1,'there should be exactly one item in memory')
        self.assertEqual(self.si.memory.n_occurs('example'),2,'there should be exactly two instances in memory')

        pdict = {'angle':88, 'color':'green', 'scale':2}
        self.si.add_to_memory([{'name':'example', 'time':current_time, 'property_dict':pdict}])
        self.assertEqual(self.si.memory.n_occurs('example'),
            3, 'there should be exactly three instances in memory')
        self.assertEqual(self.si.memory.get_last_observation('example'),
            (current_time, pdict), 'properties not correct in memory')

    def test_speech_recog_update(self):
        params = configparse.ParameterDict()
        self.test_socket = util.testsocket.TestSocket()
        self.si.speechcontroller.receive_sockets = [self.test_socket] 
        message = 'I want a tequila from the kitchen'
        self.test_socket.set_data(message)
        self.si.add_to_memory(self.si.speechcontroller.update())

        obs = self.mem.get_last_observation('voice_command')
        self.assertEqual(obs[1]['message'], message)
        
        

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(SensorintegratorTestCase))
    return suite

if __name__ == '__main__':
    #logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    #logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)
    unittest.main()

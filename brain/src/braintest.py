# imports
import unittest
import brain
import configparse

class  BrainTestCase(unittest.TestCase):
    def setUp(self):
        params = configparse.ParameterDict()
        params.add_option('general', 'robot_ip', '129.125.178.227')
        params.add_option('general', 'starting_behavior', 'followMe')
        params.add_option('vision_controller', 'video_source_name', 'test')
        params.add_option('body','number_of_naos','0')
        params.add_option('body','number_of_pioneers','0')

        self.brain = brain.Brain(params)
    

    def tearDown(self):
        self.brain.stop()
        

    def test_brain(self):
        """test initialisation of the brain speed"""
        self.assertEqual(self.brain.brain_speed, 10, 'default brain speed is 10')
        self.assertEqual(self.brain.get_brain_speed(),10, 'returned brain speed should be 10')
        self.brain.set_brain_speed( 20 )
        self.assertEqual(self.brain.get_brain_speed(), 20, 'brain speed should be 20')

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(BrainTestCase))
    return suite





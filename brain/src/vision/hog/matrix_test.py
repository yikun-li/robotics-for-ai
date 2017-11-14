import unittest

import os
import matrix


class Matrix_test(unittest.TestCase):
    '''
    The tests for the divider
    '''
        
    def setUp(self):
        path = (os.environ['BORG'] + '/Brain/data/hog_test/matrixtest/')
        
        self.matrix = matrix.Matrix(path,date = "2011-11-08")
        
    def tearDown(self):
        pass
    
    def test_load_matlab_data(self): 
        result = self.matrix.load_data_matlab()
        self.assertEqual(len(result),12,'The number of rows read from matlab file is incorrect')
        
    def test_load_actions(self): 
        result = self.matrix.loadActions()
        self.assertEqual(len(result),11,'The number of rows read from action file is incorrect')
        
    def test_observation_to_state(self):
        data = self.matrix.load_data_matlab()
        actions = self.matrix.loadActions()
        result, numbers = self.matrix.observation_to_state(actions, data)
        self.assertEqual(len(result),11,'Observation to state is not working properly')
        
    def test_calculate_tp_matrix(self):
        data = self.matrix.load_data_matlab()
        actions = self.matrix.loadActions()
        transitions = self.matrix.observation_to_state(actions, data)
        result = self.matrix.calculate_tp_matrix(transitions, len(transitions))
        self.assertNotEqual(len(result),0,'Observation to state is not working properly')
        
def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(Matrix_test))
    return suite

if __name__ == '__main__':
    unittest.main()  


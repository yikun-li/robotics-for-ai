'''
Created on Nov 11, 2011

@author: shantia
'''
import unittest
import decider
import os


class Decider_test(unittest.TestCase):
    '''
    The tests for the divider
    '''
        
    def setUp(self):
        #self.path = os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/matrixtest') + '/'
        self.path = '/data/scratch/Shantia/2011-11-16/'
        self.decider = decider.Decider('localhost', 50000, self.path, self.path)

        
    def tearDown(self):
        pass
      
    def test_valueiteration(self):
        name = '1321441791.jpg'
        self.decider.set_goal_location(name)
        self.assertEqual(self.decider.goal_location,2 -1  , 'The goal location is not correctly set') #Beware the matlab index is 1 higher from python index
        self.decider.value_iteration()
    
    def test_actionselection(self):
        name = '1321441791.jpg'
        self.decider.set_goal_location(name)
        self.decider.value_iteration()
        name = '1321453958.jpg'
        self.decider.get_current_location(name) 
        self.decider.select_action() 
    
def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(Decider_test))
    return suite

if __name__ == '__main__':
    unittest.main() 
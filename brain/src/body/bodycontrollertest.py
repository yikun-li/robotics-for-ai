# imports
import unittest
import configparse
import body.bodycontroller

#TODO: expand

class BodyControllerTest(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass
        
        
        
    def testSingleton(self):
        param_dict = configparse.ParameterDict()
        param_dict.add_option('body','number_of_naos', '1')
        param_dict.add_option('body','number_of_pioneers', '1')
        param_dict.add_option('body','nao_ip_0', '000.000.000')
        param_dict.add_option('body','nao_port_0', '0')
        param_dict.add_option('body','pioneer_ip_0', '000.000.000')
        param_dict.add_option('body','pioneer_port_0', '0')        
        
        #TODO: if we load the config we also have to have a robot connected...


def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(BodyControllerTest))
    return suite

if __name__ == '__main__':
    unittest.main()



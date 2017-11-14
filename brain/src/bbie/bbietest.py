import bbie
import unittest

class  BBIETestCase(unittest.TestCase):
    def setUp(self):
        self.bbie = bbie.BBIE( )

    def tearDown(self):
        self.bbie = None
        
        
    def test_update_dictionary(self):
        self.bbie.update_dictionary('hello', 'world')
        self.assertTrue(self.bbie.dictionary['hello'] == ['world'])
        self.bbie.update_dictionary('hello', 'planet')
        self.assertTrue(self.bbie.dictionary['hello'] == ['world', 'planet']) 

    def test_write_data(self):
        self.bbie.update_dictionary('hello', 'world')
        self.assertTrue(self.bbie.dictionary['hello'] == ['world'])
        self.bbie.update_dictionary('hello', 'planet')
        self.assertTrue(self.bbie.dictionary['hello'] == ['world', 'planet']) 
        self.bbie.write_data()
        bbie_test = bbie.BBIE()
        bbie_test.read_data()
        self.assertTrue(bbie_test.dictionary['hello'] == ['world', 'planet']) 
        self.bbie.clear_file()
        

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(BBIETestCase))
    return suite

if __name__ == '__main__':
    runner = unittest.TextTestRunner(verbosity=2)
    runner.run(suite())


import unittest
import behavior
import memory
import time
import behavior.unittest.unittest

class BehaviorTestCase(unittest.TestCase):
    def setUp(self):
        self.mem = memory.Memory()
        cur_time = time.time()

        #add some objects to the memory:

        name = 'example'
        prop =  {'color':'yellow', 'length':20}
        self.mem.add_item(name, cur_time, prop)

        name = 'example2'
        prop =  {'color':'blue', 'length':40}
        self.mem.add_item(name, cur_time, prop)

        name = 'example2'
        prop =  {'color':'blue', 'length':50}
        self.mem.add_item(name, cur_time, prop)


        self.testBehavior = behavior.unittest.unittest.UnitTest({'example_param': 5})
    

    def tearDown(self):

        del self.testBehavior
        self.mem.clear()


    def test_sub_behaviors(self):
        #TODO: implement
        pass


    def test_update_behavior(self):

        #check whether the update calls are correctly routed to current implementation:
        self.testBehavior.update()
        self.testBehavior.update()

        #get the implementation to use in the assert:
        impl = self.testBehavior.selected_implementation

        self.assertTrue(impl.get_counter() == 2)



    def test_check_postconditions(self):
        self.testBehavior.update()
        self.testBehavior.update()
        
        #trigger the postcondition:
        cur_time = time.time()
        name = 'testpost'
        prop =  {'color':'blue', 'length':50}
        self.mem.add_item(name, cur_time, prop)

        self.testBehavior.update()
        self.testBehavior.update()

        #get the implementation to use in the assert:
        impl = self.testBehavior.selected_implementation

        self.assertTrue(impl.get_counter() == 2)


    def test_parameters(self):
        self.testBehavior.update()
        impl = self.testBehavior.selected_implementation

        #check parameter:
        self.assertTrue(impl.example_param == 5)


def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(BehaviorTestCase))
    return suite


if __name__ == '__main__':
    unittest.main()


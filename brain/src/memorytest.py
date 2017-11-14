# imports
import unittest
import memory
from memory import Memory
import time
import os

class  MemoryTestCase(unittest.TestCase):
    def setUp(self):
        os.system("mv " + os.environ['BORG'] + "/brain/src/config/mem_config " + os.environ['BORG'] + "/brain/src/config/real_mem_config")
        self.write_config() #write a standard testing version of the mem_config file
        mem = memory.Memory()
        mem.__init__()

    def test_init(self):
        mem = memory.Memory()
        mem.clear()
        #check if lastonly objects and long-term objects have been read as in the written mem_config file

        self.assertEqual('last_item1' in mem.keep_lastonly, True)
        self.assertEqual('last_item2' in mem.keep_lastonly, True)
        self.assertEqual('last_item3' in mem.keep_lastonly, False)
        self.assertEqual(len(mem.keep_lastonly), 2, "error")

        self.assertEqual('long_item1' in mem.keep_long, True)
        self.assertEqual('long_item2' in mem.keep_long, True)
        self.assertEqual('long_item3' in mem.keep_long, False)
        self.assertEqual(len(mem.keep_lastonly), 2)

    def tearDown(self):
        mem = memory.Memory()
        mem.clear()
        os.system("mv " + os.environ['BORG'] + "/brain/src/config/real_mem_config " + os.environ['BORG'] + "/brain/src/config/mem_config")

    def test_clear(self):
        """Initializing memory"""
        mem = memory.Memory()
        mem.add_item('example', time.time(), {'blieb':'bliebje'})
        mem.clear()
        self.assertEqual(mem.n_items(), 0, "On clear 0 items should be in memory")
        
    def test_singleton(self):
        """Test singleton properties"""
        mem1 = memory.Memory()        
        mem1.add_item('example', time.time(), {'blieb':'bliebje'})
        mem2 = memory.Memory()
        self.assertEqual(mem1.n_items(), mem2.n_items(), "item should be in both references")
        n_items = mem1.n_items()
        mem1 = None
        mem2 = None
        mem3 = memory.Memory()
        self.assertEqual(mem3.n_items(), n_items, "item should be in both references")


        
    def test_add_item_occurs_item(self):
        """adding items to memory"""
        current_time = time.time()
        mem = memory.Memory()
        mem.add_item('last_item1', current_time, {'color':'yellow'})
        self.assertEqual(mem.n_items(), 1, "1 item should be in memory")
        self.assertEqual(mem.n_occurs('last_item1'), 1, "there is now one instance of example")
        current_time = time.time()
        mem.add_item('last_item1', current_time, {'color':'red'})
        self.assertEqual(mem.n_items(), 1, "1 item should be in memory")
        self.assertEqual(mem.n_occurs('last_item1'), 1, "there should still be only one instance, as the item should be overwritten")
        self.assertEqual(mem.get_last_observation('last_item1')[1]['color'], 'red', "the second item should overwrite the first")

    def test_add_item_occurs_item_lastonly(self):
        """adding items to memory"""
        current_time = time.time()
        mem = memory.Memory()
        mem.add_item('example', current_time, {'color':'yellow'})
        self.assertEqual(mem.n_items(), 1, "1 item should be in memory")
        self.assertEqual(mem.n_occurs('example'), 1, "there is now one instance of example")
        current_time = time.time()
        mem.add_item('example', current_time, {'color':'red'})
        self.assertEqual(mem.n_items(), 1, "1 item should be in memory")
        self.assertEqual(mem.n_occurs('example'), 2, "there are now two instances of example")

    def test_get_observations(self):
        """Getting observations from memory"""
        ctime = time.time()
        name = 'example'
        prop = {'color':'yellow'}
        mem = memory.Memory()
        mem.add_item(name, ctime, prop)
        self.assertEqual(mem.get_observations(name), [(ctime, prop)], "observations incorrect")
        ctime2 = time.time()
        prop2 = {'color':'blue'}
        mem.add_item(name, ctime2, prop2)
        self.assertEqual(mem.get_observations(name), [(ctime, prop), (ctime2, prop2)], "observations incorrect")
        name2 = 'doesntexist'
        self.assertEqual(mem.get_observations(name2), [], "observations incorrect")

    def test_get_recent_observations(self):
        name = 'example'
        prop =  {'color':'yellow'}
        times = [1., 5., 7., 19.]
        mem = memory.Memory()
        for time in times:
            mem.add_item(name, time, prop)
        #get observations with timestamp > 6
        recent_obs = mem.get_recent_observations(name, 6)
        expected_times = [7., 19.]
        for i, expected_time in enumerate(expected_times):
            #time from the returned list:
            returned_time = recent_obs[i][0]
            self.assertEqual(returned_time, expected_time)
    
    def test_get_last_observation(self):
        """Remembering last observations"""
        ctime = time.time()
        name = 'example'
        prop = {'color':'yellow'}
        mem = memory.Memory()
        mem.add_item(name, ctime, prop)
        self.assertEqual(mem.get_last_observation(name), (ctime, prop), "last observation incorrect")
        prop2 = {'color':'blue'}
        mem.add_item(name, ctime - 1, prop2)
        self.assertEqual(mem.get_last_observation(name), (ctime, prop), "last observation incorrect")
        prop3 = {'color':'red'}
        mem.add_item(name, ctime + 1, prop3)
        self.assertEqual(mem.get_last_observation(name), (ctime + 1, prop3), "last observation incorrect")
        prop4 = {'color':'pink'}
        mem.add_item(name, ctime + 1, prop4)
        self.assertEqual(mem.get_last_observation(name), (ctime + 1, prop4), "last observation incorrect")

        self.assertEqual(mem.n_occurs(name), 4, "number of observations incorrect")
        name2 = 'doesntexist'
        self.assertEqual(mem.get_last_observation(name2), None, "there should be no observations")
        
    def test_get_observations_newskool(self):
        """Getting observations from memory"""
        ctime = time.time()
        name = 'example'
        prop = {'color':'yellow'}
        mem = memory.Memory()
        mem.add_item(name, ctime, prop)
        self.assertEqual(mem.get_observations(name, False), [prop], "observations incorrect")
        ctime2 = time.time()
        prop2 = {'color':'blue'}
        mem.add_item(name, ctime2, prop2)
        self.assertEqual(mem.get_observations(name, False), [prop, prop2], "observations incorrect")
        name2 = 'doesntexist'
        self.assertEqual(mem.get_observations(name2, False), [], "observations incorrect")

    def test_get_recent_observations_newskool(self):
        name = 'example'
        prop =  {'color':'yellow'}
        times = [1., 5., 7., 19.]
        mem = memory.Memory()
        for time in times:
            mem.add_item(name, time, prop)
        #get observations with timestamp > 6
        recent_obs = mem.get_recent_observations(name, 6, False)
        expected_times = [7., 19.]
        self.assertEqual(len(recent_obs), len(expected_times))
    
    def test_get_last_observation_newskool(self):
        """Remembering last observations"""
        ctime = time.time()
        name = 'example'
        prop = {'color':'yellow'}
        mem = memory.Memory()
        mem.add_item(name, ctime, prop)
        self.assertEqual(mem.get_last_observation(name, False), prop, "last observation incorrect")
        prop2 = {'color':'blue'}
        mem.add_item(name, ctime - 1, prop2)
        self.assertEqual(mem.get_last_observation(name, False), prop, "last observation incorrect")
        prop3 = {'color':'red'}
        mem.add_item(name, ctime + 1, prop3)
        self.assertEqual(mem.get_last_observation(name, False), prop3, "last observation incorrect")
        prop4 = {'color':'pink'}
        mem.add_item(name, ctime + 1, prop4)
        self.assertEqual(mem.get_last_observation(name, False), prop4, "last observation incorrect")

        self.assertEqual(mem.n_occurs(name), 4, "number of observations incorrect")
        name2 = 'doesntexist'
        self.assertEqual(mem.get_last_observation(name2, False), None, "there should be no observations")

    def test_is_now(self):
        '''test if the is_now function works'''
    
        ctime = time.time()
        name = 'example'
        prop = {'color':'yellow'}
        mem = memory.Memory()
        mem.add_item(name, ctime, prop)

        self.assertEqual(mem.is_now('example',['color==\"yellow\"']), True, "is_now is failing!")
        self.assertEqual(mem.is_now('example',['color==\"blue\"']), False, "is_now is failing!")
        self.assertEqual(mem.is_now('example2',['color==\"yellow\"']), False, "is_now is failing!")
        self.assertEqual(mem.is_now('example2',['color==\"blue\"']), False, "is_now is failing!")

        ctime = time.time()
        name = 'example'
        prop = {'color':'blue'}
        mem.add_item(name, ctime, prop)

        self.assertEqual(mem.is_now('example',['color==\"yellow\"']), False, "is_now is failing!")
        self.assertEqual(mem.is_now('example',['color==\"blue\"']), True, "is_now is failing!")
        self.assertEqual(mem.is_now('example',['nonexistent==\"red\"']), False, "is_now is failing on nonexisting property!")

    def test_was_ever(self):
        '''test if the was_ever function works'''

        ctime = time.time()
        name = 'example'
        prop = {'color':'yellow'}
        mem = memory.Memory()
        mem.add_item(name, ctime, prop)

        self.assertEqual(mem.was_ever('example',['color==\"yellow\"']), True, "mem.was_ever is failing!")
        self.assertEqual(mem.was_ever('example',['color==\"blue\"']), False, "mem.was_ever is failing!")
        self.assertEqual(mem.was_ever('example2',['color==\"yellow\"']), False, "mem.was_ever is failing!")
        self.assertEqual(mem.was_ever('example2',['color==\"blue\"']), False, "mem.was_ever is failing!")

        ctime = time.time()
        name = 'example'
        prop = {'color':'blue'}
        mem.add_item(name, ctime, prop)

        self.assertEqual(mem.was_ever('example',['color==\"yellow\"']), True, "mem.was_ever is failing!")
        self.assertEqual(mem.was_ever('example',['color==\"blue\"']), True, "mem.was_ever is failing!")
        self.assertEqual(mem.was_ever('example',['color==\"red\"']), False, "mem.was_ever is failing!")
        self.assertEqual(mem.was_ever('example',['nonexistent==\"red\"']), False, "mem.was_ever is failing on nonexisting property!")


    def test_was_last_time(self):
        '''test if the was_last_time function works'''
    
        mem = memory.Memory()
            
        self.assertEqual(mem.was_last_time('example',['color==\"yellow\"']), False, "mem.was_last_time is failing!")
        self.assertEqual(mem.was_last_time('example',['color==\"red\"']), False, "mem.was_last_time is failing!")
        self.assertEqual(mem.was_last_time('example',['nonexistent==\"red\"']), False, "mem.was_last_time is failing!")
        
        ctime = time.time()
        name = 'example'
        prop = {'color':'yellow'}
        mem = memory.Memory()
        mem.add_item(name, ctime, prop)

        self.assertEqual(mem.was_last_time('example',['color==\"yellow\"']), True, "mem.was_last_time is failing!")
        self.assertEqual(mem.was_last_time('example',['color==\"red\"']), False, "mem.was_last_time is failing!")
        self.assertEqual(mem.was_last_time('example',['nonexistent==\"red\"']), False, "mem.was_last_time is failing!")


        ctime = time.time()
        name = 'example'
        prop = {'color':'red'}
        mem = memory.Memory()
        mem.add_item(name, ctime, prop)
        
        self.assertEqual(mem.was_last_time('example',['color==\"yellow\"']), False, "mem.was_last_time is failing!")
        self.assertEqual(mem.was_last_time('example',['color==\"red\"']), True, "mem.was_last_time is failing!")
        self.assertEqual(mem.was_last_time('example',['nonexistent==\"red\"']), False, "mem.was_last_time is failing!")        
        

    def test_recognizable_objects(self):
        '''test if the memory knows what object it knows'''

        mem = memory.Memory()

        self.assertEqual(mem.is_object_recognizable("chair"), False, "error")
        
        mem.add_recognizable_objects("vision",["chair","table"])

        self.assertEqual('chair' in mem.get_recognizable_objects(), True)
        self.assertEqual('table' in mem.get_recognizable_objects(), True)
        self.assertEqual('couch' in mem.get_recognizable_objects(), False)
        self.assertEqual(len(mem.get_recognizable_objects()), 2)

        mem.add_recognizable_object("vision", "couch")
        self.assertEqual('couch' in mem.get_recognizable_objects(), True, "error")
        self.assertEqual(len(mem.get_recognizable_objects()), 3, "error")

        self.assertEqual(mem.is_object_recognizable("chair"), True, "error")
        self.assertEqual(mem.is_object_recognizable("couch"), True, "error")

        mem.clean_recognizable_objects("vision")

        self.assertEqual(mem.is_object_recognizable("chair"), False, "error")

        self.assertEqual('chair' in mem.get_recognizable_objects(), False, "error")
        self.assertEqual('table' in mem.get_recognizable_objects(), False, "error")
        self.assertEqual('couch' in mem.get_recognizable_objects(), False, "error")
        self.assertEqual(len(mem.get_recognizable_objects()), 0, "error")


    def write_config(self):
        conf = open(os.path.abspath(os.environ['BORG'] + '/brain/src/config/mem_config'), 'w')
        conf.write("""#All objects not mentioned here will be kept for the entire runtime of the brain.

[lastonly]
#all items from which only the last detection needs to be saved. Mainly items that take up much memory
last_item1 =
last_item2 = 

[long]
#all items that have to be saved when the program quits. Not yet implemented! (for now, same as unmentioned objects)
long_item1 = 
long_item2 = 

        """)
        conf.close()




def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(MemoryTestCase))
    return suite

if __name__ == '__main__':
    unittest.main()

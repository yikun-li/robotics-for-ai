

'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import basebehavior.behaviorimplementation
import random


class SubProbability_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    '''this is a behavior implementation template'''
    def implementation_init(self):
	pass

    def implementation_update(self):
        self.name = self.m.get_last_observation("NameObject")
	if bool(random.getrandbits(1)):
	    print "Successfully print name: " + self.name[1]
            self.set_finished()
	else:
	    print "Unsuccessfully print name"
            self.set_failed("")





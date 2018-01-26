'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import random

import basebehavior.behaviorimplementation


class SubProbability_x(basebehavior.behaviorimplementation.BehaviorImplementation):
    '''this is a behavior implementation template'''

    def implementation_init(self):
        pass

    def implementation_update(self):
        self.name = self.m.get_last_observation("NameObject")

        if bool(random.getrandbits(1)):
            print '- ' * 30
            print self.name[1]
            print '- ' * 30
            self.set_finished()

        else:
            print '- ' * 30
            print 'Print the last value of Name from memory failed!'
            print '- ' * 30
            self.set_failed("")

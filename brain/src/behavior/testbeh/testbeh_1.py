

'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import basebehavior.behaviorimplementation


class TestBeh_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    '''this is a behavior implementation template'''

    #this implementation should not define an __init__ !!!


    def implementation_init(self):

        self.state = "idle";
        #define list of sub-behavior here
        pass

    def implementation_update(self):

        if (self.state == 'idle'):
            print 'idle'
            self.state = 's1'
        elif (self.state == 's1'):
            print 's1'
            self.state = 's2'
        elif (self.state == 's2'):
            print 's2'
            self.set_finished()

        #you can do things here that are low-level, not consisting of other behaviors

        #in this function you can check what behaviors have failed or finished
        #and do possibly other things when something has failed
        pass




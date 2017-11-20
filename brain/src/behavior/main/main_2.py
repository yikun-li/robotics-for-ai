

'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import basebehavior.behaviorimplementation


class Main_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    '''this is a behavior implementation template'''

    #this implementation should not define an __init__ !!!


    def implementation_init(self):

        #define list of sub-behavior here
        self.CreateSubPrintName()
        self.CreateSubTimer()
        self.CreateSubProbability()

        pass

    def implementation_update(self):

        #you can do things here that are low-level, not consisting of other behaviors

        #in this function you can check what behaviors have failed or finished
        #and do possibly other things when something has failed
        pass

    def CreateSubPrintName(self):
        self.subPrintName = self.ab.subprintname()

    def CreateSubTimer(self):    
        self.subTimer = self.ab.subtimer()

    def CreateSubProbability(self):
        self.subProbability = subprobability()



from abc import abstractmethod

from abc import ABCMeta

import memory
import allbehaviors
import body
import time
import os
import logging
import util.nullhandler
import bbie.bbie as bbie
import speech.testcontroller

logging.getLogger('Borg.Brain.Behavior.BehaviorImplementation').addHandler(util.nullhandler.NullHandler())


class BehaviorImplementation:
    __metaclass__ = ABCMeta


    def __init__(self,behavior_parent):
        self.logger = logging.getLogger('Borg.Brain.Behavior.BehaviorImplementation')
        self.list_contents = None
        self.selected_behaviors = []
        self.ab = allbehaviors.AllBehaviors()
        self.m = memory.Memory()
        self.body = body.bodycontroller.BodyController()
        self.speech = speech.testcontroller.TestController()

        #we store the instance of the abstract behavior, so we can use its functions:
        self._behavior_parent = behavior_parent

        self._behavior_parent.load_exceptions()

        #self.implementation_init()
        self.__init_called = False


    def update(self):
        '''if a behavior implementation is not using a list with preconditions and
        behaviors, it should implement (override) its own update function'''

        if not self.__init_called:
            self.implementation_init()
            
            self.__init_called = True

        #check preconditions of sub behaviors:
        if (self.selected_behaviors != []):

            for (behavior, preq) in self.selected_behaviors:
                #behavior is a string, so make it an object:
                command = "behavior_object = self." + behavior
                exec(command)

                #now we have an object called behavior_object that is our behavior
                if (not behavior_object.is_stopped()):
                    if (behavior_object.is_running()):
                        #do the next step for this behavior
                        behavior_object.update()
                    else:
                        #the behavior is not yet running, so we need to check the precondition:
                        if (self.evaluate_condition(preq)):
                            # Log the behavior start.
                            #DEBUG
                            print "Should log precon here!"
                            bbie.store_precon_in_csv(behavior_object.get_id(), behavior_object.get_name(), behavior_object.get_depth(), 0)
                            
                            #start the behavior!
                            behavior_object.update()


        #now we run possible code in the implementation:
        self.implementation_update()


    @abstractmethod
    def implementation_init(self):
        '''this method should be implemented by an implementation'''
        pass

    @abstractmethod
    def implementation_update(self):
        '''this method should be implemented by an implementation'''
        pass


    def check_exceptions(self):
        '''check if one or more of the exceptions of this behavior are valid'''
        #this is done by the abstract behavior, so call that one:
        return self._behavior_parent.check_exceptions()


    def evaluate_condition(self,condition):
        '''evaluate a condition in the context of the behavior'''

        #this memory might be used in the condition, so ignore netbeans warning!
        #it is not unused!
        m = self.m

        return eval(condition)

    def set_failed(self,reason):
        '''register that the behavior has failed'''
        #we register this on the abstract behavior, not the implementation
        self._behavior_parent.set_failed(reason)
        
    def set_finished(self):
        '''register that the behavior has finished'''
        #we register this on the abstract behavior, not the implementation
        self._behavior_parent.set_finished()

    def set_params(self, params):
        '''adds all parameters that are in the dictionary to this implementation object'''

        #TODO: check if a parameter is already set, if so, do not update it!

        #make a command for each parameter you want to set
        for key, value in params.items():
            if type(value) == str:
                command = "self.%s = '%s'" % (key, value)
            else: 
                command = "self.%s = %s" % (key, str(value))
            exec(command)


    def check_postcondition(self):
        '''check the postcondition of this behavior'''
        return self._behavior_parent.check_postcondition()

    def finished_behaviors(self):
        '''return a list of finished behaviors'''
        eval("self.examplewait1.is_finished()")
        return filter (lambda bh, val = self: eval ( "val." + bh[0] + ".is_finished()" ), self.selected_behaviors)

    def running_behaviors(self):
        '''return a list of running behaviors'''
        return filter (lambda bh, val = self: eval ( "val." + bh[0] + ".is_running()" ), self.selected_behaviors)

    def failed_behaviors(self):
        '''return a list if failed behaviors'''
        return filter (lambda bh, val = self: eval ( "val." + bh[0] + ".is_failed()" ), self.selected_behaviors)

    def get_all_exeptions_behaviors(self):
        '''get textual information about why the behaviors that failed failed'''
        return [(bh,bh.get_failure_reason()) for bh in self.failed_behaviors()]


from abc import abstractmethod

from abc import ABCMeta

import time
import inspect
import logging
import os

import bbie.bbie as bbie
import allbehaviors
import body.bodycontroller as body
import util.nullhandler

logging.getLogger('Borg.Brain.Behavior.AbstractBehavior').addHandler(util.nullhandler.NullHandler())

class AbstractBehavior:
    __metaclass__ = ABCMeta

    '''This is an abstract class for a behavior. A behavior can have multiple implementations,
    those implementations are in classes of the type BehaviorImplementation'''
    
    # Most recent author: Marko Doornbos (hylianhunter@gmail.com)

    def __init__(self, params):
        self.logger = logging.getLogger('Borg.Brain.Behavior.AbstractBehavior')
        self.name = self.get_name()
        self.selected_implementation = None
        self.ab = allbehaviors.AllBehaviors()
        self.preconditions = None
        self.params = params #the params are kept here, and set in the implementation in the select_implementation function
        self.body = body.BodyController()
        self._all_exceptions = []
        self._best_behaviors = None
        self._expected_time = None
        self._implementation_id = None
        self._num_tries = 0

        self._state = "waiting"
        self._failed_reason = ""
        self._stop_at_exception = False

        self._start_time = time.time()
        
        self._depth = 0


    def check_params(self):
        '''this method can be used to check if certain (obligatory) parameters
        are present. Could be implemented in a specific behavior'''
        pass


    @abstractmethod
    def get_name(self):
        '''this function should be implemented by each class that inherits from behavior setting
        the name of the behavior (not the implementation, so "goto", not "goto_1")
        this method should set self.name'''
        pass


    def update(self):
        '''update the currently chosen behavior, and check its postcondition'''

        if (self._state is "stopped"):
            #if we are stopped, do nothing
            return
        
        if (self._state is "finished"):
            #we are done, so store data and set to stopped.
            self.save_results()
            print "The behavior " + self.name + " has finished."
            self._state = "stopped"
            return
            
        if(self._state is "failed"):
            #if we failed, start a different version.
            
            #give the results to the machine learning:
            print "The behavior " + self.name + " has failed."
            print "Reason given: " + self._failed_reason
            self.save_results()
            
            if self.check_tries():
                # Unless we've tried too often already.
                print "Maximum number of tries for behavior " + self.name + " reached. Stopping behavior."
                self._state = "stopped"
                return
            
            self._state = "running"
            
            #start another version.
            self.select_implementation()
            self._start_time = time.time()
            return

        if (not self._state is "running"):
            #if we are not running, but also not finished, start
            self._start()

        #check if we have reached the goal of this behavior:
        if (self.check_postcondition()):
            self._state = "finished"
            return
        else:
            #check exceptions of this behavior:
            self.check_exceptions()
            
            #run the implementation update
            self.selected_implementation.update()

            #check if this behavior is running for to long already:
            duration = time.time() - self._start_time
            if (duration > self._expected_time):
                self._state = "failed"
                self._failed_reason = "The behavior timed out."

    
    def _start(self):
        '''start the behavior, choose the best implementation, and record
        its starting time'''
        self.check_params()
        self._start_time = time.time()
        self.select_implementation()
        self._state = "running"

    
    def save_results(self):
        '''save the results of the currently running implementation
        so that information can be used by select_implementation()'''

        duration = time.time() - self._start_time
        bbie.save_results(self._implementation_id, os.environ['BORG']+"/brain/src/bbie/ie_files/"+self.name.lower()+".pkl", duration, self.is_finished(), self.is_failed())
        
    def check_tries(self):
        '''check if the maximum number of tries for this behavior has been reached already. 
        Returns False if there have not been too many tries, True if there have.'''
        self._num_tries = self._num_tries + 1
        if bbie.check_tries(self._implementation_id, os.environ['BORG']+"/brain/src/bbie/ie_files/"+self.name.lower()+".pkl", self._num_tries):
            return True
        else:
            return False


    @abstractmethod
    def check_postcondition(self):
        '''this function has to be implemented in a derivation of this class
        and checks if we have reached the postcondition'''
        pass


    @abstractmethod
    def load_exceptions(self):
        '''(for now) individual behaviors should implement this method, we might
        want to make it general, and not abstract any more...'''
        #TODO: just implement this here?
        pass


    def check_exceptions(self):
        '''check whether one of the exceptions hold'''
        
        for exception in self._all_exceptions:
            if (self.evaluate_condition(exception)):
                self.set_failed(exception)
                if (self._stop_at_exception):
                    self._state = "failed"
                    return True
        
        return False


    def evaluate_condition(self, condition):
        '''evaluate a condition in the context of the behavior'''
        self.selected_implementation.evaluate_condition(condition)


    def stop_in_case_of_exception(self,value):
        '''this function can be used to tell the behavior that it should stop if
        an exception happens'''
        self._stop_at_exception = value


    def set_failed(self,reason):
        '''set wheter this behavior has failed'''
        self._state = "failed"
        self._failed_reason = reason

        
    def get_failure_reason(self):
        '''returns the reason why the behavior has failed'''
        return self._failure_reason

    def is_failed(self):
        '''returns whether this behavior has failed (not its subbehaviors)'''
        if self._state is "failed":
            return True
        else:
            return False
            
    def get_id(self):
        return self._implementation_id

    def set_finished(self):
        '''set this behavior to finished'''
        self._state = "finished"

    def is_stopped(self):
        '''we also need to have a stopped state, to prevent the behavior from being started
        again when the precondition holds after being stopped'''
        if self._state is "stopped":
            return True
        else:
            return False
            
    def set_stopped(self):
        self._state = "stopped"

    def is_running(self):
        '''returns whether this behavior is running'''
        if self._state is "running":
            return True
        else:
            return False

    def is_finished(self):
        '''returns whether this behavior is finished'''
        if self._state is "finished":
            return True
        else:
            return False

    def stop(self):
        '''stop the current implementation'''
        self._state = "stopped"

    def simple_name(self, name):
        return name.lower().replace('_', '')

    def get_classes(self, module):
        classes = []
        for i in dir(module):
            item = getattr(module, i)
            if inspect.isclass(item):
                classes.append(item)
        return classes      
    
    def find_class(self, module):
        own_name = self.simple_name(self.get_name())
        classes = self.get_classes(module)
        for c in classes:
            name = self.simple_name(c.__name__)
            name = name[:len(own_name)]
            if name == own_name:
                return c
        return None
        
    def is_behavior(self):
        return True
        
    def get_depth(self):
        return self._depth

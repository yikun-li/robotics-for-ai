import time
import logging

import nullhandler
import loggingextra

logging.getLogger('Borg.Brain.Util.Timer').addHandler(nullhandler.NullHandler())

class Timer(object):
    """
    The Timer class can be used to do timeout checks in various settings, mainly
    behaviors. The main usage will be the use of the class method check, which
    will obtain a Timer from the repository with the specified name and check
    if it finished yet. If the timer was not used before, it will be instantiated.
    The class can also be used standalone, in that case the methods _check and
    _reset should be used instead.
    """
    __registry = {}

    """
    Check whether the timer with the given name has finished yet. The duration
    should be provided to instantiate the timer if this had not been done
    before.
    """
    @classmethod
    def check(cls, name, duration):
        #print "Created timer: ", name
        t = cls.get(name, duration)
        return t._check()

    """
    Delete the timer from the registry. A next call to check will restart it.
    """
    @classmethod
    def reset(cls, name):
        if name in cls.__registry:
            del cls.__registry[name]

    """
    Obtain a timer with a given name. If the time does not exist, it will be
    instantiated.
    """
    @classmethod
    def get(cls, name, duration):
        if name not in cls.__registry:
            t = Timer(duration)
            cls.__registry[name] = t
        else:
            t = cls.__registry[name]
        return t

    """
    Remove timers from the registry that have not been used for a specific amount
    of time. The default for this time is 10 minutes. Should only be used when
    creating massive amounts of timers with various names. 
    """
    @classmethod
    def gc(cls, timeout=600):
        for name, timer in cls.__registry.iteritems():
            if timer.remain() < -timeout:
                del cls.__registry[name]

    """
    Create a new Timer object with a given time
    """
    def __init__(self, duration):
        self._reset(duration)

    """
    Reset the timer. If the duration is omitted, it will use the same duration
    as used when instantiating the Timer in the first place.
    """
    def _reset(self, duration=None):
        if duration:
            self.__duration = duration

        self.__start = time.time()
        self.__end = self.__start + self.__duration

    """
    Check if the timer has finished. It will return True if the timer has
    finished, and false if it has not finished.
    """
    def _check(self):
        return time.time() >= self.__end

    def remain(self):
        return time.time() - self.__end

################################################################################
## DRIVER                                                                     ##
################################################################################
## This is a simple demonstration of the Timer. It will instantiate a Timer   ##
## to wait for 1.15 seconds and loop until the timer is satisfied. It will    ##
## then reset the timer and repeat this procedure for 5 times in total.       ##
################################################################################
if __name__ == "__main__":
    for i in range(5):
        while not Timer.check("behavior.test", 1.15):
            print "."
            time.sleep(0.05)
        Timer.reset("behavior.test")
        if i < 4:
            print "Resetting timer, iteration %d" % (i + 1)
    print "Done"

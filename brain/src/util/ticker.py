import time
import logging

import nullhandler
import loggingextra

logging.getLogger('Borg.Brain.Util.Ticker').addHandler(nullhandler.NullHandler())

class Ticker(object):
    """
    The Ticker class can be used to have a loop run on a certain frequency or
    duration. You should pass the constructor the frequency or the ticktime by
    specifying either the first argument for frequency, or the keyword argument
    ticktime as the time each tick should last, measured in seconds.

    Usage is simple. Instantiate the Ticker by providing the required frequency.
    Then, in the loop, start with a ticker.start_tick() command, and as the
    final statement, call ticker.end_tick().

    Another option is to combine these two into a single call to ticker.tick()
    at the beginning of the loop.

    It is possible to use a function different from time.sleep to perform the
    actual sleep: you can pass a function as the sleep argument to either the
    tick() or the end_tick() method. See the end_tick() function for more
    details.
    """
    def __init__(self, frequency=5, ticktime=None, name="Ticker", verbose=False, ignore_delay=False):
        """
        The constructor sets the frequency or the tick time, and
        optionally defines a name for the ticker.
        """
        self.logger = logging.getLogger('Borg.Brain.Util.Ticker')
        self.__ticktime = 0.2
        self.__name = name
        self.__ignore_delay = ignore_delay
        if frequency:
            self.set_frequency(frequency)
        elif ticktime:
            self.set_ticktime(ticktime)
        self.__tickend = time.time() + self.__ticktime
        self.__verbose = verbose
        self.__tick_list_size = 100
        self.__check_tick_time = 0
        self.__num_ticks = 0
        self.__ticks_on_time = 0
        self.__sleep_func = None

    def set_name(self, name):
        """
        Set the name for this ticker, which is used in the logging messages"
        """
        self.__name = str(name)

    def set_frequency(self, frequency):
        """
        Sets the tick time as a frequency in hertz, by calculating the time a
        tick lasts with that frequency
        """
        if not frequency:
            self.__ticktime = None
        else:
            self.__ticktime = float(1.0 / float(frequency))

    def get_frequency(self):
        """
        Return the frequency in hertz, by performing the inverse
        operation to get the frequency back
        """
        if not self.__ticktime:
            return None
        return round(1.0 / self.__ticktime, 1)
    
    def set_ticktime(self, ticktime):
        """
        Sets the tick time as a duration in seconds directly
        """
        self.__ticktime = float(ticktime)

    def get_ticktime(self):
        """
        Returns the the time each tick should last
        """
        return self.__ticktime

    def tick(self, sleep=True, *args, **kwargs):
        """This method ends previous tick and starts the next"""
        result = self.end_tick(sleep, *args, **kwargs) # End previous tick
        self.start_tick() # Start new tick
        return result
        
    def start_tick(self):
        """
        This method should be called at the start of a tick, so that the
        prefered ending time can be calculated
        """
        if not self.__ticktime:
            return
        self.__tickend = time.time() + self.__ticktime

    def tick_remain(self):
        """
        This function returns the time remaining in the current tick.
        """
        remain = self.__tickend - time.time()
        self.__num_ticks += 1
        return max(0, remain)

    def end_tick(self, sleep=True, *args, **kwargs):
        """
        This method should be called at the end of a tick. If no time remains,
        this does nothing. If the end of the tick has already passed, a warning
        is emitted signalling that the code using the ticker takes too long to
        complete to achieve the set frequency.  If time does remain, this method
        sleeps until the end of the tick.

        By default, this method will use time.sleep to do the sleeping.
        Optionally, the sleep argument may be used to specify a sleeping
        function to use instead. The rest of the arguments will be passed
        to this function. If there is a keyword argument named timeout,
        the remaining time will be passed to the function as this argument,
        otherwise the remaining time will be passed as the first argument. If
        the sleep argument evaluates to False, the remaining time in the tick
        will be returned instead of slept.

        If the default time.sleep function is used, the time slept will
        be returned by this function. Otherwise, the result of the sleep
        function provided will be returned instead.
        """
        if not self.__ticktime:
            return
        remain = self.__tickend - time.time()
        self.__num_ticks += 1
        result = max(0, remain)
        if remain >= 0:
            if self.__verbose:
                self.logger.debug("[%s] Sleeping for %5f seconds" % \
                                  (self.__name, remain))
            self.__ticks_on_time += 1
            if sleep:
                if sleep is True:
                    time.sleep(remain)
                else:
                    if "timeout" in kwargs:
                        kwargs['timeout'] = remain
                        result = sleep(*args, **kwargs)
                    else:
                        result = sleep(remain, *args, **kwargs)
        
        self.check_tick_speed()
        return result

    def check_tick_speed(self):
        if self.__check_tick_time and time.time() >= self.__check_tick_time:
            # Check tick speed every 5 seconds
            self.__check_tick_time = time.time() + 30
            delayed = self.__num_ticks - self.__ticks_on_time
            if not self.__ignore_delay:
                max_delayed = self.__num_ticks / 2.0
                if delayed >= max_delayed: 
                    self.__ticksontime = []
                    hertz = self.get_frequency()
                    avg_tick_time = 30.0 / self.__num_ticks
                    avg_hertz = 1.0 / avg_tick_time
                    self.logger.warn("[%s] Average tick time over past 30 "     \
                                     "seconds: %f (= %f Hertz); set tick "      \
                                     "time: %f (= %f Hertz)"                    \
                                     % (self.__name, avg_tick_time, avg_hertz,
                                        self.__ticktime, hertz))
                    if hertz > 2:
                        self.logger.warn("[%s] Unable to achieve requested "   \
                                         "frequency of %.1f Hz. This is "      \
                                         "either because the machine is too "  \
                                         "slow or because of too long "        \
                                         "timeouts in other parts of the "     \
                                         "code. Slowing down by 5%%."          \
                                         % (self.__name, hertz))
                        self.set_frequency(hertz * 0.95)
                    else:
                        self.logger.warn("[%s] Unable to achieve requested "   \
                                         "frequency of %.1f Hz. This is "      \
                                         "either because the machine is too "  \
                                         "slow or because of too long "        \
                                         "timeouts in other parts of the "     \
                                         "code. " % (self.__name, hertz))

            # Reset tick counters
            self.__num_ticks = self.__ticks_on_time = 0
        if self.__check_tick_time == 0:
            self.__check_tick_time = time.time() + 30

################################################################################
## DRIVER                                                                     ##
################################################################################
## This is a simple demonstration of the Ticker. It will instantiate a ticker ##
## at a frequency of 10 Hertz and call the tick method in a loop. At each     ##
## iteration, a print will be made. The program will continue until it is     ##
## interrupted by the user or by a signal.                                    ##
################################################################################
if __name__ == "__main__":
    logging.getLogger('Borg.Brain.Util').addHandler(loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain.Util').setLevel(logging.DEBUG)
    ticker = Ticker(frequency=10, verbose=True)
    try:
        while True:
            ticker.tick()
            time.sleep(0.02)
            print "Tick"
    finally:
        print "Done"

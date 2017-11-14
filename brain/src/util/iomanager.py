import threading
import time
import logging
import sys
import copy
import atexit
import os
import fcntl
import abc
import errno
import traceback

import nullhandler
import loggingextra
import iopoll
import signal
from ticker import Ticker

"""
################################################################################
## The IOManager                                                              ##
################################################################################
The _IOManager class is a class that is used to manage objects that wrap file
descriptors in some way, such as network sockets, pipes, pty's and files.

The _IOManager class should never be instantiated directly but should always be
accessed through the use of the IOManager() function that makes it sort-of
singleton. The reason the __new__ method is not used instead is that that
occasionally gives problems when registering new objects, due to the threaded
nature of the object.

The _IOManager runs in a loop, polling all the registered file descriptors.  For
each file descriptor, functions are called to notify them that there is data to
read or write.

For the polling, the _IOPoll class is used. This is a wrapper around the
different polling mechanisms available on different platforms. These include
select, poll, epoll and kqueue/kevent. If the poller supports it, edge triggerd
behavior will be enabled on it. This also slightly changes the behavior of the
IOManager. If Edge Triggered behavior is enabled, for each file descriptor for
which an IO event is received, its handle_read or handle_write method will be
called continously until a IOWrapperEnd exception is caught. When this happens,
the file descriptor should not have any more data to read or write, to reset the
file descriptor in the epoll object. This implies that all reading and writing
on file descriptors in the handle_read and handle_write methods should be done
in a non-blocking way, otherwise the last call to those methods will block
instead of throwing the IOWrapperEnd exception.

Access to the IOPoll object and the lists of registered IOWrappers is
synchronized using a lock. Mostly, the lock will acquired by the poll method of
the IOPoll object. This means that if a new wrapper is registered, or if new
data is available to write, it has to wait for the lock to be released before
the IOPoll

################################################################################
## IOWrapper requirements                                                     ##
################################################################################
Each file descriptor should be wrapper in a subclass of the IOWrapper class,
which enforces a set of methods to be available on the object.

In addition to the _handle_read and _handle_write methods, a _handle_attempt and
_handle_loop method should also be avabile. The _handle_attempt is called for
registered wrappper objects that are not currently connected. This is useful
mainly for network sockets that try to connect to a remote host, or server
sockets that cannot currently bind to their port because the port is taken by
another process or is otherwise unavailable.

The handle_loop function is called around 10 times per second, and allows the
IOWrappers to perform some action on regular intervals.

Most importantly, each wrapper should define a fileno() method that returns the
file descriptor belonging to the object. This is used to register it in IOPoll
and to map any events to the correct wrapper.

Other functions that should be defined are close() to close the file descriptor
and unregister it from the IOManager, connected() that should return whether the
IOWrapper is connected/active (meaning that the fileno() method should return an
open file descriptor).

Current implementations of IOWrapper subclasses are ThreadedSocket and
ThreadedPipe. The first wraps a network socket, either client or server.  The
ThreadedPipe object wraps pipes in a non-blocking way.

Usually, you only need to concern yourself with IOManager when you are writing
or modifying a new IOWrapper, for example for reading files.

################################################################################
## Error handling                                                             ##
################################################################################
IOManager should be very stable because it will/can manage all IO of the system
and not just a single socket or file. Therefore, all calls to IOWrapper
instances are wrapped in a try/except block that unforgivingly removes the
wrapper from the IOManager if they result in an exception. It will generate a
CRITICAL message in the logger to inform the user about this. The only exception
is the IOWrapperEnd exception that should be raised by the _handle_read and
_handle_write methods of the IOWrapper class when there is no more data to read
or when no more data can be written.

Do take care that your subclasses/implementations do not generate exceptions or
the wrappers will be closed and removed from the wrapper.

################################################################################
## Profiling                                                                  ##
################################################################################
Sometimes it can be handy to profile the IOManager to see where delays in
network traffic are coming from. Under normal circumstances, most time should be
spent in the poll method from the IOPoll object; waiting for new IO to handle.
Profiling will enable profiling in the IOManager thread, so this will include
all function calls to the IOWrapper objects. This might indicate bottlenecks in
implementations.

To enable profiling, set the profile attribute of the _ConnManager class to the
filename to which to writing profiling information. For example:

_ConnManager.profile = "~/iomanager.prof"

This will store the profiling data to iomanager.prof in your home directory.
This profiling data can then be read using the pstats package. See the Python
documentation for more information, it's at:

http://docs.python.org/library/profile.html

################################################################################
## Stopping conditions: signals, timeouts and threads                         ##
################################################################################
Any IOWrapper will at some point register itself with the IOManager by obtaining
an instance with the IOManager() function, that will instantiate and start a new
IOManager if needed. IOManager will also by default catch any signals the
process receives and react on those by shutting down the IOManager and closing
all the open file descriptors. Note that for this a shutdown flag is used to
avoid starting a new IOManager when signals have been caught. This makes sure no
new instance is started. If you want to handle signals in another way, you can
overwrite the signal handlers of IOManager and make sure the IOManager signal
handler is not called for that signal. If the IOManager is catching a signal and
you want to restart IOManager afterwards, you will have to set the shutdown flag
(iomanager.__iom_shutdown) to False afterwards.

When IOManager is running and no file descriptors have been registered for over
30 seconds, it will shut itself down to save processing cycles. As soon as any
IOWrapper tries to register itself with the IOManager, a new instance will be
registered.

IOManager also monitors the thread that started the IOManager in the first place
and will terminate as soon as that thread dies. This could have a undesired
effect if the IOManager is initially started from a thread that is not the main
thread of the program. If you are experiencing this in your code, a simple
solution is to initially start the IOManager from your main thread. You can
simply do this by putting the following code somewhere in your main thread,
before any IOWrapper is instantiated in any thread:

from util.iomanager import IOManager
IOManager()
"""

_iom_instance = None
_iom_shutdown = False

def IOManager():
    """
    This method returns the currently active IOManager instance, or creates
    a new one if none exists
    """
    global _iom_instance
    global _iom_shutdown

    if _iom_instance:
        return _iom_instance

    if _iom_shutdown or _iom_shutdown is None:
        raise Exception("Should not call IOManager after shutdown")

    _iom_instance = _IOManager()
    _IOManager.set_signal_handlers([signal.SIGINT, signal.SIGTERM, signal.SIGHUP,  
                                      signal.SIGPIPE, signal.SIGSEGV, signal.SIGBUS, 
                                      signal.SIGILL, signal.SIGABRT, signal.SIGFPE])
    return _iom_instance

def clear_IOM():
    """
    This method clears the IOManager instance currently active, if any
    """
    global _iom_instance
    if _iom_instance:
        _iom_instance.stop()
        if threading.current_thread() != _iom_instance:
            _iom_instance.join()

    _iom_instance = None
    _IOManager.restore_signal_handlers()

def __atexit_handler():
    """
    This handler will stop the IOManager on interpreter exit. It
    will set the shutdown flag to True and run clear_IOM.
    """
    global _iom_shutdown
    _iom_shutdown = True
    clear_IOM()

atexit.register(__atexit_handler)

class _IOManager(threading.Thread):
    """
    ############################################################################
    ## The IOManager                                                          ##
    ############################################################################
    The _IOManager class is a class that is used to manage objects that wrap
    file descriptors in some way, such as network sockets, pipes, pty's and
    files.

    The _IOManager class should never be instantiated directly but should
    always be accessed through the use of the IOManager() function that makes
    it sort-of singleton. The reason the __new__ method is not used instead is
    that that occasionally gives problems when registering new objects, due to
    the threaded nature of the object.

    The _IOManager runs in a loop, polling all the registered file descriptors.
    For each file descriptor, functions are called to notify them that there is
    data to read or write.
    """
    __instance = None
    __signal_handlers = {}
    profile = None

    #######################################################################
    # Class Methods
    #######################################################################
    # The following methods will register and unregister signal handlers
    # and handle the signals themselves.
    #######################################################################
    @classmethod
    def signal_handler(cls, signum, frame):
        """
        This method can be registered as a signal handler and will terminate
        the IOManager thread on the signal and call the original signal
        handler.
        """
        global _iom_instance

        # Find name of signal
        signame = str(signum)
        for key in signal.__dict__.keys():
            if key.startswith("SIG") and getattr(signal, key) == signum:
                signame = key
                break

        try:
            logger = _iom_instance._IOManager__logger
            logger.warning("Caught signal %s. Terminating IOManager" % signame)
        except:
            print "Caught signal %s. Terminating IOManager" % signame

        original_handler = None
        if signal in cls.__signal_handlers:
            original_handler = cls.__signal_handlers[signal]

        clear_IOM()

        if original_handler:
            original_handler(signal, frame)
        else:
            sys.exit(1)

    @classmethod
    def set_signal_handlers(cls, signals):
        """
        This method is called whenever a IOManager is instantiated. It sets
        the signal_handler to be the handler for a set of signals to stop the
        thread when they arrive. 
        """
        for sig in signals:
            try:
                original_handler = signal.getsignal(sig)
                if original_handler == cls.signal_handler:
                    continue
                signal.signal(sig, cls.signal_handler)
                cls.__signal_handlers[sig] = original_handler
            except Exception as e:
                pass

    @classmethod
    def restore_signal_handlers(cls):
        """
        This method is called when a IOManager thread ends to restore the
        original behavior.
        """
        signals = cls.__signal_handlers.keys()
        for sig in signals:
            try:
                signal.signal(sig, cls.__signal_handlers[sig])
            except Exception as e:
                pass
        cls.__signal_handlers = {}

    #######################################################################
    # Constructor and destructor
    #######################################################################
    def __del__(self):
        """
        Stop the IOManager thread when the object is deleted.
        """
        self.stop()

    def __init__(self):
        """
        Set up the IOManager thread and start it.
        """

        super(_IOManager, self).__init__()
        self.__poll = iopoll.IOPoll()
        self.__et = self.__poll.set_edge_triggered(True)
        self.__wrappers = {}
        self.__disconnected_wrappers = []
        self.__running = True

        self.__wrapper_lock = threading.RLock()
        self.__poll_lock = threading.RLock()
        self.__logger = logging.getLogger('Borg.Brain.Util.IOManager')
        self.__logger.addHandler(nullhandler.NullHandler())
        self.__empty_time = time.time() + 10
        self.__next_tick = time.time() + 0.1
        self.__parent_thread = threading.current_thread()

        self.__read_list = set([])
        self.__write_list = set([])

        self.__ticker = Ticker(10)
        self.__saved_time = 0

        # Set up wake up pipe in non-blocking mode
        self.__wakeup_read, self.__wakeup_write = os.pipe()
        fl = fcntl.fcntl(self.__wakeup_read, fcntl.F_GETFL)
        fcntl.fcntl(self.__wakeup_read, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        fl = fcntl.fcntl(self.__wakeup_write, fcntl.F_GETFL)
        fcntl.fcntl(self.__wakeup_write, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        self.__poll.register(self.__wakeup_read, True, False, True)

        # Start IOManager thread
        self.start()

    ############################################################################
    ## IOManager thread                                                       ##
    ############################################################################
    def run(self):
        """
        This method will be called when the thread starts. It wraps the _run
        method to make it possible to profile the IOManager.
        """
        if not self.__class__.profile is None:
            import cProfile
            cminstance = self
            cProfile.runctx('self._run()', globals(), locals(), self.__class__.profile)
        else:
            self._run()

    def _run(self):
        """
        This method is the main thread of the IOManager.  It polls all active
        IOWrappers and calls their read and write handlers when data can be read or
        written.  It will also try to connect any unconnected IOWrappers that have been
        registered.
        """
        global _iom_instance
        self.__logger.info("IOManager started")
        timeout = 0
        while self.__running:
            # Poll for events
            if self.__et:
                self.__poll_et()
            else:
                self.__poll_lt()

            # Perform IOWrapper checking on the actual
            # frequency of the ticker, not each time
            # there is data to read or write
            if time.time() >= self.__next_tick:
                self.__next_tick += self.__ticker.get_ticktime()
                # Call the handle loop function of each connected wrapper
                self.__handle_loop()
                # Attempt to connect all IOWrappers
                self.__attempt_wrappers()
                # Check if IOManager should keep running
                self.__check_stop()
                # Allow for control to be interrupted if other threads require
                # attention. Do not waste time on this so use a very short sleep
                time.sleep(0.0001)

        self.__cleanup()
        clear_IOM()
        self.__logger.debug("Savings of wakeup pipe: %f seconds" % self.__saved_time)
        self.__logger.info("IOManager stopped")

    ############################################################################
    ## Helper methods                                                         ##
    ############################################################################
    ## The following methods are (private) helper methods for the IOManager,  ##
    ## handling events and validating wrappers.                               ##
    ############################################################################
    def __poll_lt(self):
        """
        Helper method for the main thread. Process all events from the poller:
        call the respective read and write handlers. This version of the method
        is for Level Triggered behavior, such as kqueue and poll. It will call
        the read/write handlers of each FD that is ready once. 
        """
        timeout = self.__ticker.end_tick(False)
        self.__last_tick_end = time.time() + timeout
        with self.__poll_lock:
            events = self.__poll.poll(timeout * 1000)

        self.__ticker.start_tick()

        # Process events on the IOWrappers
        for fd, event in events:
            # If this was the wake-up pipe, skip 
            if self.__check_wakeup(fd, len(events)):
                continue

            wrapper = self.__find_wrapper(fd)
            if not wrapper:
                continue

            if event & iopoll.IO_ERROR:
                self.__wrap_function(wrapper, "close")
            if event & iopoll.IO_READ:
                self.__wrap_function_io(wrapper, "_handle_read")
            if event & iopoll.IO_WRITE:
                self.__wrap_function_io(wrapper, "_handle_write")

    def __poll_et(self):
        """
        Helper method for the main thread. Process all events from the poller:
        call the respective read and write handlers. This version of the method
        is for Edge Triggered behavior, when using epoll. It will maintain a
        set of file descriptors ready for reading and writing. 

        On each loop, it will check if those sets are empty. If they are, a call
        to the poll function is made with the timeout returned by the ticker, to
        obtain the set frequency. If the sets are not empty, the lists are
        quickly updated by calling poll with a timeout of 0. The ready lists are
        updated with the events returned by the call to poll.
        """
        if self.__read_list or self.__write_list:
            timeout = 0
        else:
            timeout = self.__ticker.end_tick(False)

        self.__last_tick_end = time.time() + timeout
        with self.__poll_lock:
            events = self.__poll.poll(timeout * 1000)

        self.__ticker.start_tick()
        # Update the lists for reading and writing, and
        # remove/close the file descriptors with an 
        # error state.
        error_list = set([])
        for fd, event in events:
            if fd in error_list:
                continue

            if self.__check_wakeup(fd, len(events)):
                continue

            if event & iopoll.IO_ERROR:
                self.__read_list.discard(fd)
                self.__write_list.discard(fd)
                error_list.add(fd)
                wrapper = self.__find_wrapper(fd)
                if wrapper:
                    self.__wrap_function(wrapper, "close")
                continue
            if event & iopoll.IO_READ:
                self.__read_list.add(fd)
            if event & iopoll.IO_WRITE:
                self.__write_list.add(fd)

        # Perform IO
        self.__perform_io_et(self.__ticker.end_tick(False) * 0.9)

    def __perform_io_et(self, max_time):
        """
        After polling and updating of the ready lists has completed the function
        iterates over the file descriptors ready for reading and writing. The
        _handle_read and _handle_write of each file descriptor wrapper will be
        called round-robin in a loop, for as long as the current tick lasts.
        If the file descriptors become non-readable or non-writable before this
        happens, the function simply exits and on will continue in the next
        loop.
        """
        # Perform IO on ready file descriptors
        start = time.time()
        while time.time() < start + max_time:
            if not self.__read_list and not self.__write_list:
                break

            # Handle file descriptors ready for reading
            read_fds = sorted(self.__read_list)
            for fd in read_fds:
                wrapper = self.__find_wrapper(fd)
                if not wrapper:
                    self.__read_list.discard(fd)
                    continue
                try:
                    self.__wrap_function_io(wrapper, '_handle_read')
                except IOWrapperEnd:
                    self.__read_list.discard(fd)

            # Handle file descriptors ready for writing
            write_fds = sorted(self.__write_list)
            for fd in write_fds:
                wrapper = self.__find_wrapper(fd)
                if not wrapper:
                    self.__write_list.discard(fd)
                    continue
                try:
                    self.__wrap_function_io(wrapper, '_handle_write')
                except IOWrapperEnd:
                    self.__write_list.discard(fd)

    def __check_wakeup(self, fd, num):
        """
        This method checks if the fd is the wakeup pipe, and if so, reads
        the data from that pipe. 
        """
        if fd == self.__wakeup_read:
            if num == 1:
                # Was woken up, count savings. self.__last_tick_end
                # if the time the poll would have ended if it was
                # not awoken by the wakeup pipe.
                self.__saved_time += self.__last_tick_end - time.time()
            try:
                # Empty pipe
                while True:
                    os.read(self.__wakeup_read, 1)
            except OSError as err:
                if err.errno == errno.EAGAIN:
                    # Resource Temporarily Unavailable, occurs when there is 
                    # no more data to read. Normal behavior.
                    return True
                # Other error, something else is wrong. Should not happen.
                raise
        return False

    def __handle_loop(self):
        """
        This method calls the handle_loop function of each wrapper, once every
        true tick. This should be at the frequency of the ticker, about 10
        times per second.
        """
        with self.__wrapper_lock:
            for wrapper in self.__wrappers.values():
                self.__wrap_function(wrapper, "_handle_loop")

    def __wrap_function(self, wrapper, funcname):
        """
        This method calls a function on the wrapper in a try/except block. This
        is to make sure the IOManager will keep on running even when
        exceptions occur in one of the wrappers. This will prevent from the
        complete system to collapse when one IOWrapper fails.
        """
        try:
            function = getattr(wrapper, funcname)
            return function()
        except Exception as err:
            self.__handle_exception(wrapper, funcname, err)
            return None

    def __wrap_function_io(self, wrapper, funcname):
        """
        This method calls a function on the wrapper in a try/except block. This
        is to make sure the IOManager will keep on running even when
        exceptions occur in one of the wrappers. This will prevent from the
        complete system to collapse when one IOWrapper fails.

        The difference with this function and __wrap_function is that this one
        is aimed at the _handle_read and _handle_write methods. It will catch
        the IOWrapperEnd exception and raise it again, to indicate that the
        fd can no longer be read or written.
        """
        try:
            function = getattr(wrapper, funcname)
            return function()
        except IOWrapperEnd:
            # Expected exception when no more data can be read or written
            if self.__et: # poll_et needs this to update ready list
                raise
            else:         # poll_lt doesn't need this
                return False
        except Exception as err:
            self.__handle_exception(wrapper, funcname, err)
            return None

    def __handle_exception(self, wrapper, method, exception):
        """
        This method is called whenever an exception has occured in any of the
        wrappers functions. It outputs the message and removes the wrapper from
        the IOManager.
        """
        # First print the normal exception output
        #traceback.print_exc()

        self.__logger.critical("Exception raised in %s in IOWrapper %s. " \
                               "Removing IOWrapper from IOManager. The "  \
                               "exception is: %s"                         \
                               % (repr(wrapper), method, repr(exception)))
        fd = None
        # Get fd from object
        if method != "fileno":
            try:
                fd = wrapper.fileno()
            except:
                pass

        if not fd:
            # Look through the list of wrappers to find the wrapper
            for cur_fd, fd_wrapper in self.__wrappers.iteritems():
                if wrapper is fd_wrapper:
                    fd = cur_fd
                    break
                
        # Remove the wrapper from the lists
        with self.__wrapper_lock:
            if fd:
                if fd in self.__wrappers:
                    del self.__wrappers[fd]
                    
            if wrapper in self.__disconnected_wrappers:
                self.__disconnected_wrappers.remove(wrapper)
            

        # If the exception was not raised in the close method,
        # try to gracefully close the file descriptor. Otherwise, 
        # let garbage collection clean it up.
        if method != "close":
            try:
                wrapper.close()
            except:
                pass

    def __attempt_wrappers(self):
        """
        Helper method for the main thread. Attempt to connect unconnected
        wrappers
        """
        with self.__wrapper_lock:
            sock_list = self.__disconnected_wrappers

        for wrapper in sock_list:
            if not self.__wrap_function(wrapper, "connected"):
                self.__wrap_function(wrapper, "_handle_attempt")

    def __check_stop(self):
        """
        Check if there are any registered IOWrappers, and if not, stop running
        after 30 seconds.  Also stop if the parent thread ended to make sure the
        interpreter does not keep running just for the IOManager.
        """
        if not self.__parent_thread.is_alive():
            global _iom_shutdown
            self.__logger.info("Parent thread ended. Stopping IOManager.")
            _iom_shutdown = True
            self.__running = False

        if not self.__wrappers and not self.__disconnected_wrappers and time.time() > self.__empty_time:
            self.__logger.info("No IOWrappers registered. Stopping IOManager")
            self.__running = False
        elif self.__wrappers or self.__disconnected_wrappers:
            self.__empty_time = time.time() + 30

    def __cleanup(self):
        """
        This method will close all open file descriptor wrappers, and clean up
        the lists of the IOManager.
        """
        wrappers = copy.copy(self.__wrappers)

        num = len(wrappers)
        for fd in wrappers:
            wrappers[fd].close()

        self.__wrappers = {}
        self.__disconnected_wrappers = []
        self.__logger.info("Closed %d IOWrappers" % num)
        os.close(self.__wakeup_read)
        os.close(self.__wakeup_write)

    def __find_wrapper(self, fd):
        """
        Return the file descriptor wrapper object belonging to the specified file descriptor
        """
        with self.__wrapper_lock:
            if fd in self.__wrappers:
                return self.__wrappers[fd]
            with self.__poll_lock:
                if self.__poll.is_known(fd):
                    self.__logger.warning("Cannot find wrapper object belonging to " \
                                          "file descriptor %d. Unregistering." % fd)

                    # Try to unregister with poller
                    self.__poll.unregister(fd)

            return None

    def __validate_wrapper(self, wrapper):
        """
        This method validates the wrapper to be an instance of the IOWrapper
        base class, making sure it implements the required methods. It will
        raise an exception if the wrapper is invalid.
        """
        if not isinstance(wrapper, IOWrapper):
            raise Exception("Only subclasses of IOWrapper should be registered in the IOManager")

    def __wakeup(self):
        """
        This method wakes up the current call to poll by writing a zero byte to
        the wake-up pipe. This will wake the poller and increase the speed at
        which the lock is released. 
        """
        os.write(self.__wakeup_write, "\x00")

    ############################################################################
    ## Public API                                                             ##
    ############################################################################
    ## Functions that can and should be called by IOWrapper subclasses        ##
    ## to register and unregister themselves with the IOManager               ##
    ############################################################################
    def register(self, wrapper):
        """
        Register a connected file descriptor wrapper
        """
        self.__validate_wrapper(wrapper)
        fd = self.__wrap_function(wrapper, "fileno")
        if not type(fd) is type(0):
            self.__logger.error("Cannot register IOWrapper with file descriptor %s" % repr(fd))
            return False
        with self.__wrapper_lock:
            if fd in self.__wrappers:
                self.__logger.error("File descriptor %d already registered in IOManager" % fd)
                return False

            self.__logger.debug("Registering IOWrapper with file descriptor %d" % fd)
            if wrapper in self.__disconnected_wrappers:
                self.__disconnected_wrappers.remove(wrapper)

            self.__wrappers[fd] = wrapper

            self.__wakeup()
            with self.__poll_lock:
                self.__poll.register(fd, True, False, True)
            return True

    def register_unconnected(self, wrapper):
        """
        Register a disconnected file descriptor wrapper that needs its
        _attempt method to be called at a regular interval.
        """
        self.__validate_wrapper(wrapper)
        with self.__wrapper_lock:
            self.__logger.debug("Registering disconnected IOWrapper")
            self.__disconnected_wrappers.append(wrapper)
            return True

    def unregister(self, wrapper):
        """
        Unregister a file descriptor wrapper, usually because it has been
        disconnected or closed.
        """
        self.__validate_wrapper(wrapper)
        found = False
        fd = self.__wrap_function(wrapper, "fileno")
        if not type(fd) is type(0):
            self.__logger.error("Cannot unregister IOWrapper with file descriptor %s" % repr(fd))
            return False

        with self.__wrapper_lock:
            self.__wakeup()
            self.__read_list.discard(fd)
            self.__write_list.discard(fd)
            with self.__poll_lock:
                if fd in self.__wrappers:
                    found = True
                    self.__logger.debug("Unregistering IOWrapper with file descriptor %d" % fd)
                    del self.__wrappers[fd]

                if found:
                    self.__poll.unregister(fd)
                return found

    def unregister_unconnected(self, wrapper):
        """
        Unregister an unconnected file descriptor wrapper, usually because it
        has succesfully opened/connected or because it is no longer needed.
        """
        self.__validate_wrapper(wrapper)
        with self.__wrapper_lock:
            if wrapper in self.__disconnected_wrappers:
                self.__disconnected_wrappers.remove(wrapper)
                return True
            else:
                self.__logger.warning("Cannot unregister unregistered IOWrapper " \
                                      "%s" % repr(wrapper))
                return False

    def set_writable(self, wrapper, writable):
        """
        Marks the file descriptor as writable in the poller
        """
        self.__validate_wrapper(wrapper)
        fd = self.__wrap_function(wrapper, "fileno")
        if type(fd) is type(0):
            self.__wakeup()
            with self.__poll_lock:
                try:
                    self.__poll.modify(fd, True, writable, True)
                except IOError as e:
                    if e.errno == errno.EBADF:
                        self.__logger.warning("Invalid File Descriptor %d in " \
                                              "%s. Closing IOWrapper."         \
                                              % (fd, str(wrapper)))
                        self.__wrap_function(wrapper, "close")
                    else:
                        raise
            return True
        else:
            self.__logger.error("Cannot modify IOWrapper with file descriptor %s" % fd)
            return False

    def stop(self):
        """
        Stop the IOManager. This will set the running flag to false, which
        should terminate the IOManager thread in a short timespan.
        """
        self.__running = False

    def running(self):
        return self.__running

class IOWrapperEnd(Exception):
    pass

class IOWrapper(object):
    """
    The IOWrapper class is a class that wraps an object with a file descriptor,
    that can be used with the IOManager class.  It defines a set of abstract
    methods that should be implemented and a set of methods that can be
    implemented, optionally.
    """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def close(self):
        """
        This method should close the file descriptor.
        """
        pass

    @abc.abstractmethod
    def fileno(self):
        """
        This method should return the file descriptor associated with the
        wrapper, to be registered in the IO Poll object.
        """
        return None

    @abc.abstractmethod
    def connected(self):
        """
        This method should return True when the wrapper has an open file
        descriptor, and False when it's not connected.
        """
        return False

    @abc.abstractmethod
    def _handle_read(self):
        """
        This method will be called whenever the file descriptor is readable.
        """
        pass

    @abc.abstractmethod
    def _handle_write(self):
        """
        This method will be called whenever the file descriptor is writable and
        has been set to be writable using the set_writable method of the
        IOManager.
        """
        pass

    def _handle_loop(self):
        """
        This method will be called when the connected method returns true, and
        the class has registered itself using the register method in the
        IOManager.
        """
        pass

    def _handle_attempt(self):
        """
        This method will be called at regular intervals when the wrapper has
        registered itself using the register_unconnected method of the
        IOManager and the connected method returns False.
        """
        pass

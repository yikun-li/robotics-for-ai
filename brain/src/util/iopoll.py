import select
import errno
import logging
import loggingextra
import nullhandler

"""
################################################################################
## IOPoll class                                                               ##
################################################################################
IOPoll is a wrapper around several different polling mechanisms available on
different platforms. These mechanisms poll for events on file descriptors and
notify the user when a file descriptor is readable or writable.

Select is available on almost all platforms, but has severe performance issues,
especially with large sets of file descriptors. All file descriptors must be
passed on each poll using select. Therefore, select is available but used only
as a fallback for platforms on which no better alternative is available.

Linux provides the poll interface, with which a set of file descriptors can be
registered. These file descriptors will be polled each time the poll function is
called. Since the file descriptors are registered within the kernel, this
provides great performance benefits.

Since kernel 2.5.44, a new interface is available: epoll. This is a faster
version of poll and also allows the use of Edge Triggered polling. This means
that events are only reported once: if data is available to read, the poll will
return this once and then will not report this again until there is new data
available to read. This means that after the file descriptor has been reported
readable or writable, you have to exhaust the IO buffers and keep reading or
writing until all data has been read or written (and EAGAIN occurs), before new
events will be reported.

On BSD derivatives, kernel queues are available (kqueue). On these queues,
kernel events can be registered and these events will report readability or
writability of file descriptors.

To simplify the usage of the different polling mechanisms, the IOPoll class can
be used. Simply instantiate an IOPoll() object and the best available mechanism
is used. Using the poll method with a timeout in milliseconds, the respective
poll function of the selected mechanism is invoked and the events are returns in
a standardized format as either IO_READ, IO_WRITE or IO_ERROR, or a combination
of those. Optionally, an argument specifiying a preference may be passed to
IOPoll which will try to use the specified polling mechanism. You can specify,
epoll, poll, kqueue or select, but the success depends on the availablility of
the mechanism.
"""
IO_READ = 1
IO_WRITE = 2
IO_ERROR = 4

io_logger = logging.getLogger('Borg.Brain.Util.IOPoll')
io_logger.addHandler(nullhandler.NullHandler())

_poll_class = None

def IOPoll(pref=None):
    """
    This function is a wrapper around the _IOPoll_xxxx classes. It selects the
    best polling mechnism available or uses the specified preference. It will
    then instantiate a polling object of the selected class (_IOPoll_epoll,
    _IOPoll_poll, _IOPoll_kqueue or _IOPoll_select).
    """
    global _poll_class
    if pref:
        # A preference has been given
        if pref == "epoll":
            if hasattr(select, 'epoll'):
                _poll_class = _IOPoll_epoll
                io_logger.info('Using epoll for IO Polls')
            else:
                io_logger.error("EPoll not available, cannot use")
        if pref == "poll":
            if hasattr(select, 'poll'):
                _poll_class = _IOPoll_poll
                io_logger.info('Using poll for IO Polls')
            else:
                io_logger.error("Poll not available, cannot use")
        if pref == "kqueue":
            if hasattr(select, 'kqueue'):
                _poll_class = _IOPoll_kqueue
                io_logger.info('Using kqueue for IO Polls')
            else:
                io_logger.error("KQueue not available, cannot use")
        if pref == "select":
            _poll_class = _IOPoll_select
            io_logger.info("Using select for IO Polls")
    
    if not _poll_class:
        # If no class has been selected, use the best available mechanism
        if hasattr(select, 'epoll'):
            io_logger.info('Using epoll for IO Polls')
            _poll_class = _IOPoll_epoll
        elif hasattr(select, 'poll'):
            io_logger.info('Using poll for IO Polls')
            _poll_class = _IOPoll_poll
        elif hasattr(select, 'kqueue'):
            io_logger.info('Using kqueue for IO Polls')
            _poll_class = _IOPoll_kqueue
        else:
            io_logger.info('Using select for IO Polls')
            _poll_class = _IOPoll_select
    return _poll_class()

class _IOPoll(object):
    """
    This class is the base IOPoll class. It maintains a set of the registered
    file descriptors. It needs to be subclassed for it to be useful.
    """
    def __init__(self):
        self._read_fds = set()
        self._write_fds = set()
        self._error_fds = set()
        self._edge_triggered = False

    def __del__(self):
        """
        Close the poll object when the Python object is deleted.
        """
        self.close()
    
    def edge_triggered(self):
        """
        Return whether edge triggered mode is enabled on this poll object
        """
        return self.edge_triggered

    def set_edge_triggered(self, et):
        """
        This method tries to enable the edge triggered mode and returns True if
        there is support for Edge Triggered mode or False if it is not
        supported.
        """
        return False

    def fileno(self):
        """
        If the poll object has a file descriptor of its own, such as poll and
        epoll, the file descriptor will be returned.
        """
        return None

    def close(self):
        """
        Close the poll object, if applicable.
        """
        pass

    def is_known(self, fd):
        """
        Check if the specified fd is registered in this poll object.
        """
        return fd in self._read_fds or \
               fd in self._write_fds or \
               fd in self._error_fds

    def register(self, fd, read=True, write=True, error=True):
        """
        Register a new file descriptor in the IOPoll object, and monitor it for
        reading, writing or errors, as specified in the parameters.
        """
        if read:
            self._read_fds.add(fd)
        if write:
            self._write_fds.add(fd)
        if error:
            self._error_fds.add(fd)

    def modify(self, fd, read=True, write=True, error=True):
        """
        Change the specified file descriptor to be monitored for reading,
        writing or errors as specified in the parameters.
        """
        self.unregister(fd)
        self.register(fd, read, write, error)

    def unregister(self, fd):
        """
        Remove the file descriptor from the poll object
        """
        self._read_fds.discard(fd)
        self._write_fds.discard(fd)
        self._error_fds.discard(fd)

    def poll(self, timeout = 0):
        """
        Call the polling function with a timeout in milliseconds. Return a list
        of (fd, event) tuples.
        """
        return []

    def select(self, read_fds = [], write_fds = [], error_fds = [], timeout = 0):
        """
        Emulate the select functionality with the polling object: create a new
        poll object, register the specified file descriptors, poll for events
        and return the results.
        """
        poll = self.__class__()
        for fd in read_fds:
            poll.register(fd, True, False, False)
        for fd in write_fds:
            poll.register(fd, False, True, False)
        for fd in error_fds:
            poll.register(fd, False, False, True)
        return poll.poll(timeout)

class _IOPoll_epoll(_IOPoll):
    """
    Implementation of the IOPoll class for the epoll polling mechanism,
    available on Linux since kernel version 2.5.44. Supports edge triggered
    and level triggered polling. Level Triggered is the default setting, Edge
    Triggered can be enabled per file descriptor.
    """
    def __init__(self):
        super(_IOPoll_epoll, self).__init__()
        self._poll = select.epoll()

    def set_edge_triggered(self, et):
        """
        Enable Edge Triggered behavior for all file descriptors registered after
        the call to this function.
        """
        self._edge_triggered = et
        if et:
            io_logger.info("Set epoll to Edge Triggered mode")
        else:
            io_logger.info("Set epoll to Level Triggered mode")
        return True

    def fileno(self):
        """
        Return the file descriptor of the epoll object.
        """
        return self._poll.fileno()

    def close(self):
        """
        Close the poll object.
        """
        self._poll.close()

    def _eventmask(self, read, write, error):
        """
        Helper function to get the correct event mask to register or modify
        a file descriptor in the epoll object.
        """
        eventmask = 0
        if read:
            eventmask = eventmask | select.EPOLLIN | select.EPOLLPRI | select.EPOLLRDNORM
        if write:
            eventmask = eventmask | select.EPOLLOUT | select.EPOLLWRNORM
        if error:
            eventmask = eventmask | select.EPOLLERR | select.EPOLLHUP
        if self._edge_triggered:
            eventmask = eventmask | select.EPOLLET
        return eventmask

    def register(self, fd, read=True, write=True, error=True):
        """
        This method registers a new file descriptor in the epoll object and
        monitors it for reading, writing or errors, as specified in the 
        parameters.
        """
        self._poll.register(fd, self._eventmask(read, write, error))
        if read:
            self._read_fds.add(fd)
        if write:
            self._write_fds.add(fd)
        if error:
            self._error_fds.add(fd)

    def modify(self, fd, read=True, write=True, error=True):
        """
        Change the events monitored for an already registered file descriptor.
        """
        self._poll.modify(fd, self._eventmask(read, write, error))
        if read:
            self._read_fds.add(fd)
        else:
            self._read_fds.discard(fd)
        if write:
            self._write_fds.add(fd)
        else:
            self._write_fds.discard(fd)
        if error:
            self._error_fds.add(fd)
        else:
            self._error_fds.discard(fd)

    def unregister(self, fd):
        """
        Remove a file descriptor from the poll object.
        """
        try:
            self._poll.unregister(fd)
        except:
            io_logger.warning("Trying to unregister a file descriptor that was not registered" % fdi)
            return

        self._read_fds.discard(fd)
        self._write_fds.discard(fd)
        self._error_fds.discard(fd)
        
    def poll(self, timeout = 0):
        """
        Poll the epoll object for the given timeout in milliseconds and return a
        list of (fd, event) tuples.
        """
        try:
            # epoll has timeout in seconds, so divide the amount of milliseconds
            # by 1000
            pevents = self._poll.poll(timeout / 1000.0)
        except IOError as e:
            if e.errno == 4: # interrupted system call; nothing serious, probably shutting down socket
                return []
            else:
                raise
        events = []
        for fd, pevent in pevents:
            if pevent & select.EPOLLIN or pevent & select.EPOLLPRI:
                events.append((fd, IO_READ))
            if pevent & select.EPOLLOUT:
                events.append((fd, IO_WRITE))
            if pevent & select.EPOLLERR or pevent & select.EPOLLHUP:
                events.append((fd, IO_ERROR))
        return events

class _IOPoll_poll(_IOPoll):
    """
    Implementation of the IOPoll class for the poll polling mechanism, available
    on Linux.
    """ 
    def __init__(self):
        super(_IOPoll_poll, self).__init__()
        self._poll = select.poll()

    def _eventmask(self, read, write, error):
        """
        This is a helper method to get the correct event mask to register or
        modify a file descriptor in the poll object.
        """
        eventmask = 0
        if read:
            eventmask = eventmask | select.POLLIN | select.POLLPRI
        if write:
            eventmask = eventmask | select.POLLOUT
        return eventmask

    def register(self, fd, read=True, write=True, error=True):
        """
        This method registers a new file descriptor in the epoll object and
        monitors it for reading, writing or errors, as specified in the 
        parameters.
        """
        super(_IOPoll_poll, self).register(fd, read, write, error)
        self._poll.register(fd, self._eventmask(read, write, error))

    def unregister(self, fd):
        """
        Remove a registered file descriptor from the poll object.
        """
        try:
            self._poll.unregister(fd)
        except:
            io_logger.warning("Trying to unregister a file descriptor %d that was not registered" % fd)
            return

        super(_IOPoll_poll, self).unregister(fd)
        
    def poll(self, timeout = 0):
        """
        Poll the kqueue object for the given timeout in milliseconds and return a
        list of (fd, event) tuples.
        """
        try:
            pevents = self._poll.poll(timeout)
        except IOError as e:
            if e.errno == errno.EINTR: # interrupted system call; nothing serious, probably shutting down socket
                return []
            else:
                raise
        events = []
        for fd, pevent in pevents:
            if pevent & select.POLLIN or pevent & select.POLLPRI:
                events.append((fd, IO_READ))
            if pevent & select.POLLOUT:
                events.append((fd, IO_WRITE))
            if pevent & select.POLLERR or pevent & select.POLLHUP or pevent & select.POLLNVAL:
                events.append((fd, IO_ERROR))
        return events

class _IOPoll_kqueue(_IOPoll):
    """
    Implementation of the IOPoll class for Kernel Queues, available on
    BSD-derivatives. 
    """
    def __init__(self):
        super(_IOPoll_kqueue, self).__init__()
        self._kqueue = select.kqueue()

    def set_edge_triggered(self, et):
        """
        Enable Edge Triggered behavior for all file descriptors registered after
        the call to this function.
        """
        self._edge_triggered = et
        if et:
            io_logger.info("Set kqueue to Edge Triggered mode")
        else:
            io_logger.info("Set kqueue to Level Triggered mode")
        return True

    def fileno(self):
        """
        Returns the file descriptor of the kqueue object.
        """
        return self._kqueue.fileno()

    def close(self):
        """
        Close the kernel queue
        """
        self._kqueue.close()

    def register(self, fd, read=True, write=True, error=True):
        """
        Register a new file descriptor in the kqueue, monitoring it for reading,
        writing or errors as specified by the parameters.
        """
        kevents = []
        flags = select.KQ_EV_ADD
        if self._edge_triggered:
            flags = flags | select.KQ_EV_CLEAR
        if read:
            kevents.append(select.kevent(fd, filter=select.KQ_FILTER_READ, flags=flags))
            self._read_fds.add(fd)
        if write:
            kevents.append(select.kevent(fd, filter=select.KQ_FILTER_WRITE, flags=flags))
            self._write_fds.add(fd)
        if error:
            self._error_fds.add(fd)
        for kevent in kevents:
            self._kqueue.control([kevent], 0)
            
    def modify(self, fd, read=True, write=True, error=True):
        """
        Modify a file descriptor already registered in the kqueue to change the
        events being monitored.
        """
        kevents = []
        add_flags = select.KQ_EV_ADD
        if self._edge_triggered:
            add_flags = add_flags | select.KQ_EV_CLEAR
        if read and not fd in self._read_fds:
            kevents.append(select.kevent(fd, filter=select.KQ_FILTER_READ, flags=add_flags))
            self._read_fds.add(fd)
        elif not read and fd in self._read_fds:
            kevents.append(select.kevent(fd, filter=select.KQ_FILTER_READ, flags=select.KQ_EV_DELETE))
            self._read_fds.discard(fd)
        if write and not fd in self._write_fds:
            kevents.append(select.kevent(fd, filter=select.KQ_FILTER_WRITE, flags=add_flags))
            self._write_fds.add(fd)
        elif not write and fd in self._write_fds:
            kevents.append(select.kevent(fd, filter=select.KQ_FILTER_WRITE, flags=select.KQ_EV_DELETE))
            self._write_fds.discard(fd)
        if error:
            self._error_fds.add(fd)
        else:
            self._error_fds.discard(fd)
        for kevent in kevents:
            self._kqueue.control([kevent], 0)
            
    def unregister(self, fd):
        """
        Remove a file descriptor from the kqueue object.
        """
        kevents = []

        if fd in self._read_fds:
            try:
                kevents.append(select.kevent(fd, filter=select.KQ_FILTER_READ, flags=select.KQ_EV_DELETE))
            except ValueError:
                pass # kevent finds unregistering closed FD's an error
            self._read_fds.discard(fd)
        if fd in self._write_fds:
            try:
                kevents.append(select.kevent(fd, filter=select.KQ_FILTER_WRITE, flags=select.KQ_EV_DELETE))
            except ValueError:
                pass # kevent finds unregistering closed FD's an error
            self._write_fds.discard(fd)
        if fd in self._error_fds:
            self._error_fds.discard(fd)
        for kevent in kevents:
            self._kqueue.control([kevent], 0)
        
    def poll(self, timeout = 0):
        """
        Poll for events during a timeout specified in milliseconds. Returns a
        list of (fd, event) tuples.
        """
        # kqueue has timeout in seconds, so divide the milliseconds by 1000
        try:
            kevents = self._kqueue.control(None, 1000, timeout / 1000.0)
        except IOError as e:
            if e.errno == 4: # interrupted system call; nothing serious, probably shutting down socket
                return []
            else:
                raise
        
        # Normalize events
        events = []
        for kevent in kevents:
            fd = kevent.ident
            event = None
            if kevent.filter == select.KQ_FILTER_READ:
                event = IO_READ
            elif kevent.filter == select.KQ_FILTER_WRITE:
                event = IO_WRITE
            if kevent.flags & select.KQ_EV_ERROR or \
               kevent.flags & select.KQ_EV_EOF:
                event = IO_ERROR
            events.append((fd, event))
        return events

class _IOPoll_select(_IOPoll):
    """
    This is a IOPoll implementation using the select mechanism, available on
    nearly all platforms. It it used as a fallback if no other mechanisms are
    available. Select performs significantly worse than the alternatives.
    """
    def __init__(self):
        super(_IOPoll_select, self).__init__()

    def poll(self, timeout = 0):
        """
        Perform a poll during the specified timeout in milliseconds. Uses the
        select method to perform the poll.
        """
        return self.select(self._read_fds, self._write_fds, self._error_fds, timeout)

    def select(self, read_fds=[], write_fds=[], error_fds=[], timeout=0):
        """
        Perform a select on the provided file descriptors. Used by poll method.
        """
        # select has timeout in seconds 
        try:
            readable, writable, errors = select.select(read_fds, write_fds, error_fds, timeout / 1000.0)
        except IOError as e:
            if e.errno == errno.EINTR: # interrupted system call; nothing serious, probably shutting down socket
                return []
            else:
                raise
        events = []
        for fd in readable:
            events.append((fd, IO_READ))
        for fd in writable:
            events.append((fd, IO_WRITE))
        for fd in errors:
            events.append((fd, IO_ERROR))

        return events

if __name__ == "__main__":
    import socket
    import random
    import time

    logging.getLogger('Borg.Brain.Util').addHandler(loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain.Util').setLevel(logging.DEBUG)
    poller = IOPoll("kqueue")

    port = random.randint(10000, 20000)
    print "Listening on port %d" % port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("", port))
    s.listen(5)

    poller.register(s, True, True, True)
    while True:
        events = poller.poll(0)
        print repr(events)
        time.sleep(1)


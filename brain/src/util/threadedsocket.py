import cStringIO as stringio
import cPickle as pickle
import struct
import threading
import time
import logging
import socket
import os
import sys
import errno

import nullhandler
import loggingextra
from iomanager import IOManager, IOWrapper, IOWrapperEnd

"""
################################################################################
## ThreadedSocket class                                                       ##
################################################################################
The ThreadedSocket is an instance of the IOWrapper for IOManager, to
network connections in an asynchronous manner.                             
                                                                           
It shows some relation to the asyncore system or the SocketServer framework but
it has some major advantages. The use of the IOManager allows allows the use of
the IOPoll poller that can fall back to the select class available on nearly
every platform, the poll or epoll interfaces on Linux, and the kqueue interface
on BSD-derivatives. All these interfaces perform significantly better than the
select system call, especially the edge triggered epoll mechanism on Linux. This
will increase throughput and responsivity of the IO manager. See the IOManager
class for more information about this.

The system uses threads, hence the name. This means that while IO is processed
asynchronously, due to the Global Interpreter Lock on CPython, it still needs
some time from the interpreter to work once in a while. If your code uses busy
loops, the IOManager will not get the chance to process the incoming and 
outgoing data and therefore the performance might degrade.

When a ThreadedSocket object is instantiated (see the comments at the
constructor, below), it will register itself with the IOManager, as an
unconnected socket, unless a connected socket has been passed in, in which case
it will be registered as a connected socket right away.

The IOManager will then call the _do_attempt function of the ThreadedSocket
approximately 10 times per second, at which the socket can try to connect. If it
actually does this 10 times per second depends on the retry_timeout parameter.

When the socket succesfully connects to the remote host or binds to the port, it
will register itself as a connected socket with the IOManager. As soon as there
are incoming connections or if there is data to read or write, the
_handle_read and _handle_write methods are called to handle this. Also, 10
times per second, the handle_loop function will be called to perform any side
functions a subclass of the ThreadedSocket might want to perform. The processing
in this method should be minimal.

To allow subclasses to handle received data differently, the handle_receive
method can be overridden. It will be called with the received data. The default
implementation will store it in the receive buffer which is a list of items
received in chronological order.

###############################################################################
## Sending and receiving: cPickle and cStringIO                              ##
###############################################################################
The ThreadedSocket will use cPickle by default to wrap pickleable Python objects
before sending them over the socket. This allows for more complex structures,
such as dictionaries or NumPy arrays to be sent over the socket.

When the data has been preprocessed, it will be stored in a cStringIO buffer.
This buffer is used when sending the data. After each call to send, the number
of bytes actually sent determines the new position in the cStringIO buffer. This
way, the strings will not be copied every time but the internal pointer of the
cStringIO will be used. This gives much better performance.

When data is received, it is stored in a cStringIO buffer once again. This
buffer is then used by cPickle to load the pickled items one by one. When
unrecognized data appears, it is collected and added a separate string in the
resulting data. Pickled items could also be truncated because they have been
sent in several parts. When this happens, the data received so far is stored and
prepended the next time data arrives. Unpickling the data is then attempted
again. This way, no data should be lost.

###############################################################################
## Unix Domain Sockets                                                       ##
###############################################################################
Unix Domain Sockets can be used on Posix compliant operating systems. They
behave similar to normal sockets but bypass the TCP/IP stack, making them more
performant in almost all situations. The data does not have to be
wrapped and unwrapped in IP packets, which is already an improvement and because
communication over Unix Domain Sockets is always local, when a client writes
data to the server, it can immediately be stored in the right memory location.

As mentioned above, Unix Domain Sockets can only be used for communication
between process on the same machine. They are accessed through a file in the
filesystem but all communication takes place in the kernel. Therefore, mounting
a filesystem over SSH and attempting to use a domain socket that way will not
work.

The ThreadedSocket supports Unix Domain Sockets. To use them, provide a name as
a port argument instead of a port number. When the type of the port argument is
string, it is used as the name of a socket. The socket files created by
ThreadedSocket are stored in the /tmp directory. Thus, using port 'test' will
result in a socket file /tmp/test.

Because the files are accessed through the filesystem, one can easily remove or
restrict access to the socket by deleting the file or by changing the attributes
of the file. If yu remove the socket file, the socket will remain open but no
process will be able to connect to it.

###############################################################################
## Ending a connection                                                       ##
###############################################################################
When connected to a ThreadedSocket, you can close the connection in several
ways. Sending a Ctrl+C or Ctrl+D sequence to the socket will result in closure
of the socket. This can be used when debugging using Telnet, for example.

Programmatically, you can just close the socket by closing the corresponding
file descriptor. ThreadedSocket will detect this at the other end and close its
own socket.
"""

class ThreadedSocket(IOWrapper):
    end_sequences = ["\xff\xf4\xff\xfd\x06", # Ctrl+C
                     "\x04"]                 # Ctrl+D
    def __init__(self,
                 host="localhost", # The host to connect to or serve
                 port=49152,       # The port to connect to or serve
                 giveup=5,         # Amount of reconnect attempts
                 use_socket=None,  # Pass in a connected socket object to use
                 retry_timeout=1,  # Timeout between connection attempts
                 server=None,      # True for a server, False for clients
                 use_pickle=True,  # Whether to pickle/unpickle 
                                   #    send/received data
                 bufsize="auto",   # Number of bytes to send/receive at once
                 linger=.1,        # Whether to set the SO_LINGER option
                 handler=False,    # Set to true to make this a incoming 
                                   #    connection handler
                 logger=None       # Pass in a logger to use
                ):
        """
        Initialize the socket. The parameters define the behavior of the socket.
        
        Host should be an IP address or hostname to which to connect or on which
        to listen. If the server argument is not specifically set, the hostname
        will also define whether this will be a server or client socket: if host
        is empty, it will be a server socket listening on all interfaces,
        otherwise it will be a client socket. 

        port should be the port to listen on. When the port is numeric it is
        assumed to be a TCP port. When it is a string, it will be cast to an int
        specifying a TCP port. If the conversion failed, it is assumed to be a
        Unix Socket, located in /tmp. This will raise an exception if the socket
        is trying to connect to a remove Unix Socket, which is impractical and
        defeats the purpose.

        giveup is the amount of retries performed to connect to a socket or bind
        to a port.

        use_socket can be used to pass in an already connected socket. The same
        socket options will be set on this socket as newly created ones.

        retry_timeout defines the number of seconds to wait between connection/
        binding attempts of unconnected sockets.

        server is a boolean that specifies whether this will be a server or a
        client socket. A server socket will bind and listen to a port, a client
        socket will attempt to connect to a port. The default value decides this
        automatically, based on the value of the host parameter as described
        above.

        use_pickle is a boolean specifying whether communication on this socket
        will proceed using the Pickle protocol. If set to true, all outgoing
        data will be pickled and all incoming data will be unpickled.

        bufsize specifies the amount of data to send or receive at once. The
        default is auto, meaning it will be set depending on other options. When
        the socket to use is a Unix Domain Socket, it will use 128 KiB. If it is
        a client socket connecting to a local socket, it will be a 16 KiB and if
        it is a server socket or a client socket connecting to a non-local host,
        a buffer of 8 KiB is used.

        linger is a number specifying whether the socket will be set in
        linger mode (see man page for setsockopt(2)). When in linger mode, the
        call to close on the socket will block for at most a set number of
        seconds and will then forcefully close the socket. Otherwise, the close
        call is non-blocking and the socket will get in a TIME_WAIT state, which
        effectively blocks the port for up to 4 minutes, possibly resulting in
        'Port already bound' messages. If the linger parameter is 0, LINGER will
        not be used; otherwise this is the amount of seconds to set.

        handler can be set to True to activate incoming connection handler 
        behavior. This basically makes sure that no reconnetion attempt is 
        performed when the connection is closed by the other end.
        """
        
        
        
        self.__socket = None
        self.__fileno = None
        self.__client_sockets = []
        self.__connected = False
        self.__listener = False
        self.__handler = handler
        self.__use_pickle = use_pickle

        self.__send_lock = threading.RLock()
        self.__receive_lock = threading.RLock()
        self.__receive_buffer = []
        self.__recv_buffer = ""
        self.__send_buffer = []
        self.__send_buffer_ids = set([])
        self.__buffer_start_pos = 0
        self.__send_id = 0

        self.__set_port(host, port)

        self.__set_buffer_size(bufsize)
        self.__linger = float(linger)
        self.__total_bytes = 0

        self.__log_input = False
        self.__log_output = False
        
        if server is None and self.__host == "":
            self.__listener = True
        else:
            self.__listener = server

        if logger:
            self.__logger = logger
        else:
            self.__logger = logging.getLogger('Borg.Brain.Util.ThreadedSocket')

        self.__logger.addHandler(nullhandler.NullHandler())

        self.__last_attempt = 0
        self.__timeout = retry_timeout
        self.__giveup = giveup
        self.__attempt = 0

        if use_socket:
            self.__socket = use_socket
            self.__fileno = self.__socket.fileno()
            self.__port = port
            self.__connected = True
            self.__set_socket_options(self.__socket)
            mgr = IOManager()
            mgr.register(self)
        else:
            mgr = IOManager()
            mgr.register_unconnected(self)

    def __del__(self):
        """
        When the object goes out of scope, close and unregister
        the socket.
        """
        try:
            self.close(active=True)
        except TypeError:
            pass

    def __repr__(self):
        if self.__listener:
            return "<ThreadedSocket Server on %s>" % self.__port
        elif self.__handler:
            if self.__unix_socket:
                return "<ThreadedSocket Server Handler connected to %s>" \
                       % self.__port
            else:
                return "<ThreadedSocket Server Handler connected to %s:%s>" \
                       % (self.__host, self.__port)
        else:
            if self.__unix_socket:
                return "<ThreadedSocket Client Connecting to %s>" % self.__port
            else:
                return "<ThreadedSocket Client Connecting to %s:%s>" \
                       % (self.__host, self.__port)

    def __str__(self):
        return self.__repr__()

    ############################################################################
    ## IOMANAGER INTERFACE                                                    ##
    ############################################################################
    ## The following methods are meant to be called by the IOManager          ##
    ## thread only. They are called when there is data to read or write on    ##
    ## the socket, or on each iteration of the loop.                          ##
    ############################################################################
    def _handle_read(self):
        """
        This method is called from the IOManager thread when there is data
        available to read from the socket.
        """
        if not self.connected():
            raise IOWrapperEnd()

        # If this is a server socket, accept a connection
        if self.listening():
            return self._handle_accept()

        try:
            data = self.__socket.recv(self.__buffer_size)
        except socket.error as err:
            raise IOWrapperEnd()
            ## Some kind of error on the socket means that
            ## the other end closed the connection
            #self.close()
            #return
         
        if self.__is_connection_closed(data):
            return

        data = self.__unwrap_data(data)
        if self.__log_input:
            self.__logger.debug("Received data: %s" % repr(data))
        self.handle_receive(data)

    def _handle_accept(self):
        """
        This method is called from the _handle_read method when
        the socket for which there is data to read is a server socket,
        meaning there is a connection to accept.
        """
        try:
            (sock, address) = self.__socket.accept()
        except socket.error as e:
            if e.errno == errno.EINVAL: # Invalid Argument
                self.__logger.critical("Invalid argument on accepting " \
                                       "connection: %s. Closing socket." \
                                       % (repr(self)))
                self.close()
            raise IOWrapperEnd()

        if self.__unix_socket:
            self.__logger.info("Incoming connection on %s" % str(self.__port))
            address = ("", self.__port)
        else:
            self.__logger.info("Incoming connection from %s" % str(address))

        self.address = address
        self.handle_accept(sock, address)

    def _handle_write(self):
        """
        This method is called from the IOManager thread when there the socket
        is writable and marked writable in the IO poller. It uses the stringio
        object of the current send buffer to get the next data to send and
        makes sure that the stringio object is set at the correct position,
        based on the amount of bytes actually sent.
        """
        with self.__send_lock:
            if not self.__send_buffer:
                # No data to write, make sure that this
                # socket is marked not writable in the
                # IOManager
                if self.connected():
                    mgr = IOManager()
                    mgr.set_writable(self, False)
                raise IOWrapperEnd()

        
            cur_id, cur_buffer = self.__send_buffer[0]
            cur_buffer.seek(self.__buffer_start_pos)
            data = cur_buffer.read(self.__buffer_size)
            if not data:
                # If the buffer ran out of data,
                # remove it from the queue and return.
                # The next attempt to write will set
                # the socket non-writable if there is
                # no next send buffer.
                cur_buffer.close()
                self.__send_buffer.pop(0)
                self.__send_buffer_ids.remove(cur_id)
                self.__buffer_start_pos = 0
                
                return

            sent_bytes = 0
            try:
                sent_bytes = self.__socket.send(data)
                self.__buffer_start_pos += sent_bytes
            except socket.error as err:
                if err.errno == errno.EAGAIN: # Resource temporarily unavailable
                    raise IOWrapperEnd()
                raise
            
            if self.__log_output:
                self.__logger.debug("Sent data: %s" % repr(data[:sent_bytes]))
                
            self.__total_bytes += sent_bytes
            return sent_bytes

    def _handle_loop(self):
        """
        This method is called from IOManager about 10 times per second and
        allows subclasses to perform an action at regular intervals.
        """
        self.handle_loop()

    def _handle_attempt(self):
        """
        This method is called when the socket is not
        connected, once every loop of the IOManager.
        """
        self.handle_loop_disconnected()
        if time.time() < self.__last_attempt + self.__timeout:
            return False

        self.__last_attempt = time.time()
        if self.__listener:
            result = self.__listen()
        else:
            result = self.__connect()

        if not result:
            self.__attempt += 1
            action = "bind to port" if self.__listener else "connect"
            if self.__giveup >= 0 and self.__attempt >= self.__giveup:
                self.__logger.critical("Failed to %s for %d times. Giving up." \
                                       % (action, self.__attempt))
                mgr = IOManager()
                mgr.unregister_unconnected(self)
            elif self.__giveup >= 0:
                self.__logger.critical("Failed to %s for %d/%d times. " \
                                       "Retrying in %d seconds."        \
                                       % (action, self.__attempt,       \
                                          self.__giveup, self.__timeout))
            else:
                self.__logger.critical("Failed to %s for %d times. " \
                                       "Retrying in %d seconds."     \
                                       % (action, self.__attempt,    \
                                          self.__timeout))
        else:
            self.__attempt = 0

    ############################################################################
    ## HELPER METHODS                                                         ##
    ############################################################################
    ## The following methods are meant for internal use only. They are        ##
    ## helpers for the public and IOManager interface of the ThreadeSocket    ##
    ## class.                                                                 ##
    ############################################################################
    def __is_local(self, host):
        """
        Determine whether the given hostname is a local or a remote
        hostname.
        """
        return host == "127.0.0.1"      \
            or host == "::1"            \
            or host == "localhost"      \
            or host == ""

    def __set_port(self, host, port):
        """
        Set the specified hostname and port on this socket, determining the
        type of socket to use: unix or inet. 
        """
        self.__host = host
        try:
            self.__port = int(port)
            self.__unix_socket = False
        except ValueError:
            if not self.__is_local(host):
                raise Exception("Cannot connect to Unix Domain Socket on a " \
                                "non-local host")
            self.__port = "/tmp/%s" % port
            self.__unix_socket = True
        self.address = (self.__host, self.__port)

    def __set_socket_options(self, sock):
        """
        This method sets the ThreadedSocket class options on the specified socket. 
        The options set are non-blocking mode and the SO_LINGER option.
        """
        enable = False if self.__linger == 0 else 1
        linger = int(self.__linger * 1000)
        sock.setsockopt(socket.SOL_SOCKET,               \
                        socket.SO_LINGER,                \
                        struct.pack('ii', 1, int(linger)))
        sock.setblocking(False)

    def __set_buffer_size(self, bufsize):
        """
        This method sets the buffer size, based on the specified parameter. If
        the bufsize parameter is set to "auto", it will be decided based on
        the type of socket. See the constructor for more details.
        """
        if bufsize == "auto":
            if self.__unix_socket:
                self.__buffer_size = 1024 * 256
            else:
                if self.__is_local(self.__host):
                    self.__buffer_size = 16384
                else:
                    self.__buffer_size = 8192
        else:
            self.__buffer_size = int(bufsize)

    def __is_connection_closed(self, data):
        """
        This helper method checks the received data to see if the connection
        is closed or should be closed. When the read data is empty or when
        an ending sequence (Ctrl+C or Ctrl+D) is received, the connection is
        also closed.
        """
        if not self.connected():
            return True

        if (not data) or (data in self.end_sequences):
            self.close(active=False)
            return True

        return False

    def __unwrap_data(self, data):
        """
        This method tries to unpickle the received data. If this fails, the
        contents are stored as a plain string. If the string appears to be
        truncated pickled data, the contents are stored in a buffer and
        prepended to further incoming data.
        """
        if not self.__use_pickle:
            return [data]

        with self.__receive_lock:
            if self.__recv_buffer:
                data = self.__recv_buffer + data
                self.__recv_buffer = ""

        inputdata = stringio.StringIO(data)
        out_data = []
        erroneous_data = ""
        pos = 0
        while True:
            try:
                cur_data = pickle.load(inputdata)
                out_data.append(cur_data)
                pos = inputdata.tell()
            except pickle.UnpicklingError as err:
                inputdata.seek(pos)
                if err.message.find("truncated") >= 0:
                    # The string is a pickled string, but
                    # appears to be truncated. Store the remaining
                    # buffer and try again when more data is received.
                    remain_data = inputdata.read()
                    with self.__receive_lock:
                        self.__recv_buffer = remain_data
                    break
                else:
                    erroneous_data += inputdata.read(1)
            except EOFError:
                diff = inputdata.tell() - pos
                # Unprocessed data is still in the buffer, store it
                if diff:
                    inputdata.seek(pos)
                    remain_data = inputdata.read()
                    with self.__receive_lock:
                        self.__recv_buffer = remain_data
                break
            except ValueError as err:
                self.logger.error("Unpickling failed: %s. Ignoring to fix objectdetector" % repr(err))
                inputdata.seek(pos)
	        erroneous_data += inputdata.read(1)
		
        # Append erroneous data as a string
        if erroneous_data:
            out_data.append(erroneous_data)
            self.__logger.warn("Unpickling received data failed: %s" \
                               % repr(erroneous_data))

        return out_data

    def __connect(self):
        """
        This method is called by the _do_attempt method when the socket is a
        connecting socket that should try to connect
        """
        if self.__socket:
            return False

        if self.__unix_socket:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            address = (self.__port)
            to_what = "Unix Domain Socket %s" % self.__port
        else:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            address = (self.__host, self.__port)
            to_what = "TCP port %d" % self.__port
        try:
            sock.connect(address)
            self.__set_socket_options(sock)
        except socket.error as e:
            self.__logger.error("Could not connect to %s: %s" \
                                % (to_what, str(e)))
            return False
        except Exception as e:
            # Other exceptions mean bad settings. Try one more time but do not
            # keep on trying forever..
            self.__logger.error("Could not connect to %s: %s" \
                                % (to_what, str(e)))
            self.__giveup = 2
            return False

        self.__socket = sock
        self.__connected = True
        self.__fileno = self.__socket.fileno()

        mgr = IOManager()
        mgr.register(self)
        if not self.send_buffer_empty():
            mgr.set_writable(self, True)

        return True

    def __listen(self):
        """
        This method is called from the connection manager when the socket is a
        server socket and should bind to the port specified.
        """
        if self.__socket or not self.__listener:
            return False

        if self.__unix_socket:
            if os.path.exists(self.__port):
                try:
                    os.unlink(self.__port)
                except:
                    self.__logger.error("Socket %s exists and cannot be " \
                                        "removed" % self.__port)
                    return False

            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            address = (self.__port)
            to_what = "Unix Domain Socket %s" % self.__port
        else:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            address = (self.__host, self.__port)
            to_what = "TCP port %d" % self.__port

        try:
            sock.bind(address)
            sock.listen(5)
            self.__set_socket_options(sock)
        except socket.error as e:
            self.__logger.warning("Could not bind to %s: %s" \
                                  % (to_what, str(e)))
            return False
        except Exception as e:
            # Other exceptions usually mean bad settings. Try one more time but
            # do not keep on trying forever..
            self.__logger.error("Could not connect to %s: %s" \
                                % (to_what, str(e)))
            self.__giveup = 2
            return False

        self.__logger.info("Socket listening on %s initialized" % to_what)

        self.__socket = sock
        self.__fileno = self.__socket.fileno()
        self.__connected = True
        mgr = IOManager()
        mgr.register(self)
        return True

    ############################################################################
    ## THREADING 'IMPLEMENTATION'                                             ##
    ############################################################################
    ## The next four methods are for backwards compatibility with the old     ##
    ## ThreadedSocket implementation, which had a separate thread for each    ##
    ## connection. To avoid failures due to code using these methods, they    ##
    ## are implemented as no-op methods.                                      ##
    ############################################################################
    def start(self):
        pass
    
    def join(self):
        pass

    def is_alive(self):
        return True

    def terminate(self):
        """
        This method closes the socket
        """
        self.close()

    ############################################################################
    ## SUBCLASS METHODS                                                       ##
    ############################################################################
    ## The following methods can be overriden by a subclass to change the     ##
    ## default behavior of the socket. They can handle incoming data,         ##
    ## perform actions during each iteration of the loop of the IOManager     ##
    ## and perform cleanup behavior on shutdown.                              ##
    ############################################################################
    def handle_accept(self, sock, address):
        """
        This method handles the newly accepted connection. It could be
        overloaded to use a different class to handle the new connection.
        """
        host, port = address
        sock = ThreadedSocket(host, 
                              port,
                              use_socket=sock,
                              server=False,
                              use_pickle=self.__use_pickle,
                              handler=True)
        self.__client_sockets.append(sock)
        return sock

    def handle_receive(self, data):
        """
        Default handler for when data has been received. The default
        implementation appends it to the receive buffer and it can be obtained
        by calling get_data or wait_data.
        """
        with self.__receive_lock:
            self.__receive_buffer.extend(data)

    def handle_shutdown(self):
        """
        Default handler for when the socket closes. The default implementation
        does nothing. It can be overridden to perform cleanup when the socket
        closes.
        """

    def handle_loop(self):
        """
        Default handler for the loop. This is called upon every loop
        where this socket has a connection to a server or is a server
        socket bound to a port. The default implementation does nothing.
        """
        pass

    def handle_loop_disconnected(self):
        """
        Default handler for the loop. This is called upon every loop where the
        socket has no connection. The default implementation does nothing.
        """
        pass

    ###########################################################################
    ## PUBLIC API                                                            ##
    ###########################################################################
    ## The following functions are the public API of the ThreadedSocket.     ##
    ## They are meant of modules and functions using the ThreadedSocket to   ##
    ## get access to client sockets of a server and to send and receive      ##
    ## data on the socket.                                                   ##
    ###########################################################################
    def close(self, active=True):
        """
        This method will close the socket. The active parameter will specify
        whether the socket is closed on this side or was closed on the other
        side. If the socket was closed on this side, no attempt to reconnect
        will be attempted. Otherwise, a reconnect will be attempted.
        """
        if not self.__socket or not self.__connected:
            return

        if self.__unix_socket:
            if self.__listener:
                close_what = "Unix Domain Socket listening on %s" \
                             % (self.__port)
            else:
                close_what = "Unix Domain Socket connection to %s" \
                             % (self.__port)
        else:
            if self.__listener:
                close_what = "server socket on port %d" % (self.__port)
            else:
                close_what = "TCP connection to %s:%d" \
                             % (self.__host, self.__port)

        self.__connected = False
        mgr = IOManager()
        mgr.unregister(self)

        if not self.__socket is None:
            try:
                self.__socket.close()
                self.__socket.shutdown()
            except socket.error as e:
                pass

        if active:
            self.__logger.info("Closed %s" % close_what)
        else:
            self.__logger.info("%s closed." % close_what)

        if self.__unix_socket and self.__listener:
            try:
                os.unlink(self.__port)
            except:
                pass

        self.__socket = None
        self.__fileno = None

        # If remote end closed connection, attempt to reconnect
        if not active and           \
           not self.__listener and  \
           not self.__handler and   \
           self.__giveup != 0:
            mgr = IOManager()
            mgr.register_unconnected(self)
        else:
            self.handle_shutdown()

    def get_clients(self):
        """
        This method returns the clients connected to this server socket. It
        filters out the disconnected sockets and updates the internal list.
        """
        with self.__receive_lock:
            proper_list = []
            for wrapper in self.__client_sockets:
                if wrapper.connected():
                    proper_list.append(wrapper)
            self.__client_sockets = proper_list
            return proper_list


    def send(self, data, optional=False):
        """
        This method enqueues new data to be send. It wraps the data by pickling
        it if configured to do so and creates a new StringIO buffer to store the
        new data.
        """
        if self.__listener:
            if not self.__client_sockets:
                self.__logger.warning("No clients connected to send data")
                return False
            with self.__receive_lock:
                for wrapper in self.__client_sockets:
                    wrapper.send(data)
            return True 

        if self.__use_pickle:
            try:
                data = pickle.dumps(data)
            except pickle.PicklingError:
                self.__logger.error("Cannot pickle data %s" % repr(data))
                return False

        with self.__send_lock:
            set_writable = False
            if not self.__send_buffer:
                set_writable = True

            id = self.__send_id
            self.__send_id += 1
            buffer = stringio.StringIO(data)
            self.__send_buffer.append((id, buffer))
            self.__send_buffer_ids.add(id)

            if set_writable and self.connected():
                mgr = IOManager()
                mgr.set_writable(self, True)
            return id

    def get_data(self):
        if self.__listener:
            if not self.__client_sockets:
                self.__logger.warning("No clients connected to receive data")
                return False
            data = []
            with self.__receive_lock:
                for wrapper in self.__client_sockets:
                    data.extend(wrapper.get_data())
            return data

        with self.__receive_lock:
            if not self.__receive_buffer:
                return []
            data = self.__receive_buffer
            self.__receive_buffer = []
            return data

    def set_verbose(self, input=True, output=True):
        """
        Set verbosity of the socket logging. This method enables or disables
        logging of input and output on the logger.
        CAUTION: enabling logging can severely decrease performance when sending
        or receiving large amounts of data.
        """
        self.__log_input = input
        self.__log_output = output

    ############################################################################
    ## STATUS METHODS                                                         ##
    ############################################################################
    ## The follinowing methods are simple status pollers that tell if the     ##
    ## socket is connected or listening, or how much data has been sent.      ##
    ############################################################################
    def connected(self):
        """Return whether the socket is connected/bound"""
        return self.__connected

    def has_data(self):
        """Return whether the socket has received data"""
        return len(self.__receive_buffer) > 0

    def listener(self):
        """Returns True if this is a server socket, False otherwise"""
        return self.__listener

    def listening(self):
        """Returns True when the server socket is bound to its port"""
        return self.listener() and self.connected()

    def send_buffer_empty(self):
        """Returns True if the send buffer is empty"""
        with self.__send_lock:
            return not self.__send_buffer

    def bytes_transfered(self):
        """Returns the amount of bytes sent over the socket"""
        return self.__total_bytes

    def fileno(self):
        return self.__fileno

    ############################################################################
    ## SYNCHRONOUS INTERFACE                                                  ##
    ############################################################################
    ## The following methods provide a semi-synchronous interface as they     ##
    ## allow to perform more or less blocking requests on the socket. They    ##
    ## will wait a certain amount of time for an event to happen.             ##
    ############################################################################
    def wait_data(self, timeout=None):
        """
        This method waits for at most timeout seconds for new data to arrive. If
        new data does arrive, it is returned, otherwise an empty list is 
        returned.

        If timeout is None, no timeout will be used.
        """
        start_time = time.time()
        while True:
            data = self.get_data()
            if data:
                return data

            if not timeout is None and time.time() >= start_time + timeout:
                break
            time.sleep(0.01)
        return []

    def wait_connect(self, timeout=None, disconnect=False):
        """
        This method waits for at most timeout seconds for a connection to be
        made.  For a server socket, this will wait for a client to connect and
        return the new client socket. For a client socket, this will wait for
        the connection to the server to be made and return True.

        None is returned if no connection is made before the request times out.

        If timeout is None, no timeout will be used.
    
        If disconnect is set to True, this method checks for disconnection
        instead.
        """
        start_time = time.time()
        while True:
            if self.__listener:
                clients = self.get_clients()
                result = bool(clients)
                if disconnect:
                    result = not result
                if result:
                    return clients[0]
            elif (not disconnect) and self.__connected:
                return True
            elif disconnect and (not self.__connected):
                return True
            
            if not timeout is None and time.time() >= start_time + timeout:
                break
            time.sleep(0.01)

        return None

    def wait_listen(self, timeout=None):
        """
        This method waits for at most timeout seconds for a server socket to
        bind to the specified port. It returns True when the server has
        succesfully bound to the port. It returns False when, after timeout
        seconds, the server still has not bound to the port.
    
        If timeout is None, no timeout will be used.
        """
        start_time = time.time()
        while True:
            if self.listening():
                return True
            
            if not timeout is None and time.time() >= start_time + timeout:
                break
            time.sleep(0.01)

        return False

    def wait_send(self, timeout=None):
        """
        This method waits for at most timeout seconds for the send buffer to
        become empty, meaning that all queued data has been sent over the 
        socket.

        If timeout is None, no timeout will be used.
        """
        start_time = time.time()
        while True:
            if self.send_buffer_empty():
                return True
            
            if not timeout is None and time.time() >= start_time + timeout:
                break
            time.sleep(0.01)

        return False

    def sendall(self, *args, **kwargs):
        """
        This method enqueues new data to be send, stores the send ID of the new
        data and waits until that send ID is no longer in the send buffer queue,
        meaning that the data has succesfully been sent over the socket.

        Basically, this provides a synchronous send. It returns True when the
        data has been sent. It returns False if the data could not be sent,
        probably because the connection has been closed.
        """
        id = self.send(*args, **kwargs)
        while True:
            if not id in self.__send_buffer_ids:
                return True
            if not self.connected():
                return False
            time.sleep(0.01)
    

################################################################################
## DRIVER                                                                     ##
################################################################################
## The following main function will start a server or a client socket,        ##
## depending on the command line arguments used. If one argument is specified,##
## this will be the port to listen on. If two arguments are specified, this   ##
## will be a client socket connecting to the specified hostname and port.     ##
##                                                                            ##
## The server socket will wait for an incoming connection and store all       ##
## data received on it to a file.                                             ##
##                                                                            ##
## The client socket will connect and read the contents of a file and         ##
## send them to the server.                                                   ##
################################################################################
if __name__ == "__main__":
    logging.getLogger('Borg.Brain.Util').addHandler(loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain.Util').setLevel(logging.DEBUG)

    # Change this to some large file to check data throughput
    filepath = "/home/student/vakken/Robotica/sdk/"
    filename = filepath + "opennao-academics-1.8.16-nao-geode.ext3.gz"
    if len(sys.argv) == 2:
        port = sys.argv[1]
        host = ""
        server = True
    elif len(sys.argv) == 3:
        host = sys.argv[1]
        port = sys.argv[2]
        server = False
    else:
        port = "test"
        host = "localhost"
        server = False

    sock = ThreadedSocket(host, port, server=server, use_pickle=False)

    if server:
        sock.wait_listen()
        print "Listening"
    else:
        sock.wait_connect()
        print "Connected"

    if server:
        client = sock.wait_connect()
        if not client:
            print "No client connected"
            sock.close()
            sys.exit(0)

        print "Connected, saving data to output.gz"
        bytes = 0
        f = open("output.gz", "wb")
        bytes = 0
        start_time = time.time()
        while client.connected():
            elapsed = time.time() - start_time
            data = sock.get_data()
            for d in data:
                bytes += len(d)
                #f.write(d)
            time.sleep(0.05)

        f.close()

        elapsed = time.time() - start_time
        rate = bytes / elapsed / 1024.0 / 1024.0
        print "Disconnected, %d bytes transfered - %f mb/s" % (bytes, rate)
    else:
        data = open(filename, "rb").read()
        nbytes = len(data)
        print "Data read; %d b" % nbytes
        start_time = time.time()
        sock.send(data)
        
        while sock.bytes_transfered() < nbytes and sock.connected():
            bytes = sock.bytes_transfered()
            elapsed = time.time() - start_time
            speed = bytes / elapsed
            print "%d kb transfered - speed: %f mb/s" \
                  % (bytes / 1024.0, speed / 1024.0 / 1024.0)
            time.sleep(0.1)
        bytes = sock.bytes_transfered()
        elapsed = time.time() - start_time
        speed = bytes / elapsed
        print "Done"
        print "%d transfered - speed: %f mb/s" \
              % (bytes, speed / 1024.0 / 1024.0)

    sock.close()

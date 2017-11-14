import cv
import threading
import cPickle as pickle
import struct
import sys
import time
import hashlib
import cStringIO as stringio
import hashlib
import zlib

from threadedsocket import ThreadedSocket

"""
################################################################################
## BinarySocket class                                                         ##
################################################################################
The BinarySocket is an instance of the IOWrapper for IOManager, to handle
network connections in an asynchronous manner.                             

It uses the ThreadedSocket class to handle the IO related functionality of the
class.

The purpose of the BinarySocket class is to send binary data over a socket.
The pickle class can be used for this theoretically, but with large amounts of
data this is not efficient as pickling takes a lot of time. Each piece of binary
data can be accompanied by a piece of metadata. The metadata should be
relatively small as it will be pickled before being transported. The binary
string can be of any length. If you want, you can use the BinarySocket as a
replacement for threadedsocket as you can simply omit the binary data and just
send metadata; yielding more or less the same results, but this is not
recommended.

The BinarySocket works rather simply by prepending a 9-byte header before each
request. The first 4 bytes represent an unsigned integer giving the amount of
bytes of metadata in the current request, the next 4 bytes represent an unsigned
integer giving the amount of bytes of binary data in the current request. The
last byte represents a boolean indicating whether the data has been compressed.
The size of the metadata is calculated after pickling the metadata and optional
compression. The size of the binary data is calculated after optional
compression.

When receiving data, the header will be read. Afterwards, the specified amount
of bytes is read as metadata and the data is unpickled, and next, the specified
amount of bytes of binary data is read. When the entire request has been
received, the metadata and binary data is appended to the receive buffer and can
be obtained normally, as if you were retrieving data from the ThreadedSocket:

data = binarysocket.get_data()

Here, data will be a list containing the received data. Each element will be a
tuple (metadata, binary_data). You can then use the metadata to recreate
whatever the binary data representes. For example, you can store the depth,
number of channels and the dimensions of an opencv image in the metadata and use
the image.tostring() method to generate the binary data. This way, you can
transport images over the socket easily. This also works for, for example, large
numpy arrays, sound streams or other large data.

################################################################################
## Reliability                                                                ##
################################################################################
The BinarySocket class is designed to be used over Unix Domain Sockets, TCP
sockets connecting to a local host or to a Local Area Network. It was not
designed to be used over a more unstable connection such as the internet. It
therefore does not split the data, use checksums or have means to refetch
incorrectly transferred parts, making the protocol more efficient over stable
connections. To transfer over the internet, more elaborate protocols are
required. 
"""

class BinarySocket(ThreadedSocket):
    """
    The BinarySocket class allows to send large chunks of binary data,
    accompanied by some metadata in a Python structure, over a socket.
    """
    def __init__(self, host, port, use_socket=None, server=False, handler=False, bufsize="auto", compress=False, compress_level=6):
        """
        Set up the constructor. Call the ThreadedSocket constructor with the
        parameters specified. Also set up buffers for the headers, binary data
        and metadata.
        """
        super(BinarySocket, self).__init__(host, port, server=server, use_socket=use_socket, use_pickle=False, bufsize=bufsize, handler=handler)
        self.__header_buffer = ""
        self.__binary_buffer = ""
        self.__meta_buffer = ""
        self.__header_length = 2 * 4 + 1 # 2 Unsigned Ints, 1 Bool
        self.__binary_length = None
        self.__binary_compressed = False
        self.__meta_length = None
        self.__buffer_lock = threading.Lock()

        self.set_compression(compress, compress_level)

    def set_compression(self, compress, level=6):
        self.__compress = bool(compress)
        self.__compress_level = min(max(int(level), 1), 9)

    def handle_accept(self, sock, address):
        """
        Handle a new incoming connection: make sure that the new connection
        handler is also a BinarySocket instance.
        """
        if sock:
            host, port = address
            handler = BinarySocket(host, port, use_socket=sock, handler=False, bufsize=self._ThreadedSocket__buffer_size)
            self._ThreadedSocket__client_sockets.append(handler)

    def read_part(self, read_buffer, buffer_name, total_size):
        """
        Read a part from the incoming data in a StringIO buffer. 
        
        read_buffer is the buffer from which to read the data
        buffer_name is the name of the data member in which to store the data
        total_size is the total number of bytes that should end up in the buffer

        Tries to read as many bytes as possible from the buffer until the
        buffer has a total size of total_size bytes. If the buffer already has
        the correct length, True will be returned. When enough data can be read
        to fill the buffer completely, True will also be returned. If more data
        is required, False will be returned.
        """
        if buffer_name[0:2] == "__":
            # Perform name mangling
            buffer_name = "_BinarySocket" + buffer_name

        # Calculate the number of bytes still needed
        cursize = len(self.__dict__[buffer_name])
        remain = total_size - cursize
        if remain == 0:
            return True

        # Try to read the remaining bytes from the buffer
        try:
            str = read_buffer.read(remain)
        except EOFError:
            return False

        # Append the read data to the buffer and check the new size
        self.__dict__[buffer_name] += str
        new_size = len(self.__dict__[buffer_name])
        if new_size < total_size:
            return False

        # Buffer is of proper length
        return True

    def handle_receive(self, data):
        """
        This method handles incoming data. It tries to form a complete package
        of header, metadata and binary data. When it succeeds, it adds the
        data to the receive_buffer, from where it can be obtained using the
        get_data method of the ThreadedSocket class.
        """
        alldata = ""
        for item in data:
            alldata += item

        # Wrap data in a StringIO buffer
        cur_buffer = stringio.StringIO(alldata)

        with self.__buffer_lock:
            # While the buffer is not fully processed
            while True:
                # Read header, if no length for metadata and binary data is
                # available
                if self.__meta_length is None:
                    if self.read_part(cur_buffer, "__header_buffer", self.__header_length):
                        header = struct.unpack("!II?", self.__header_buffer)
                        self.__meta_length, self.__binary_length, self.__binary_compressed = header
                        total = len(self.__header_buffer) + self.__meta_length + self.__binary_length
                    else:
                        break

                # Read metadata bytes when the metadata buffer does not have the
                # correct length yet.
                if len(self.__meta_buffer) < self.__meta_length:
                    if not self.read_part(cur_buffer, "__meta_buffer", self.__meta_length):
                        break

                # Read binary bytes when the binary buffer does not have the
                # correct length yet.
                if len(self.__binary_buffer) < self.__binary_length:
                    if not self.read_part(cur_buffer, "__binary_buffer", self.__binary_length):
                        break

                # Read a complete 'packet', unpack
                if self.__binary_compressed:
                    self.__binary_buffer = zlib.decompress(self.__binary_buffer, 15, self.__binary_length * 2)
                    self.__meta_buffer = zlib.decompress(self.__meta_buffer, 15, self.__meta_length * 2)

                binary_data = self.__binary_buffer
                meta_data = pickle.loads(self.__meta_buffer)
                self.__binary_buffer = ""
                self.__meta_buffer = ""
                self.__header_buffer = ""
                self.__meta_length = None
                self.__binary_length = None
                self.__binary_compressed = False

                # Append to the receive buffer (of ThreadedSocket class)
                with self._ThreadedSocket__receive_lock:
                    self._ThreadedSocket__receive_buffer.append([meta_data, binary_data])

    def __logger(self):
        """
        Wrapper method to obtain the logger of the ThreadedSocket class.
        """
        return self._ThreadedSocket__logger

    def send(self, meta_data, binary_data=""):
        """
        This method enqueues new data to be send. It wraps the data by pickling
        it if configured to do so and creates a new StringIO buffer to store the
        new data.
        """
        if self.listener():
            self.__logger().warning("Cannot send data through a server socket")
            return False

        if not type(binary_data) is type(""):
            self.__logger().warning("Binary data should be a string")

        meta_pickled = pickle.dumps(meta_data)
        if self.__compress:
            binary_data = zlib.compress(binary_data, self.__compress_level)
            meta_pickled = zlib.compress(meta_pickled, self.__compress_level)
            
        # Set meta length, binary length and boolean indicating if compression
        # was used or not in the header string.
        header_str = struct.pack("!II?", len(meta_pickled), len(binary_data), self.__compress)
        data = header_str + meta_pickled + binary_data

        return super(BinarySocket, self).send(data)


################################################################################
## DRIVER                                                                     ##
################################################################################
## The following example will set up a server or a client, depending on if    ##
## there are any command line arguments or not. If there are, it will be a    ##
## server, otherwise a client. The client will read images from ~/testimages  ##
## and send them to the server. The server will receive and show images that  ##
## have been received over the socket.                                        ##
################################################################################
if __name__ == "__main__":
    from ticker import Ticker
    import glob
    import random
    import logging
    import loggingextra
    import hashlib
    import os

    logging.getLogger('Borg.Brain.Util').addHandler(loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain.Util').setLevel(logging.DEBUG)

    server = False
    if len(sys.argv) > 1:
        server = True

    name = "PictureServer" if server else "PictureClient"
    host = "localhost"
    port = "Pictures"

    if not server:
        # The client loads images to send
        path = os.path.join(os.environ['BORG'], "brain", "data", "models", "RobotLab_1", "*.jpg")
        path = os.path.expanduser(path)
        image_names = glob.glob(path)
        images = []
        for filename in image_names:
            print "Loading %s..." % filename
            img = cv.LoadImage(filename)
            images.append(img)

        print "Loaded %d images" % len(images)

    # Start the BinarySocket
    comp = True if not server else False
    sock = BinarySocket(host, port, server=server, bufsize=1024*256, compress=comp, compress_level=9)
    client = sock if not server else None
    ticker = Ticker(5)

    # Keep running until user interrupts
    while True:
        ticker.tick()
        if not client:
            client = sock.wait_connect(0.5)

        if not client:
            continue

        if not client.connected() and server:
            # If a client disconnected, wait for a new connection
            client = None
            continue

        # Get all data from the socket
        data = client.get_data()
        for metadata, binary_data in data:
            # Display the received image
            shape = metadata['shape']
            nchannels = metadata['nChannels']
            depth = metadata['depth']
            digest = metadata['md5']
            h = hashlib.md5()
            h.update(binary_data)
            dig = h.hexdigest()
            if dig == digest:
                print "Correct MD5 sum on binary data: %s" % dig
            else:
                print "Incorrect MD5 sum: %s  (should be %s)" % (dig, digest)
            img = cv.CreateImageHeader(shape, depth, nchannels)
            cv.SetData(img, binary_data)
            cv.ShowImage(name, img)
            cv.WaitKey(30)

        if not server:
            # Send an image to the server
            img = random.choice(images)
            metadata = {"shape": (img.width, img.height),
                        "nChannels": img.nChannels,
                        "depth": img.depth}
            binary_data = img.tostring()
            h = hashlib.md5()
            h.update(binary_data)
            metadata['md5'] = h.hexdigest()
            print "Sending image with checksum: %s" % metadata['md5']
            client.send(metadata, binary_data)


import socket
import pickle
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Util.ReceiveSocket').addHandler(util.nullhandler.NullHandler())

class ReceiveSocket(object):
    """class to receive information over tcp/ip socket"""

    def __init__(self, port=50006):
        """Initialize socket and wait for an incoming connection on port"""
        self.logger = logging.getLogger('Borg.Brain.Util.ReceiveSocket')
        self.logger.info("Waiting for connection on port: %d" % port)
        HOST = ''                 # Symbolic name meaning all available interfaces
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, port))
        s.listen(1)
        self.conn, addr = s.accept()
        self.logger.info("Connected by %s", repr(addr))

    def read(self, do_pickle=True):
        """
        Wait for data on the socket and read it.
        """
        sdata = self.conn.recv(8192) #Temporary fix for large amounts of data
        if not sdata:
            self.conn.close()
            raise Exception('no valid data received on socket')
        self.conn.send('ok') #so sender knows the message has been read
        if do_pickle:
            return pickle.loads(sdata)
        return sdata


    def set_blocking(self, value):
        """
        Sets the blocking mode of the underlying socket that is being used for
        communication.
        """
        self.conn.setblocking(value)

    def __del__(self):
        self.conn.close()



if __name__ == "__main__":

    rc = ReceiveSocket(50006)
    while True:
        print "read: ", rc.read()



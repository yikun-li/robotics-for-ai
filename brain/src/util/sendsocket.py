import socket
import pickle
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Util.SendSocket').addHandler(util.nullhandler.NullHandler())

class SendSocket(object):
    """class to send information over tcp/ip socket"""

    def __init__(self, host, port=50006):
        """initialize socket and conneckt it to given host:port"""
        self.logger = logging.getLogger('Borg.Brain.Util.SendSocket')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((host,port))

    def send(self, data):
        """send a serialized python object through the socket"""
        sdata = pickle.dumps(data)
        bytes = self.socket.send(sdata)
        #print "Bytes sent: ", bytes
        try:
            self.socket.recv(8192) #receive acknowledgement that message has been send
        except socket.error as (errno, str):
            if errno == 104:
                self.logger.error("Connection reset. Send failed")
                return
            else:
                raise

    def send_raw(self, data):
        """send a raw string through the socket"""
        self.socket.send(data)

    def close(self):
        """Close the socket"""
        self.socket.close()

if __name__ == "__main__":
    rc = SendSocket('localhost', 50006)
    for i in range(10):
        rc.send("test " + str(i))
    print "terminated"

import SocketServer
import struct
import math

message = None

class PredatorHandler(SocketServer.BaseRequestHandler):

    def __init__(self, request, client_address, server):
        SocketServer.BaseRequestHandler.__init__(self, request, client_address, server)
        self.timeout = 1

    def handle(self):
        data = self.request[0]
        socket = self.request[1]
        numOfValues = len(data) / 8
        global message
        message = struct.unpack('>' + 'd' * numOfValues, data)

def resetMessage():
    message = None

def observed():
    try:
        return not math.isnan(message[0])
    except:
        return False

def getMessage():
    return message

def getBoundingBox():
    try:
        return ((int)(message[0]), (int)(message[1]), (int)(message[2]), (int)(message[3]))
    except:
        return []

def getConfidence():
    try:
        return message[4]
    except:
        return None

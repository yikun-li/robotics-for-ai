import socket
import configparse
import sys
import threading
import logging

import visioncontroller
import visionmodule_connection_handler
import util.nullhandler

logging.getLogger('Borg.Brain.VisionController').addHandler(util.nullhandler.NullHandler())

class VisionModuleListener(util.threadedsocket.ThreadedSocket):
    """This class listens on a specific port to handle connections"""

    def __init__(self, port, identifier, controller):
        """Start up a listening socket on a specific port and register it with the controller"""
        self.logger = logging.getLogger("Borg.Brain.VisionController.VisionModuleListener")
        util.threadedsocket.ThreadedSocket.__init__(self, '', port, server=True, logger=self.logger)

        # Store controller and connection identifier
        self.controller = controller
        self.identifier = identifier
        self.port = port

    def handle_accept(self, sock, address):
        if sock:
            handler = visionmodule_connection_handler.VisionModuleConnectionHandler(sock, address, self.identifier, self.controller, listening_port=self.port)

    def identifier(self):
        """Return the identifier of this listener, eg the name of the vision module connecting to it"""
        return self.identifier

if __name__ == "__main__":
    controller = None
    server1 = VisionModuleListener(49152, 'First', controller, True)
    server1.start()
    server2 = VisionModuleListener(8081, 'Second', controller, True)
    server2.start()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print "Caught signal. Closing sockets, please wait..."
        server1.stop_listener()
        server2.stop_listener()
        server1.join()
        server2.join()

import socket
import threading
import pickle
import time
import logging

import util.nullhandler
import util.threadedsocket

logging.getLogger('Borg.Brain.VisionController').addHandler(util.nullhandler.NullHandler())

class CommunicatorConnector(util.threadedsocket.ThreadedSocket):
    """This class connects to a vision communicator"""

    def __init__(self, host, port, modules, retry_timeout=5):
        """Initialize the communicator connection handler"""
        util.threadedsocket.ThreadedSocket.__init__(self, host, port, server=False, giveup=-1, retry_timeout=1)

        self.send_buffer = []
        self.host = host
        self.port = port
        self.logger = logging.getLogger("Borg.Brain.VisionController.CommunicatorConnector")

        for module in modules:
            if host == "" or host == "127.0.0.1" or host == "localhost":
                module["port"] = "Vision_%d" % module["port"]
            self.start_module(module["port"], module["command"])

    def send_command(self, command, args={}):
        """
        Send a single command to the communicator
        """
        args['command'] = command
        
        self.send(args)

    def start_module(self, port, module):
        """
        This function sends a start command for a certain module to
        the communicator
        """
        if type(port) is type(""):
            self.logger.debug("Sending start request for module %s on Unix Domain Socket %s" % (module, port))
        else:
            self.logger.debug("Sending start request for module %s on port %d" % (module, port))
        cmd = {"command":   "start_module",
               "port":      port,
               "module":    module}
        self.send(cmd)

    def restart_module(self, port, module):
        """
        This function sends a restart command for a certain module to
        the communicator
        """
        if type(port) is type(""):
            self.logger.debug("Sending restart request for module %s on Unix Domain Socket %s" % (module, port))
        else:
            self.logger.debug("Sending restart request for module %s on port %d" % (module, port))
        cmd = {"command":   "restart_module",
               "module":    module,
               "port":      port}
        self.send(cmd)

    def stop_module(self, port, module):
        """
        This function sends a stop command for a certain module to
        the communicator
        """
        if type(port) is type(""):
            self.logger.debug("Sending stop request for module %s on Unix Domain Socket %s" % (module, port))
        else:
            self.logger.debug("Sending stop request for module %s on port %d" % (module, port))
        cmd = {"command":   "stop_module",
               "port":      port,
               "module":    module}
        self.send(cmd)

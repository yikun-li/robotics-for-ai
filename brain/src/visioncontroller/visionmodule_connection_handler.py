import socket
import pickle
import time
import logging

import util.nullhandler
import util.threadedsocket

logging.getLogger('Borg.Brain.VisionController').addHandler(util.nullhandler.NullHandler())

class VisionModuleConnectionHandler(util.threadedsocket.ThreadedSocket):
    """This class handles an incoming connection"""

    def __init__(self, socket, address, identifier, controller, timeout=10, listening_port=""):
        """Initialize the vision module connection handler"""
        if address:
            self.host, self.port = address
        else:
            self.host = "localhost"
            self.port = listening_port
        util.threadedsocket.ThreadedSocket.__init__(self, self.host, self.port, 0, socket, server=False)

        self.logger = logging.getLogger("Borg.Brain.VisionController.VisionModuleConnectionHandler")
        self.controller = controller
        self.identifier = identifier
        self.module_name = None
        self.heartbeat = time.time()
        self.heartbeat_timeout = timeout

        if controller:
            controller.register_handler(self)

        # Always make sure the client sends its capabilities
        self.send_command("send_capabilities")

    def handle_loop(self):
        """
        This method checks the last time a heartbeat was received, and if that
        was too long ago, it closes the connection.
        """
        if time.time() - self.heartbeat > self.heartbeat_timeout:
            self.logger.warn("No signal from %s in %d seconds. Disconnecting." % (repr(self.address), self.heartbeat_timeout))
            if self.controller:
                host, port = self.address
                self.controller.request_module_restart(self.identifier)
            self.close()

    def handle_receive(self, data):
        for obs in data:
            if "command" not in obs:
                self.send("Command not understood.\r\n")
                continue
            if obs['command'] == "observation" and \
               obs['name'] == 'obstacle_matrix' and \
               'mtime' in obs:
                # For profiling obstacle matrix - add time spent
                # in transit from vision module to here
                scantime = obs['mtime']
                local_delay = obs['local_delay']
                diff = time.time() - scantime
                network_delay = diff - local_delay
                obs['network_delay'] = network_delay
            self.parse_command(obs)

    def handle_shutdown(self):
        """
        This method makes sure the handler gets unregistered in the controller
        when the module shuts down. And it requests a restart. The restart
        method checks if the module should still be running, and if so, it
        requests a restart from the communicator.
        """
        self.logger.info("Module %s no longer connected." % repr(self.identifier))
        if self.controller:
            self.controller.request_module_restart(self.identifier)
            self.controller.unregister_handler(self)
    
    def send_command(self, command, params = {}):
        cmd = {"command": command, "params": params}
        self.send(cmd)

    def parse_command(self, data):
        # Valid message; store time as heartbeat
        self.heartbeat = time.time()
        if data['command'] == "heartbeat":
            # If this is a heartbeat, the time has been stored, so just
            # continue now.
            pass
        elif data['command'] == "module_name":
            # The client sent the name of the module.
            # TODO: validate that the correct module has indeed
            #       connected
            self.module_name = data['module_name']
        elif data['command'] == "recognizable_object" and self.controller:
            # The client sent a recognizable object
            self.controller.add_recognizable_object(data['object_name'], self.identifier, self.address)
        elif data['command'] == "observation" and self.controller:
            # The client sent an observation
            self.controller.add_observation(data, self.identifier, self.address)

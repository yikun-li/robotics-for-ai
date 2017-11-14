import os
import sys
import socket
import threading
import time
import logging
import random
import memory

import configparse
import memory
import visionmodule_listener
import visionmodule_connection_handler
import communicator_connector
import util.nullhandler

logging.getLogger('Borg.Brain.VisionController').addHandler(util.nullhandler.NullHandler())

class VisionController(object):
    """
    Class that transfers vision data from the communicators to the brain
    """
    __instance = None

    ################################
    ## Constructor and destructor ##
    ################################
    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(VisionController, cls).__new__(cls, *args, **kwargs)
        return cls.__instance
    
    def __init__(self):
        """Initialize Visioncontroller, connect with communicators"""
        # Make sure only once initialized; somehow sometimes the constructor
        # is called, even when returning the existing instance...
        if hasattr(self, "initialized"):
            return
        # Initialize data members
        self.logger = logging.getLogger("Borg.Brain.VisionController")
        self.__modules = {}
        self.__module_ports = {}
        self.__module_settings = {}
        self.__listeners = []
        self.__handlers = []
        self.__communicators = []
        self.__observations = []
        self.__data_lock = threading.Lock()
        self.__handler_lock = threading.Lock()
        self.__config = []
        self.__module_check = time.time()
        self.__all_up = False
        self.__stopping = False
        self.__store_sources_time = time.time()
        self.__active_sources = {}
        self.memory = memory.Memory()
        self.initialized = True

    def __del__(self):
        '''destroy the class and its data'''
        #TODO: maybe implement sometime?
        VisionController.__instance__ = None
        pass

    ####################
    ## Public methods ##
    ####################
    def set_config(self, param_dict):
        """
        This method sets the config, and is called in the brain this is _not_
        done in the constructor, since the behaviors construct this class as
        well, and they don't have the param_dict.
        """
        self.__modules = {}
        self.__module_ports = {}
        self.__module_settings = {}
        self.__config = param_dict
        self.__load_configuration()
        self.__start_communication()
        random.seed()

    def update(self):
        """Update the memory with all the new observations"""
        self.check_modules()
        self.check_communicators()

        with self.__data_lock:
            detections = self.__observations
            self.__observations = []

        return detections

    def check_modules(self):
        """
        This method check if all modules that should be connected
        are actually connected, and if not, requests a restart from
        the communicator. A timeout it used so that this is only done
        once every 10 seconds. In the meantime, the result is cached.
        """
        if time.time() - self.__module_check < 10:
            return self.__all_up

        self.__all_up = True
        self.__module_check = time.time()
        for hostname, modules in self.__modules.iteritems():
            # Check if the communicator at this host is connected
            if not self.check_communicator(hostname):
                self.logger.error("check_modules: Not connected to communicator on %s" % hostname)
                self.__all_up = False
                continue
            
            for module in modules:
                found = False
                for handler in self.__handlers:
                    h_hostname, h_module = handler.identifier
                    if h_hostname == hostname and h_module == module:
                        found = True
                        break
                if not found:
                    self.logger.warn("check_modules: Module %s on host %s is not connected. Requesting restart from communicator" % (module, hostname))
                    self.request_module_restart((hostname, module))
                    self.__all_up = False

        if not self.__all_up:
            self.logger.error("Not all vision modules are running. Please check laptop status and communicator output")
        return self.__all_up

    def check_communicators(self):
        for communicator in self.__communicators:
            host = communicator.host
            data = communicator.get_data()
            for item in data:
                if type(item) is type({}) and "command" in item:
                    cmd = item['command']
                    if cmd == "active_sources":
                        self.__active_sources[host] = item['active_sources'] 

        if self.__store_sources_time < time.time() - 2:
            self.__store_sources_time = time.time()
            obs = {"name": "active_video_sources",
                   "time": time.time(),
                   "property_dict": self.__active_sources}

            with self.__data_lock:
                self.__observations.append(obs)

    def add_recognizable_object(self, data, identifier, source):
        """
        This method adds a new recognizable object to the memory. This method
        is to be called from a VisionModuleConnectionHandler.
        """
        with self.__data_lock:
            m = memory.Memory()
            m.add_recognizable_object(identifier, data)

    def get_modules(self, host):
        """get_modules returns the modules for a specified hostname"""
        modules = []
        for port, service in self.__module_ports.iteritems():
            (module_host, module) = service
            if module not in self.__module_settings:
                continue

            command = self.__module_settings[module]
            if host == module_host:
                modules.append({"port": port, "command": command})
        return modules

    def stop_connections(self):
        """
        stop_connections closes all connections managed by the vision
        controller. It first requests closing to all sockets and then
        waits to join all threads that have been spawned. When this
        method returns, all threads spawned by the VisionController
        will have terminated
        """
        self.__stopping = True
        # First instruct all threads to stop
        for listener in self.__listeners:
            listener.close()

        for handler in self.__handlers:
            handler.close()

        for communicator in self.__communicators:
            communicator.close()

    #######################################################
    ## Methods for VisionModuleConnectionHandler objects ##
    #######################################################
    def add_observation(self, observation, identifier, source):
        """
        This method adds a new observation to the observation stack

        Observation should be a dictionary with 'property':value pairs.
        At least 'command':'observation' should be present.
        If 'name':NAME is present, this will be the name in memory.
        otherwise, the name in memory will be 'vision.unknown'
        """
        # Wrap the data, suitable for memory
        observation['source'] = source
        observation['identifier'] = identifier
        
        if "command" in observation:
            observation.pop('command') #removes command from observation

        if "name" in observation:
            name = observation.pop('name') #removes name from observation
        else:
            name = "vision.unknown"

        if "time" in observation:
            obs_time = observation.pop('time') #removes time from observation
        else:
            obs_time = time.time()

        # Add the observation to the memory
        self.memory.add_item(name, obs_time, observation)

        #obs = {"name": name,
        #       "time": obs_time,
        #       "property_dict": observation}

        ##print 'OBJECT: ', object
        #with self.__data_lock:
        #    self.__observations.append(obs)


    ###################################################################
    # Networking methods: handle connections, send commands, receive ##
    ###################################################################
    ## Communicator methods ##
    ##########################
    def connect_communicator(self, hostname, port):
        """Connect to a communicator"""
        modules = self.get_modules(hostname)
        self.logger.debug("Modules to start on host %s: %s" % (hostname, repr(modules)))
        try:
            if hostname == "" or hostname == "127.0.0.1" or hostname == "localhost":
                communicator = communicator_connector.CommunicatorConnector(hostname, "communicator", modules)
            else:
                communicator = communicator_connector.CommunicatorConnector(hostname, port, modules)
        except socket.error as (errno, str):
            if errno == 61:
                if hostname == "" or hostname == "127.0.0.1" or hostname == "localhost":
                    self.logger.critical("Unable to connect to communicator on Unix Domain Socket %s: %s" % ("communicator", str))
                else:
                    self.logger.critical("Unable to connect to communicator at %s on port %d: %s" % (hostname, port, str))
                self.stop_connections()
                quit()
            else:
                self.stop_connections()
                raise

        self.__communicators.append(communicator)
        communicator.start()

    def check_communicator(self, hostname):
        """
        Checks if there is a working connection to the specified communicator
        """
        for comm in self.__communicators:
            if comm.host == hostname:
                # Return the connection state of this communicator connection
                return comm.connected
        # No connection to this communicator found
        return False 

    def communicator_command(self, hostname, command, args = {}):
        for comm in self.__communicators:
            if comm.host == hostname:
                return comm.send_command(command, args)
        self.logger.warn("No connection to a communicator on host %s; cannot send command %s" % (hostname, command))
                

    ##################################################
    ## Starting, restarting and stopping of modules ##
    ##################################################
    def start_module(self, hostname, module):
        """This method adds a module for a certain host"""
        if module not in self.__module_settings:
            self.logger.warning("Trying to start unknown module %s on " \
                  "host %s." % (module, hostname))
            return False

        new_port = self.__get_available_port(self.start_port, self.end_port)
        service = (hostname, module)
        command = self.__module_settings[module]
        if hostname not in self.__modules:
            # Hostname unknown so far, so no communicator connection
            self.__modules[hostname] = [module]
            self.__module_ports[new_port] = service
            if hostname == "" or hostname == "localhost" or hostname == "127.0.0.1":
                new_port = "Vision_%d" % new_port
            self.start_listener(new_port, service)
            # Connect to the communicator, this will automatically request
            # the starting of the module
            self.connect_communicator(hostname, self.communicator_port)
        else:
            if module not in self.__modules[hostname]:
                self.__module_ports[new_port] = service
                if hostname == "" or hostname == "localhost" or hostname == "127.0.0.1":
                    new_port = "Vision_%d" % new_port
                self.start_listener(new_port, service)
                self.__modules[hostname].append(module)
                # Find the communicator and start the module
                for communicator in self.__communicators:
                    if communicator.host == hostname:
                        communicator.start_module(new_port, command)
        return True

    def request_module_restart(self, identifier):
        """"This method requests a restart of a module from a host"""
        # When we're quiting, request no restarts
        if self.__stopping:
            return

        hostname, module = identifier
        self.logger.debug("Request received to restart module %s on host %s" % (module, hostname))
        if module not in self.__module_settings:
            return
        command = self.__module_settings[module]

        # First check if this module should actually be running on this host
        started_modules = self.get_modules(hostname)
        module_should_run = False
        port = 0
        for module in started_modules:
            if module['command'] == command:
                module_should_run = True
                port = module['port']
                break

        if not module_should_run:
            self.logger.warn("Module %s shouldn't be running, so not restarting" % module)
            return # Do not attempt to restart

        for communicator in self.__communicators:   
            if communicator.host == hostname:
                if hostname == "" or hostname == "localhost" or hostname == "127.0.0.1":
                    port = "Vision_%d" % port
                communicator.restart_module(port, command)

    def stop_module(self, hostname, module):
        """This method stops a module for a certain host"""
        self.logger.debug("Stopping module %s:%s" % (hostname, module))
        service = (hostname, module)

        # First check if this module should actually be running on this host
        if module not in self.__module_settings:
            self.logger.warn("Module %s is unknown. Make sure it is defined in the config file!" % module)
            return
        command = self.__module_settings[module]
        self.logger.debug("Command: %s" % command)

        started_modules = self.get_modules(hostname)
        module_should_run = False
        port = 0
        for started_module in started_modules:
            if started_module['command'] == command:
                module_should_run = True
                port = started_module['port']
                break

        if not module_should_run:
            self.logger.debug("Module %s shouldn't be running, can't stop." % module)
            return

        # Remove the listening port from list of ports
        for port, module_service in self.__module_ports.iteritems():
            if service == module_service:
                del self.__module_ports[port]
                break

        # Request module stop from the communicator
        for communicator in self.__communicators:
            if communicator.host == hostname:
                comm_port = port
                if hostname == "127.0.0.1" or hostname == "localhost":
                    comm_port = "Vision_%s" % port
                communicator.stop_module(comm_port, command)
        
        # Close listening connections for this module
        for idx, listener in enumerate(self.__listeners):
            if listener.identifier == service:
                listener.close()
                del self.__listeners[idx]
                break

        # Close active connections for this module
        for idx, handler in enumerate(self.__handlers):
            if handler.identifier == service:
                handler.close()
                break

        # Remove the module from the module list for this host
        if hostname in self.__modules:
            if module in self.__modules[hostname]:
                self.__modules[hostname].remove(module)


    ###########################
    ## Vision module methods ##
    ###########################
    def send_command(self, hostname, module, command, params = {}):
        """
        Send a command to a specific module. Returns true on success, false if
        the specified module/hostname combination currently has no connection
        to the vision controller
        """
        # Find the proper module
        identifier = (hostname, module)
        with self.__handler_lock:
            for conn in self.__handlers:
                if conn.identifier == identifier:
                    # We found the proper module
                    conn.send_command(command, params)
                    self.logger.debug("Command %s sent to module %s on host %s" % (command, hostname, module))
                    return True
        self.logger.warning("Module %s on host %s does not have a connection, unable to send command." % (module, hostname))
        return False

    def start_listener(self, port, identifier):
        """This method starts a listener on a specified port"""
        try:
            listener = visionmodule_listener.VisionModuleListener(port, identifier, self)
        except socket.error as (errno, str):
            if errno == 48:
                if type(port) == type(""):
                    self.logger.critical("Unable to listen on Unix Domain Socket %s: %s" % \
                                         (port, str))
                else:
                    self.logger.critical("Unable to listen on port %d: %s" % \
                                         (port, str))
                self.stop_connections()
                quit()
            else:
                self.stop_connections()
                raise

        self.__listeners.append(listener)
        listener.start()

    def register_handler(self, handler):
        """This method registers a new connection handler"""
        with self.__handler_lock:
            self.__handlers.append(handler)

    def unregister_handler(self, handler):
        """This method registers a new connection handler"""
        with self.__handler_lock:
            self.__handlers.remove(handler)

    #####################
    ## PRIVATE METHODS ##
    #####################
    def __load_configuration(self):
        """Load the configuration from the param dictionary, or use defaults"""
        vision_params = self.__config.get_section('vision_controller')

        # Strip whitespace, split by newlines and then filter out empty elements
        modules_settings = filter(None, self.__get_option('modules_settings', '').strip().split('\n'))
        modules = filter(None, self.__get_option('modules', '').strip().split('\n'))

        self.communicator_port = int(self.__get_option('communicator_port', '49152'))

        self.start_port = int(self.__get_option('start_port', '50000'))
        self.end_port = int(self.__get_option('end_port', '52000'))

        # Parse modules and module settings
        self.__parse_settings(modules_settings)
        self.__parse_modules(modules)
        self.__assign_ports()

    def __get_option(self, option, default):
        """
        Returns an option from the configuration file, or a default
        value if it is not present in the configuration
        """
        val = self.__config.get_option('vision_controller', option)
        if val is None:
            return default
        return val

    def __start_communication(self):
        """Start up communication with vision modules and communicators"""
        # Start listeners for all vision modules
        for port, service in self.__module_ports.iteritems():
            hostname, module = service
            if hostname == "127.0.0.1" or hostname == "" or hostname == "localhost":
                port = "Vision_%d" % port
            self.start_listener(port, service)

        # Connect to communicators
        for hostname, host_modules in self.__modules.iteritems():
            self.connect_communicator(hostname, self.communicator_port)

    def __parse_modules(self, modules):
        """Parses the modules for each hostname from the configuration file"""
        for module in modules:
            (hostname, sep, modules) = module.partition('=')
            hostname = hostname.strip()
            host_modules = modules.strip().split()
            proper_modules = []
            for module in host_modules:
                if module in self.__module_settings:
                    proper_modules.append(module)
                else:
                    self.logger.warn("Configuration file says to start module " \
                                     "%s on host %s but there are no module " \
                                     "settings defined for this module. Please " \
                                     "check the configuration file for typo's" \
                                     % (module, hostname))
            self.__modules[hostname] = proper_modules

    def __parse_settings(self, modules_settings):
        """Parses the settings for each module from the configuration file"""
        for setting in modules_settings:
            (module, sep, command) = setting.partition('=')
            self.__module_settings[module.strip()] = command.strip()

    def __assign_ports(self):
        """Assign ports to all modules to be activated"""
        for hostname, modules in self.__modules.iteritems():
            for module in modules:
                available_port = self.__get_available_port(self.start_port, self.end_port)
                self.__module_ports[available_port] = (hostname, module)

    def __get_available_port(self, start_port, end_port):
        """Find a port available for listing in the range defined in the config file"""
        for i in range(end_port - start_port):
            port = random.randint(start_port, end_port)
            if port in self.__module_ports:
                continue
            return port
        raise Exception('No valid ports left')

if __name__ == "__main__":
    logging.getLogger('Borg.Brain').addHandler(logging.StreamHandler(sys.stdout))
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)
    config_dict = configparse.ParameterDict()
    configparse.parse_config(sys.argv[1], config_dict)
    vc = VisionController(config_dict)
    try:
        while True:
            data = vc.update()
            if data:
                print "received: ", data
            time.sleep(1)
    except KeyboardInterrupt:
        print "Caught signal. Closing sockets, please wait..."
        vc.stop_connections()

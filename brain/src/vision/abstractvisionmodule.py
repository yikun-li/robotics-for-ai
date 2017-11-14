import abc
import sys
import time
import logging

import util.threadedsocket


class AbstractVisionModule:
    """
    Abstract class which contains methods and properties 
    that have to be implemented in each vision module
    This is required to enforce compatibility with the
    the Vision Controller and the Communicator
    """
    __metaclass__ = abc.ABCMeta
    
    ################################################################
    # Properties
    ################################################################
    
    __socket = None
    __heartbeat = None 
    __known_objects = []
    __send_observation = {}
    __send_data = []
    
    ################################################################
    # Abstract Methods
    ################################################################    
    @abc.abstractmethod
    def __init__(self, host, port):
        """
        Should be overwritten with additional parameters.
        Parameters will probably come from the command line 
        when calling myModule.py param1=True param2=99
        """
        # Initialize the logger if it has not been done yet
        if not hasattr(self, "logger") or not self.logger:
            self.logger = logging.getLogger('Borg.Brain.Vision.AbstractVisionModule')
        self.__socket = None
        self.__host = host
        self.__port = port
        self.__heartbeat = None
        self.__known_objects =[]
        self.__send_observation = {'command': 'observation', 'name': 'unknown'}
        self.__send_data = []
                
    @abc.abstractmethod
    def train(self):
        """ 
        This method should be called for the training of the module.
        You have to end this method with self.set_known_objects()
        in order to send to the Vision Controller the list
        of recognizable objects.
        Parameter for function is list: ['couch','seat','etc']
        """
        # do some processing
        # finally call function to set the known objects
        my_objects = ['cat','dog','mouse']
        self.set_known_objects(my_objects)
        
    @abc.abstractmethod
    def run(self):
        """
        This method should be called when using the module, therefore
        after the training of the module has been performed.
        You can add properties of your detections that will be sent to the
        memory using the add_property method. 
        self.add_property('size', 25) then this will be included in the 
        dictionary part of the property_dict sent to the memory.
        You should ALWAYS include the "name" property otherwise it will be
        sent to the memory as "unknown"
        """
        while True:
            #process the next image (the actual stuff the module is supposed to do)
            #sleep a bit (depending on update frequency) 
            self.update() # and update!
        
    ################################################################
    # Shared Methods
    ################################################################
    
    # Public Methods ###############################################
    # These are the only methods that can be called in the __main__
    
    def connect(self):
        """
        Initiates the connection to the Vision Controller.
        This method should be called before the train() or the run() methods.
        Immediately after the connection is established the Vision Controller
        waits for the list of known objects (detectable) by the module.
        """
        if not self.__socket or not self.__socket.connected():
            if not self.__socket:
                self.__socket = util.threadedsocket.ThreadedSocket(self.__host, self.__port, giveup=5, use_pickle=True, server=False, logger=self.logger)
            self.__heartbeat = time.time()
            self.__socket.wait_connect()
        else:
            self.logger.warn("Already connected on port: %d" % self.__socket.get_port())
        
    def disconnect(self):
        """ Close the connection to the controller """
        if self.__socket:
            self.__socket.close()
            self.__socket = None
        
    def is_connected(self):
        """ Returns true if the connection to the controller is active """
        return self.__socket and self.__socket.connected()
        
    def set_known_objects(self, objects):
        """ Sets the objects that can be recognized by this module in list format """
        if type(objects).__name__ != 'list':
            self.logger.error("This function takes in a list of objects, you provided: %s" % type(objects).__name__)
            return
        self.__known_objects = objects
        
    def add_property(self, prop, value):
        """ Adds a property to the property_dict sent to the memory """
        if prop != 'command':
            self.__send_observation[prop] = value;
        else:
            self.logger.warning("You can't use the word 'command' as a property name, it is reserved for the communication protocol with the controller")
            return
        
    def store_observation(self):
        """ Stores one observation (one detected entity) """
        if len(self.__send_observation) > 2 or self.__send_observation['name'] != 'unknown':
            self.__send_data.append(self.__send_observation)
            # reset data to be send, so we don't send it again
            self.__send_observation = {'command': 'observation', 'name': 'unknown'}

    def update(self):
        """ Gathers data and sends it to the vision controller """
        if not self.is_connected():
            self.logger.error("Socket is not connected. Cannot update.")
            return False
        
        # send keep-alive signal
        self.__send_heartbeat()

        # Since vision modules are run piped through tee, output
        # is buffered. Therefore, flush the stdout and stderr buffers.
        sys.stdout.flush()
        sys.stderr.flush()
        
        passed = True
        # process commands from the vision controller
        data = self.__socket.get_data()
        for entry in data:
            if 'command' in entry:
                passed = self.__handle_command(entry)

        # if there has been any observation appended, send it
        self.store_observation()
        if self.__send_data:
            for obj in self.__send_data:
                self.__socket.send(obj)
                self.__heartbeat = time.time()
            self.__send_data = []

        return passed

    def handle_custom_commands(self, entry):
        """
        This method can be overridden by vision modules to handle custom commands
        from the vision controller. It is called from the __handle_command method
        when a command has been received that is not send_capabilities. It should
        return True when it has handled the command and False if it does not know
        the command.
        """
        return False

    def set_socket_verbose(self, read, write):
        """
        This method can be used to set enable or disable the logging of data
        sent and received over the socket.
        """
        self.__socket.set_verbose(read, write)
        
    # Private Methods ##############################################
    # These are only used internally and should not be public

    def __send_heartbeat(self):
        """
        Checks the last time of communication then sends heartbeat if necessary.
        This is needed because there is no other way to check if
        the python process (this running module) is hanging. This is useful
        because it could be still running but not doing anything, so you see a PID,
        but the module is not actually performing it's functions.
        """
        if self.__heartbeat < time.time() - 5:
            self.__socket.send({"command": "heartbeat"})
            self.__heartbeat = time.time()
    
    def __send_known_objects(self):
        """ Sends the list of detectable objects to the vision controller """
        if self.__known_objects == []:
            self.logger.warn("You did not set the list of recognizable objects. Use self.set_known_objects([list]) to do that")
            return False
        for obj in self.__known_objects:
            self.__socket.send({"command": "recognizable_object", "object_name": obj})
        return True

    def __handle_command(self, entry):
        """
        Handles special protocol commands from the controller.
        This can be extended with further extensions of the vision controller
        """
        passed = True
        if entry['command'] == "send_capabilities":
            passed = self.__send_known_objects()
        else:
            passed = self.handle_custom_commands(entry)
        return passed
        
if __name__ == "__main__":
    print " This file can not be instantiated and should be used to extend a vision module class"
    print " It is necessary to overwrite the __init__() , train() and run() methods"
    print " You should open the file to view the other methods and how they are used"
    print " After the extension your module file should contain something like this in the __main__:"
    print "module = MyFancyModule(host, port, param1, param2)"
    print "if module.connected(): "
    print "    while True: "
    print "        module.train() "
    print "        mocule.connect() "
    print "        module.run()"


import util.receivesocket
import os
import time
import logging
import util.nullhandler

import body.bodycontroller

logging.getLogger('Borg.Brain.BodyController').addHandler(util.nullhandler.NullHandler())

class SpeechController(object):
    """
    Class that manages all speech recognition classes
    """
    def __init__(self, param_dict):
        """Initialize Speechcontroller and constituent speech recognition systems"""
        self.logger = logging.getLogger('Borg.Brain.Speech.SpeechController')
        speech_params = param_dict.get_section('speech_controller')
        modulestring = param_dict.get_option('speech_controller','modules')
        self.logger.debug("ModuleString: " + repr(modulestring))
        modules = self.check_module_string(modulestring)
        self.logger.debug("Modules: %s" % repr(modules))
        self.receive_sockets = []
        for module in modules:
            self.connect_module(speech_params, module)

    def __del__(self):
        pass

    def check_module_string(self, modulestring):
        """Start all modules in the modulestring"""
        if modulestring != None:
            modules = modulestring.split()
        else:
            modules = []
        return modules

    def connect_module(self, speech_params, module):
        """Connect to a module on the port specified in the params"""
        print "starting in connect module " + speech_params['speech_port']
        self.receive_sockets = []
        try:
#            port = int(speech_params[module])
            port = int(speech_params['speech_port'])
            self.logger.info("Port speech controller: %d" % port)
            socket = util.receivesocket.ReceiveSocket(port)
            socket.set_blocking(0)   
            self.receive_sockets.append(socket)
            self.logger.debug("Speech controller is waiting for connection")
        except Exception as e:
            self.logger.critical("Exception: %s" % e)
            self.logger.critical("It could be no socket was found for module `%s', is it in the config file?" % module)
            raise e

    def update(self):
        """Get update from all speech modules"""
        self.detections = []
        for socket in self.receive_sockets:
            received_string = None
            try:
                received_string = socket.read(False)
                self.logger.debug("This is the received string: " + received_string)
            except Exception as e:
                #TODO:XXX: check if the exception is indeed raised just because
                #there is no data available
                #print e
                pass

            if not received_string == None:
                #Detection should only be appended if the Nao does not speak:
                body_ctrl = body.bodycontroller.BodyController()
                if (body_ctrl.get_nr_naos() > 0 and not body_ctrl.nao(0).is_speaking()) or body_ctrl.get_nr_naos() == 0:
                    command = {'name':'voice_command', 'time':time.time(), 'property_dict':{'message':received_string}}          
                    self.detections.append(command)
        return self.detections


if __name__ == "__main__":
    pass

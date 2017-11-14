import speech.speechcontroller
import speech.testcontroller
import visioncontroller.visioncontroller
#import sonar.sonarcontroller
import navigation.navigationcontroller
import logging
import memory
import util.nullhandler

logging.getLogger('Borg.Brain.SensorIntegrator').addHandler(util.nullhandler.NullHandler())

class SensorIntegrator(object):
    def __init__(self, param_dict):
        """initialize SensorIntegrator class and constituent sensor controllers"""
        #TODO: initialize using using the params in param_dict.get_section['sensor_integration']
        self.logger = logging.getLogger("Borg.Brain.SensorIntegrator")
        
        self.speechcontroller = None
        
        if param_dict.get_option('speech_controller', 'start_speech'):
            # self.speechcontroller = speech.speechcontroller.SpeechController(param_dict)
            self.speechcontroller = speech.testcontroller.TestController()
            self.speechcontroller.set_config(param_dict)
            self.logger.debug("Speech Controller initialized")
        self.visioncontroller = visioncontroller.visioncontroller.VisionController()
        self.visioncontroller.set_config(param_dict)
        self.logger.debug("Vision Controller initialized")
        #self.sonarcontroller = sonar.sonarcontroller.SonarController(param_dict)
        self.navigationcontroller = navigation.navigationcontroller.NavigationController(param_dict)
        self.logger.debug("Navigation Controller initialized")
        self.memory = memory.Memory()
        self.logger.debug("Memory initialized")

    def stop(self):
        print "Stopping vision controller... "
        self.visioncontroller.stop_connections()
        print "Done"

    def update(self):
        """update all sensor controllers and add all detections to memory"""
        if self.speechcontroller:
            self.add_to_memory(self.speechcontroller.update())
        self.add_to_memory(self.visioncontroller.update())
        #self.add_to_memory(self.sonarcontroller.update())
        self.add_to_memory(self.navigationcontroller.update())
        return self.memory

    def add_to_memory(self, objects):
        """add a list of dictionaries describing objects to memory database"""
        if objects:
            for object in objects:
                #print 'OBJECT: ', object
                self.memory.add_item( object['name'], object['time'], object['property_dict'])

if __name__ == "__main__":
    print "Hello World";

import time
import argparse
import threading
import subprocess
import socket
import sys,re
import util.nullhandler
import body.bodycontroller
sys.path.append('speech/hark-sphinx')
import Speechrecognizer

# ugly, but is needed for the behavior speechcontroller and the sensorintegrator speechcontroller to share variables.
# for the future we could make this a singleton, like memory, but this takes more time.
recognizer = None
asking = False

class TestController(object):

    def __init__(self):
        # initialize 
        self.message = 'This is a controller'
        self.test = True
        self.body = body.bodycontroller.BodyController()
        self.last_obs = time.time()
        self.new_obs = {'result' : None, 'azimuth': None ,'recogtime': None}
        self.test_obs = {'message' : "I'm connected"}
        self.pause = False
        #self.ask()

    def set_config(self, param_dict):
        speech_params = param_dict.get_section('speech_controller')
        self.grammar = param_dict.get_option('speech_controller', 'grammar')
        source = param_dict.get_option('speech_controller', 'source')
        if source == None:
            source = 'mic'
            #source = 'kinect' is the other option
        global recognizer
        recognizer = Speechrecognizer.Speechrecognizer({'source':source})
        if not self.grammar == None:
            recognizer.set_gram('speech/hark-sphinx/grammar/' + self.grammar)
        recognizer.start()
        print recognizer, 'recognizer value'

    def ask(self,question = None,cmds = None):
        global asking 
        asking = True
        print recognizer, 'recognizer value'
        if question == None:
            question = 'Are you sure about this?'
        if cmds == None:
            cmds = ['yes alice','no alice','i am not sure']
        #self.body.say(question)
        with open('closed.gram', 'w') as f:
            f.write('#JSGF V1.0;\n\ngrammar vcc;\n\npublic <all> = <response>;\n\n')
            f.write('<response> = (')
            buf = ''
            for command in cmds:
                buf += ' ' + command + ' |'
            f.write(buf[:-1] + ');')
        recognizer.set_gram('closed.gram')

    def update(self):
        global asking
        global recognizer
        #print self.message
        self.detections = []
        if not self.pause and self.body.is_speaking():
            print "[SpeechController] speech detected, pauzing"
            self.pause = True
            recognizer.pause = True

        if self.pause and not self.body.is_speaking():
            print "[SpeechController] done speaking, resuming"
            self.pause = False
            recognizer.pause = False

            #todo send neatly to recognizer
        #self.detections.append({'name':'voice_command', 'time':time.time(), 'property_dict':self.test_obs})          

        self.new_obs = recognizer.get_obs()
        if not self.new_obs['result'] == None:
            if not self.new_obs['recogtime'] == self.last_obs:
                self.last_obs = self.new_obs['recogtime']
                send_obs = {'message' : self.new_obs['result'], 'azimuth' : self.new_obs['azimuth'], '2best' : self.new_obs['2best']} 
                command = {'name':'voice_command', 'time':time.time(), 'property_dict':send_obs}
                if self.pause:
                    print "[SpeechController]", self.new_obs['result'], ", IGNORED"
                else:
                    print "[SpeechController]", self.new_obs['result']
                    self.detections.append(command)
                    if asking:
                        # if the question was asked and a valid answer returned, continue with the normal grammer
                        print "[SpeechController] switching back to original grammar"
                        asking = False
                        recognizer.set_gram('speech/hark-sphinx/grammar/' + self.grammar)      
        return self.detections

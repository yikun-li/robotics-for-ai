import basebehavior.behaviorimplementation
import time
import sys, re
from JSGFParser import JSGFParser
import datetime


class SpeechToplevel_x(basebehavior.behaviorimplementation.BehaviorImplementation):
    
    def implementation_init(self):
        print "Speech Behavior Started!"
        self.last_recogtime         = time.time()
        self.state                  = 'idle'
        self.last_speech_obs        = None  # contains the result and 2best form sphinx
        self.new_speech_obs         = False # is used for triggering the conversation 
        
        self.grammar = JSGFParser('speech/hark-sphinx/grammar/command.gram')
        self.navigations = self.grammar.findVariableItems("<navigation>", includeVars = False)
        self.sentences = self.grammar.findVariableItems("<sentence>", includeVars = False)
        self.requests = self.grammar.findVariableItems("<request>", includeVars = False)
        self.locations = self.grammar.findVariableItems("<location>", includeVars = False)
        pass


    def implementation_update(self):
        self.update_last_speech_command()

        if self.new_speech_obs:
#             print self.last_speech_obs
            msg_opts_removed = self.remove_opts(self.last_speech_obs['message'])
            goto_location = self.find_location(msg_opts_removed)
                        
            if goto_location:
                print "Navigate the robot to %s." %goto_location
                
            elif msg_opts_removed.find('What time is it') != -1:
                print datetime.datetime.now()
                
            elif msg_opts_removed.find('What is the oldest') != -1:
                if msg_opts_removed.find('most widely used drug on earth'):
                    print 'coffee'
                
            elif msg_opts_removed.find('Who are your creators') != -1:
                print 'Krystal and Lee'
                
            elif msg_opts_removed.find('approach') != -1:
                if msg_opts_removed.find('the dining table'):
                    print 'Approach to dinning table.'
                    
            else:
                print 'Please try again.'
            
        pass


    # remove words that are between []'s
    def remove_opts(self, hypstr):
        return self.grammar.filterOptionals(hypstr)


    def find_location(self, string):
        regex_str = "(%s)" % '|'.join(self.locations)
        regex = re.compile(regex_str) 
        match = re.split(regex, string)
        print match
        if len(match) < 3: return None
        return match[1]

    
    def update_last_speech_command(self):
        # sets the new_command boolean and sets the last understood speech_observation
        if (self.m.n_occurs('voice_command') > 0):
            (recogtime, obs) = self.m.get_last_observation("voice_command")
            if not obs == None and recogtime > self.last_recogtime:
                # print "[observation] = ",obs
                self.last_speech_obs = obs
                self.last_recogtime = recogtime
                self.new_speech_obs = True
            else:
                self.new_speech_obs = False

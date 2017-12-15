
import basebehavior.behaviorimplementation
import time
import sys, re
from JSGFParser import JSGFParser

class ExampleSpeech_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        print "Speech Example Behavior Started!"
        self.last_recogtime         = time.time()
        self.state                  = 'idle'
        self.last_speech_obs        = None  # contains the result and 2best form sphinx
        self.new_speech_obs         = False # is used for triggering the conversation 
        
        self.grammar = JSGFParser('speech/hark-sphinx/grammar/rfai1.gram')
        self.names = self.grammar.findVariableItems("<name>", includeVars=False)
        self.tas = self.grammar.findVariableItems("<ta>", includeVars=False)


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


    def implementation_update(self):
        self.update_last_speech_command()

        if self.new_speech_obs:
            
            print self.last_speech_obs
            print self.remove_name(self.last_speech_obs['message'])
            the_ta = self.find_ta(self.last_speech_obs['message'])
            if the_ta:
                print "so your favorite TA is", the_ta 

            self.new_speech_obs = False


    ### Helper functions based on JSGFParser ###

    # strips the robot name from the sentence
    def remove_name(self, s):
        for name in self.names:
            start = "%s " % name
            if  s.startswith(start):
                s = s.lstrip(start)
        return s

    # remove words that are between []'s
    def remove_opts(self, hypstr):
        return self.grammar.filterOptionals(hypstr)

    # if the string was part of a <var>, return said var
    def find_type(self, string):
        if self.remove_opts(string) in self.names: return "names"
        if self.remove_opts(string) in self.tas: return "tas"
        return None

    # example function to find a TA
    def find_ta(self, string):
        regex = re.compile("(%s)" % '|'.join(self.tas))
        match = re.split(regex, self.remove_name(string))

        if len(match)<3: return None
        return match[1]

    # takes as input two hypotheses, s1, s2; and splits both parts
    # on the key " and ", to allow comparing the two hypotheses 
    # per part
    def split_order(self, s1, s2):
        splits = []
        regex = re.compile(" and ")
        matches1 = re.split(regex, s1)
        matches2 = re.split(regex, s2)
        for x, y in zip(matches1,matches2):
            splits.append([x,y])
        return splits
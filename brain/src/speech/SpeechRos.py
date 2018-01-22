import time
import argparse
import threading
import subprocess
import socket, select
import sys,re
import json
import util.nullhandler
import body.bodycontroller
import roslib; roslib.load_manifest('alice_msgs')
import rospy
import ossaudiodev
import struct, os
import math
import pyaudio
from collections import OrderedDict
from std_msgs.msg import String
from alice_msgs.srv import MemorySrv
from alice_msgs.srv import MemoryReadSrv
from pocketsphinx import *
from sphinxbase import *

class Speechrecognizer(threading.Thread):
    def __init__(self, args=None):
        threading.Thread.__init__(self)

        self.obs = {'result' : None, 'azimuth': None ,'recogtime': None,'2best' : None}
        self.running = True
        self.args = args
        self.pause = False
        self.hark_process = None
        
        # setup PocketSphinx
        hmm = '../../../speech/hark-sphinx/model/en-us'
        lm = '../../../speech/hark-sphinx/model/en-us.lm'
        dic = '../../../speech/hark-sphinx/model/cmu07a.dic'

        config = Decoder.default_config()
        config.set_string('-logfn', '/dev/null') #comment for debug information
        config.set_string('-hmm', hmm)
        config.set_string('-lm', lm)
        config.set_string('-dict', dic)
        # config.set_string('-rawlogdir', 'logs') #uncomment to log the raw stream data
        print "[SpeechRecognizer] loading language model ..."
        self.decoder = Decoder(config)
        print "[SpeechRecognizer] done."
        
        self.grammarcontent = None      # the whole grammarfile for regex
        self.grammaropts = []         # optional and therefore disregarded words

    def get_obs(self):

        return self.obs

    def run(self):
        try:
            print "[SpeechRecognizer] Running SpeechRecognizer thread"
            if self.args == None:
                self.listen_kinect()
            else:
                if self.args['source'] == "kinect":
                    self.listen_kinect()
                if self.args['source'] == "mic":
                    self.listen_mic()
                if self.args['source'] == "wav":
                    self.listen_wav()
        except IOError:
            print "[SpeechRecognizer] SpeechRecognizer thread killed"

    def get_optional_words(self):
        p = re.compile(ur'(?<=\[)[a-z< >]+(?=\])')
        return re.findall(p, self.grammarcontent)

    def filter_optional_words(self, phrase):
        for optional in self.grammaropts:
            phrase = phrase.replace(" %s " % optional, " ")
        return phrase

    def get2best(self, decoder):
        nBestList = {}
        for oneBest in decoder.nbest():
            if not oneBest.hyp() is None:
                score = oneBest.hyp().best_score 
                hypstr = oneBest.hyp().hypstr
                hypstr = self.filter_optional_words(hypstr)
                
                if not hypstr in nBestList:
                    nBestList[hypstr] = 0
                nBestList[hypstr] += 1

        decodersBest = self.filter_optional_words(decoder.hyp().hypstr) # best hypothesis
        nBestList[decodersBest] = 999999 # make sure that the decoders best has the best rank always
        nBestListOrdered = list(OrderedDict(sorted(nBestList.items(), key=lambda t: t[1], reverse=True)))
        nBestListOrdered = nBestListOrdered[0:2]
        if len(nBestListOrdered) < 2:
            return [nBestListOrdered[0],nBestListOrdered[0]]
        return nBestListOrdered
    
    def set_gram(self, gramfile):
        jsgf = Jsgf(gramfile)
        rule = jsgf.get_rule('vcc.all')
        fsg = jsgf.build_fsg(rule, self.decoder.get_logmath(), 7.5)
        #fsg.writefile('grammar/vcc.fsg')
        self.decoder.set_fsg("newgram", fsg)
        self.decoder.set_search("newgram")
        with open(gramfile, "r+") as grammarFile:
            self.grammarcontent = grammarFile.read()
        self.grammaropts = self.get_optional_words()
        print "[SpeechRecognizer] switched to grammar",gramfile

    def listen_kinect(self, HOST = "localhost", PORT = 5530):
         
        #simsrc denotes the number of simulataneous sources that can be tracked
        time.sleep(5)
        simsrc = 3
        debug = False
        raw = [{'active': False, 'raw': '', 'ID': -1, 'azimuth': 0.0} for x in range(simsrc)]

        # define decoding structures
        header = struct.Struct('3I 4I')
        srcinfo = struct.Struct('I 4f')
        numsrc = struct.Struct('I')
        srcdata = struct.Struct('2I')

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind((HOST, PORT))
        except socket.error , msg:
            print '[SpeechRecognizer] Connection to Hark failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
            sys.exit()

        s.listen(1)

        self.hark_process = subprocess.Popen(["./speech/hark-sphinx/hark-config/kinect_network.n"],stderr=subprocess.STDOUT)    
        time.sleep(0.5)    
        if self.hark_process.poll() == 0:    
            print "[SpeechRecognizer] Hark crashed, check 'arecord -l' for an kinect usb on plughw1:0"
            sys.exit()
        conn, addr = s.accept()
        print '[SpeechRecognizer] Connected with Hark @ ' + addr[0] + ':' + str(addr[1])

        while self.running:
            
            #receive header (a)    
            data = conn.recv(header.size)
            header_data = header.unpack(data)
            
            #receive number of sound sources (g)        
            data = conn.recv(numsrc.size)
            numsrc_data = numsrc.unpack(data)
            
            #reset flags in database
            for dat_0 in raw:    
                dat_0['active'] = False
            
            if numsrc_data[0]>0:          
                        
                #for each simultaneous sound source
                for x in range(numsrc_data[0]):

                    #receive source info (h)        
                    data = conn.recv(srcinfo.size)
                    srcinfo_data = srcinfo.unpack(data)

                    #store the ID, set active, and store azimuth           
                    raw[srcinfo_data[0]%simsrc]['ID'] = srcinfo_data[0]
                    raw[srcinfo_data[0]%simsrc]['active'] = True
                    raw[srcinfo_data[0]%simsrc]['azimuth'] = 180/math.pi*math.atan2(srcinfo_data[2],srcinfo_data[1])

                    #receive source data (i)
                    data = conn.recv(srcdata.size)
                    srcdata_data = srcdata.unpack(data)

                    #receive raw audio (j)
                    raw[srcinfo_data[0]%simsrc]['raw'] += conn.recv(srcdata_data[1])
            
            if self.pause:
                #if we dont want the robot to listen to itself, set the boolean pause on True
                raw = [{'active': False, 'raw': '', 'ID': -1, 'azimuth': 0.0} for x in range(simsrc)]            
            
            # now the datapackage is read, check for finished transmissions.
            for dat in raw:
                # if the sound source has been closed and has information, the sound gets processed    
                if (not dat['active']) and (dat['ID'] >= 0):
                    # print "\ndecoding", dat['ID'], len(dat['raw'])
                    self.decoder.start_utt()            
                    self.decoder.process_raw(dat['raw'],False,True)
                    self.decoder.end_utt()
                    if self.decoder.hyp() == None:
                        pass
                    else:
                        result = self.decoder.hyp().hypstr
                        print "[SpRec-kin]",result
                        self.obs['result'] = result
                        self.obs['recogtime'] = rospy.Time.now()
                        self.obs['azimuth'] = dat['azimuth']
                        self.obs['2best'] = self.get2best(self.decoder)

                    # empty the buffer
                    raw[dat['ID']%simsrc]['raw'] = ""
                    raw[dat['ID']%simsrc]['ID'] = -1
                    raw[dat['ID']%simsrc]['azimuth'] = 0.0
                        
        print "PocketSpinx terminated"
        self.decoder.end_utt()
        server_socket.close()

    def listen_wav(self, HOST = "localhost", PORT = 5530):

        print "[SpeechRecognizer] Now accepting hark input"
        # this requires you to still run hark to split and send the wav to this decoder

        simsrc = 3
        debug = False
        raw = [{'active': False, 'raw': '', 'ID': -1, 'azimuth': 0.0} for x in range(simsrc)]

        # define decoding structures
        header = struct.Struct('3I 4I')
        srcinfo = struct.Struct('I 4f')
        numsrc = struct.Struct('I')
        srcdata = struct.Struct('2I')
        
        # List to keep track of socket descriptors
        CONNECTION_LIST = []
        RECV_BUFFER = 1024 # Advisable to keep it as an exponent of 2
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen(10)
        CONNECTION_LIST.append(server_socket)

        while self.running:

            read_sockets,write_sockets,error_sockets = select.select(CONNECTION_LIST,[],[])

            for conn in read_sockets:
                #New connection
                if conn == server_socket:
                    # Handle the case in which there is a new connection recieved through server_socket
                    sockfd, addr = server_socket.accept()
                    CONNECTION_LIST.append(sockfd)
                    print "[SpRec-wav] Hark client (%s, %s) connected" % addr
                                  
                #Some incoming message from a client
                else:
                    # Data recieved from client, process it
       
                    #receive header (a) [1]    
                    data = conn.recv(header.size)
                    try:
                        header_data = header.unpack(data)
                        #print header_data
                    except:
                        print "[SpRec-wav] Client (%s, %s) is offline" % addr
                        conn.close()
                        CONNECTION_LIST.remove(conn)
                        continue
                        break
                    
                    #receive number of sound sources (g) [1]       
                    data = conn.recv(numsrc.size)
                    numsrc_data = numsrc.unpack(data)
                    
                    #print "numsrc", numsrc_data
                    #reset flags in database
                    for dat_0 in raw:    
                        dat_0['active'] = False
                        # if a source ID is active again, this flag is set True below
                        # if not, it can be decoded
                    if numsrc_data[0]>0:          
                                
                        #for each sound source
                        for x in range(numsrc_data[0]):

                            #receive source info (h) [1]        
                            data = conn.recv(srcinfo.size)
                            srcinfo_data = srcinfo.unpack(data)
                            # print "src info", srcinfo_data

                            #store the ID, set active, and store azimuth           
                            raw[srcinfo_data[0]%simsrc]['ID'] = srcinfo_data[0]
                            raw[srcinfo_data[0]%simsrc]['active'] = True
                            raw[srcinfo_data[0]%simsrc]['azimuth'] = 180/math.pi*math.atan2(srcinfo_data[2],srcinfo_data[1])

                            #receive source data (i) [1]
                            data = conn.recv(srcdata.size)
                            srcdata_data = srcdata.unpack(data)

                            #receive raw audio (j) [1]
                            raw[srcinfo_data[0]%simsrc]['raw'] += conn.recv(srcdata_data[1])
                    
                    # now the datapackage is read, check for finished transmissions.
                    for dat in raw:
                        # if there's an unactive buffer with a valid ID, it's redy for decoding    
                        if (not dat['active']) & (dat['ID'] >= 0):
                            print "\ndecoding source", dat['ID']
            
                            self.decoder.start_utt()            
                            self.decoder.process_raw(dat['raw'],False,True)
                            self.decoder.end_utt()
                            if self.decoder.hyp() == None:
                                pass
                            else:
                                result = self.decoder.hyp().hypstr
                                print "[SpRec-wav]",result
                                self.obs['result'] = result
                                self.obs['recogtime'] = rospy.Time.now()
                                self.obs['azimuth'] = dat['azimuth']
                                self.obs['2best'] = self.get2best(self.decoder)

                                
                            # empty the buffer
                            raw[dat['ID']%simsrc]['raw'] = ""
                            raw[dat['ID']%simsrc]['ID'] = -1
                            raw[dat['ID']%simsrc]['azimuth'] = 0.0


        print "PocketSpinx terminated"
        self.decoder.end_utt()
        server_socket.close()

    def listen_mic(self):
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
        stream.start_stream()
        print "[SpeechRos] Microphone stream opened: listening..."    
        in_speech_bf = True
        self.decoder.start_utt('')

        while self.running:
            buf = stream.read(1024)
            if buf:
                self.decoder.process_raw(buf, False, False)
                if self.decoder.get_in_speech() != in_speech_bf:
                    in_speech_bf = self.decoder.get_in_speech()
                    if not in_speech_bf:
                        self.decoder.end_utt()
                        if self.decoder.hyp() == None:
                            pass
                        else:
                            if self.decoder.hyp().hypstr != '':
                                result = self.decoder.hyp().hypstr
                                print "[SpRec-mic]",result
                                self.obs['result'] = result
                                self.obs['recogtime'] = rospy.Time.now()
                                self.obs['2best'] = self.get2best(self.decoder)

class  SpeechRos(object):

    def __init__(self, args=None):

        self.m_write            = rospy.ServiceProxy('memory', MemorySrv)
        self.m_read             = rospy.ServiceProxy('memory_read', MemoryReadSrv)
        self.last_obs           = rospy.Time.now()
        self.last_question_time = rospy.Time.now()
        self.possible_answers   = None
        self.new_obs            = {'result' : None, 'azimuth': None ,'recogtime': None}
        
        self.recognizer         = Speechrecognizer(args)
        self.asking             = False
        self.pause              = False
        self.rempause           = False

        if not args == None:
            self.grammar = args['gram']
            self.recognizer.set_gram('speech/hark-sphinx/grammar/'+args['gram'])
        self.recognizer.daemon = True
        self.recognizer.start()

    def run(self):
        try:
            while self.recognizer.isAlive():
                self.check_talking()

                if self.need_to_ask_question(): 
                    print "waiting for these posible answers", self.possible_answers
                    self.set_responses("question",self.possible_answers)

                self.new_obs = self.recognizer.get_obs()

                if (not self.new_obs['result'] == None) and (not self.new_obs['recogtime'] == self.last_obs):
                    self.last_obs = self.new_obs['recogtime']
                    if not self.new_obs['2best'] == None:
                        print "[SpeechRos] understood these options", self.new_obs['2best']
                    else:
                        print "[SpeechRos] understood one option", self.new_obs['result']
                    self.last_obs = self.new_obs['recogtime']
                    if self.pause:
                        print "[SpeechRos] utterance IGNORED due to self speaking"
                    else:
                        self.send_to_memory({'message' : self.new_obs['result'], 'azimuth' : self.new_obs['azimuth'], '2best' : self.new_obs['2best']})
                        #if you were waiting for an answer, and got it, restore previous grammar
                        if self.asking:
                            print "[SpeechRos] returning to previous grammar"
                            self.asking = False
                            self.recognizer.set_gram('speech/hark-sphinx/grammar/' + self.grammar)

        except KeyboardInterrupt:
            print "KeyboardInterrupt signal received \nSpeechRos Killed"
            self.recognizer.running = False
            self.recognizer.hark_process.kill()

    def set_responses(self, question = None, cmds = None):
        
        if not cmds: 
            print "[SpeechRos] no possible response given"
            return
        with open('closed.gram', 'w') as f:
            f.write('#JSGF V1.0;\n\ngrammar vcc;\n\npublic <all> = <response>;\n\n')
            f.write('<response> = (')
            buf = ''
            for command in cmds:
                buf += ' ' + command + ' |'
            f.write(buf[:-1] + ');')
        self.recognizer.set_gram('closed.gram')


    def send_to_memory(self,send_obs):
        
        try:
            self.m_write(rospy.Time.now(), "voice_command", json.dumps(send_obs))
        except rospy.service.ServiceException:
            print "could not send to memory, is behavior running?"

    def need_to_ask_question(self):
        
        try:
            result = self.m_read('get_last_observation',rospy.Time.now(),'ask_question','')
            result = json.loads(result.json)
        except rospy.service.ServiceException:
            result = None
        if result and (result['time'] > self.last_question_time):
            self.last_question_time = rospy.Time.now()
            self.asking = True
            self.possible_answers = result['answers']
            return True
        return False

    def check_talking(self):

        tmp = self.pause
        self.pause = False
        try: 
            result = self.m_read('get_last_observation',rospy.Time.now(),'body_say','')
            result = json.loads(result.json)
        except rospy.service.ServiceException:
            result = None            
        if result:
            string_length = len(result['text'])
            duration = result['duration']
            if (rospy.Time(result['time']) + rospy.Duration(0.5) + rospy.Duration(float(duration))) > rospy.Time.now():
                self.pause = True
        if self.rempause: self.pause = True
        if tmp == False and self.pause == True:
            print "[SpeechRos] pausing the speechcontroller"
        if tmp == True and self.pause == False:
            print "[SpeechRos] resuming the speechcontroller"

    def callback(self,data):

        print data.data
        if data.data == "start" and self.rempause == False:
            self.rempause = True
            print "Recognizer paused for remote speaking"
        if data.data == "done" and self.rempause == True:
            self.rempause = False
            self.pause = False
            print "Recognizer continued after remote speaking"
                
if __name__ == "__main__":
    
    # disable_signals on False catches Exceptions, effectively disabling CRTL-C
    rospy.init_node('ros_speech_input',disable_signals=True)

    parser = argparse.ArgumentParser(description='Separate SpeechRecognizer')
    parser.add_argument("--source", default = "kinect", choices = ["mic","kinect","wav"])
    parser.add_argument("--gram")
    args = parser.parse_args()
    harksphinx = SpeechRos(vars(args))
    harksphinx.run()

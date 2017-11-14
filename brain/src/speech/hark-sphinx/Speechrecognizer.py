import socket
import sys,re
import struct
import math
import argparse
import pyaudio
import subprocess
import ossaudiodev
import time
import threading
from pocketsphinx import *
from sphinxbase import *

from collections import OrderedDict
#kin = subprocess.call(['pacmd','set-source-volume','4','100000'])

class Speechrecognizer(threading.Thread):
    def __init__(self, args=None):
        threading.Thread.__init__(self)
        self.obs = {'result' : None, 'azimuth': None ,'recogtime': None,'2best' : None}
        self.running = True
        self.args = args
        self.pause = False
        
        # setup PocketSphinx
        hmm= 'speech/hark-sphinx/model/en-us'
        lm = 'speech/hark-sphinx/model/en-us.lm'
        dic = 'speech/hark-sphinx/model/cmu07a.dic'

        config = Decoder.default_config()
        config.set_string('-logfn', '/dev/null') #comment for debug information
        config.set_string('-hmm', hmm)
        config.set_string('-lm', lm)
        config.set_string('-dict', dic)
        #config.set_string('-rawlogdir', 'logs') #uncomment to log the raw stream data
        print "[SpeechRecognizer] loading language model ..."
        self.decoder = Decoder(config)
        print "[SpeechRecognizer] done."
        
        self.grammarcontent = None      # the whole grammarfile for regex
        self.grammaropts = []         # optional and therefore disregarded words

    def run(self):
        if self.args == None:
            self.listen_mic()
        else:
            if self.args['source'] == "kinect":
                self.listen_kinect()
            if self.args['source'] == "mic":
                self.listen_mic()

    def get_optional_words(self):
        p = re.compile(ur'(?<=\[)[a-z< >]+(?=\])')
        return re.findall(p, self.grammarcontent)


    def filter_optional_words(self, phrase):
        for optional in self.grammaropts:
            phrase = phrase.replace(" %s " % optional, " ")

        return phrase

    def set_gram(self, gramfile):
        jsgf = Jsgf(gramfile)
        rule = jsgf.get_rule('<vcc.all>')
        fsg = jsgf.build_fsg(rule, self.decoder.get_logmath(), 7.5)
        #fsg.writefile('grammar/vcc.fsg')
        self.decoder.set_fsg("newgram", fsg)
        self.decoder.set_search("newgram")
        with open(gramfile, "r+") as grammarFile:
            self.grammarcontent = grammarFile.read()
        self.grammaropts = self.get_optional_words()
        print self.grammaropts
        print "[SpeechRecognizer] switched to grammar",gramfile

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

    def listen_kinect(self, HOST = "localhost", PORT = 5530):
        
        time.sleep(5) 
        #obvious paramss
        simsrc = 3
        debug = False
        raw = [{'active': False, 'raw': '', 'ID': -1, 'azimuth': 0.0} for x in range(simsrc)]

        # define decoding structures
        header = struct.Struct('3I 2q')
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

        hark_process = subprocess.Popen(['xterm', '-e',"./speech/hark-sphinx/hark-config/kinect_network.n"],stderr=subprocess.STDOUT)    
        time.sleep(0.5)    
        if hark_process.poll() == 0:    
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
                    if debug: print "\ndecoding", dat['ID'], len(dat['raw'])
                    try:            
                        self.decoder.start_utt('Hark')            
                        self.decoder.process_raw(dat['raw'],False,True)
                        self.decoder.end_utt()
                        result = self.decoder.hyp().hypstr

                        self.obs['2best'] = None

                        if False: #debug printings
                            print('\nAzimuth: %.2f (deg)' % dat['azimuth']),
                            if dat['azimuth'] > 0: print '(left)' 
                            else: print '(right)'
                            print '[SpeechRecognizer] Source',dat['ID'],'Result:', result

                        try:
                            besttwo = self.get2best(self.decoder)
                            # print besttwo
                            self.obs['2best'] = besttwo

                        except Exception as e:
                            print 'get2best failed',e

                        self.obs['result'] = result
                        self.obs['recogtime'] = time.time()
                        self.obs['azimuth'] = dat['azimuth']
                        

                    except AttributeError as e:
                        pass
                    except:
                        print "[SpeechRecognizer] I couldn't decode source", dat['ID']
                    
                    # empties the local buffer
                    raw[dat['ID']%simsrc]['raw'] = "" 
                    if debug: print 'after reset tawlen', len(dat['raw'])
                    raw[dat['ID']%simsrc]['ID'] = -1
                    raw[dat['ID']%simsrc]['azimuth'] = 0.0
        self.decoder.end_utt()
        print("PocketSphinx terminated")

    def listen_mic(self):
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
        stream.start_stream()
        print "[SpeechRecognizer] Microphone stream opened: listening..."    
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
                        try:
                            if  self.decoder.hyp().hypstr != '':
                                utt = self.decoder.hyp().hypstr
                                #print '[SpeechRecognizer] Result:', utt
                                self.obs['result'] = utt
                                self.obs['recogtime'] = time.time()
                                self.obs['2best'] = [utt,utt]
                        except AttributeError:
                            pass
                        self.decoder.start_utt('')
            else:
                break
        self.decoder.end_utt()
        print("PocketSphinx terminated")
        stream.stop_stream()
        stream.close()
        p.terminate()
    
    def get_obs(self):
        return self.obs

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Borg Speech Recognition')
    parser.add_argument("--source", default = "mic", choices = ["mic","kinect"])
    parser.add_argument("--gram")
    parser.add_argument("--wav", default = None)
    args = parser.parse_args()

    recognizer = Speechrecognizer(vars(args))
    if not args.gram == None:
        recognizer.set_gram(args.gram)
    recognizer.start()

    last_obs = time.time()
    new_obs = {'result' : None, 'azimuth': None ,'recogtime': None}

    while recognizer.isAlive():
        try: 
            new_obs = recognizer.get_obs()
            if not new_obs['result'] == None:
                if not new_obs['recogtime'] == last_obs:
                    print "[SpeechRecognizer]", new_obs['result']
                    last_obs = new_obs['recogtime']
        except KeyboardInterrupt:
            print "\nSending kill to PocketSphinx"
            recognizer.running = False
        except:
            print "\something went wrong: stopping"
            recognizer.running = False
            break
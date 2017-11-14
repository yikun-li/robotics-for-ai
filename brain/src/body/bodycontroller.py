#TODO: test and create unit test

import nao
import fake_nao
import pioneer
import alice
import fake_pioneer
import movebase
import brain
import sys,re
import time
import logging
import util.nullhandler
import memory
import os
import os.path
import math
import arduino
import serial
import subprocess
import wave
import contextlib
import rospy
import tf

IVONA = False
try:
    import urllib
    import httplib2
    from bs4 import BeautifulSoup
    import json
    import pygame
    import mad #sudo apt-get install python-pymad
except:
    print "[bodycontroller] Local Ivona voice not available"
    IVONA = False

ROS_ENABLED=False
try:
    import roslib; roslib.load_manifest('alice_msgs')
    import rospy
    from std_msgs.msg import String
    ROS_ENABLED = True
except:
    print "ROS IS NOT ENABLED"
    pass


logging.getLogger('Borg.Brain.BodyController').addHandler(util.nullhandler.NullHandler())

USE_MARYTTS=False

class BodyController(object):
    """
    Controls the robots, via specific nao and pioneer objects
    (Singleton)
    """

    TO_RAD = math.pi / 180.0 

    __instance = None
    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(BodyController, cls).__new__(cls, *args, **kwargs)
        return cls.__instance

    def __init__(self):
        self.logger = logging.getLogger('Borg.Brain.BodyController')
        self.memory = memory.Memory()
        self.emergency = False
        self.use_marytts = USE_MARYTTS
        self.use_ivona = IVONA
        self.remote_speech = False
        if self.remote_speech:
            self.ivona_pub = rospy.Publisher('/ivona_speech', String)
            print "[BodyController] using remote ivona speech."
        elif self.use_ivona:
            pygame.init()
        #self.__tf_listener = tf.TransformListener()

    def get_odom(self, corrected = True):
        """
        @param  corrected   Set to true to get the corrected odometry (if available).
        @return The current odometry as dictionary with keys: x, y, yaw (in radians).
                (The transform from /odom or /map (corrected) to /base_link.)
        @return None If it is not (yet) available.
        """
        target_frame = "map" if corrected else "odom"
        #TODO:XXX: Can't use '/' in frameExists; bug in API?
        if self.__tf_listener.frameExists(target_frame) and self.__tf_listener.frameExists("base_link"):
            try:
                now = rospy.Time()
                self.__tf_listener.waitForTransform("/%s" % target_frame, "/base_link", now, rospy.Duration(10.0))
                position, quaternion = self.__tf_listener.lookupTransform("/%s" % target_frame, "/base_link", now)
            except:
                return None
            return {'x':position[0], 'y':position[1], 'yaw':tf.transformations.euler_from_quaternion(quaternion)[2]}
        else:
            return None


    def set_espeak(self, enable = True):
        self.use_svox_speach = not enable

    def set_config(self, param_dict):
        '''this method sets the config, and is called in the brain
        this is _not_ done in the constructor, since the behaviors construct this
        class as well, and they dont have the param_dict'''

        global USE_MARYTTS
        mary = param_dict.get_option("body", "marytts", None)
        if mary is not None:
            mary = eval(mary)
            self.use_marytts = USE_MARYTTS = mary

        self.__nao_list = [] #list of the nao objects
        self.__pioneer_list = [] #list of the pioneer objects
        self.__alice_list = [] #list of the alice objects
        self.__movebase = None #the movebase objects
        self.__param_dict = param_dict

        self.arduino = None
        if self.__param_dict.get_option('body','arduino') == "True":
            self.arduino = arduino.Arduino()

        self.__create_robots()

        self.speak_cnt = 0
        self.use_svox_speach = True
        self.speech_commands = []
        self.prev_speech_command = {}
        self.remove_all_prev_speech_files()

    def get_config(self):
        return self.__param_dict

    def __del__(self):
        '''destroy the class and its data'''
        #TODO: maybe implement sometime?
        pass

    def __create_robots(self):
        '''creates the correct objects for all the robots'''

        number_of_naos = int(self.__param_dict.get_option('body','number_of_naos'))
        number_of_pioneers = int(self.__param_dict.get_option('body','number_of_pioneers'))
        #number_of_alices = int(self.__param_dict.get_option('body','number_of_alices'))
        number_of_alices = 0
        if number_of_pioneers > 1 or number_of_alices > 1:
            raise Exception("Currently, no more then 1 pioneer or alice is supported!")

        self.logger.debug("Connecting to %d NAO(s)" % number_of_naos)
        for i in range(number_of_naos):
            ip = self.__param_dict.get_option('body','nao_ip_%d' % i)
            port = self.__param_dict.get_option('body','nao_port_%d' % i)
            self.__nao_list.append(nao.Nao(ip, port))
            self.logger.debug("Connection to NAO %d running on %s:%s made" % (i, ip, port))

        self.logger.debug("Connecting to %d Pioneer(s)" % number_of_pioneers)
        for i in range(number_of_pioneers):
            ip = self.__param_dict.get_option('body','pioneer_ip_%d' % i)
            port = self.__param_dict.get_option('body','pioneer_port_%d' % i)
            str_start_pose = self.__param_dict.get_option('body', 'pioneer_pose_%d' % i)
            if str_start_pose:
                start_pose = str_start_pose.split()
            else:
                start_pose = False
            self.__pioneer_list.append(pioneer.Pioneer(ip, port, start_pose))
            self.logger.debug("Connection to Pioneer %d running on %s:%s made" % (i, ip, port))
            
        for i in range(number_of_alices):
            self.__alice_list.append(alice.Alice())
            self.logger.debug("Connection to Alice from behavior is initiated")

    def stop(self):
        for pioneer in self.__pioneer_list:
            pioneer.stop()
        for alice in self.__alice_list:
            alice.stop()
        for nao in self.__nao_list:
            nao.stop()
        self.stop_speaking(True)
        self.remove_all_prev_speech_files()

    def update(self):
        """
        This method is only adding the update for pioneer odometry and movebase to the memory
        """
        for pio in self.__pioneer_list:
            data = pio.update()
            if "EMERGENCY" in data:
                if not self.emergency:
                    self.emergency = True
                    self.logger.warn("Emergency button pressed on Pioneer. Setting NAO's in emergency mode")
                    for nao in self.__nao_list:
                        # Set in emergency button pressed mode
                        nao.emergency()
                        nao.emergencyLeds(True)
                    time.sleep(1)
            else:
                if self.emergency:
                    self.emergency = False
                    self.logger.warn("Emergency situation on Pioneer recovered. Re-enabling NAO's")
                    for nao in self.__nao_list:
                        nao.say("I will now resume normal operation")
                        #nao.set_stifness()
                self.add_to_memory(data)

        for ali in self.__alice_list:
            data = ali.update()
            if "EMERGENCY" in data:
                if not self.emergency:
                    self.emergency = True
                    self.logger.warn("Emergency button pressed on Alice. Stopping Everything")
                    time.sleep(1)
            else:
                if self.emergency:
                    self.emergency = False
                    self.logger.warn("Emergency situation on Alice recovered. Re-enabling Everything")
                    for nao in self.__nao_list:
                        nao.say("I will now resume normal operation")
                        #nao.set_stifness()
                self.add_to_memory(data)
            
        if self.__movebase:
            data = self.__movebase.update()
            self.add_to_memory(data)

        if self.arduino:
            self.arduino.update()

        self.update_speech()

    def update_speech(self):
        # Check if there are queued speech commands left
        if len(self.speech_commands) > 0:

            # Check if previous speech command is finished playing
            if len(self.prev_speech_command) == 0 or time.time() > (self.prev_speech_command['start_time'] + self.prev_speech_command['duration']):
                # Remove previous speech file
                if len(self.prev_speech_command) > 0 and not self.prev_speech_command['custom_file']:
                    subprocess.Popen("rm -f " + self.prev_speech_command['filepath'], shell=True)

                # Get first command to say
                self.prev_speech_command = self.speech_commands.pop(0)
                self.prev_speech_command['start_time'] = time.time()

                if not self.prev_speech_command['custom_file']:
                    print "####>>>>>" + self.prev_speech_command['message']

                # Play sound
                if not IVONA:
                    subprocess.Popen("aplay -q " + self.prev_speech_command['filepath'], shell=True)
                else:
                    subprocess.Popen("python $BORG/brain/src/speech/play_mp3.py \"" + self.prev_speech_command['filepath'] + "\"", shell=True)
        # If last speech file still exists after speaking, remove it
        if len(self.prev_speech_command) > 0 and not self.prev_speech_command['custom_file'] and \
           time.time() > (self.prev_speech_command['start_time'] + self.prev_speech_command['duration']):
            if not IVONA:
                subprocess.Popen("rm -f " + self.prev_speech_command['filepath'], shell=True)

    def get_marytts(self):
        if not self.use_marytts:
            return None

        if hasattr(self, "marytts"):
            return self.marytts

        from util.marytts import MaryTTS
        self.marytts = MaryTTS()
        cfg = self.get_config()
        self.marytts.set_voice(cfg.get_option("body", "tts_voice", "Prudence"))
        return self.marytts

    def remote_ivona(self,text):
        #Non-Blocking
        #publish to the ivona_tts topic
        print "Ivona says: ", text
        self.ivona_pub.publish(String(text))
    
    def tts(self, text):
        tts = self.get_marytts()
        if tts:
            tts.say(text)
        elif self.use_ivona:
            self.speak_cnt = self.speak_cnt + 1
            http = httplib2.Http()
            filepath = "/tmp/svox_speach_" + str(self.speak_cnt) + ".mp3"
            url = 'http://www.ivona.com/'
            response, content = http.request(url, 'GET')
            soup = BeautifulSoup(content)
            csrfield = soup.find(id="VoiceTesterForm_csrfield")['value']
            values = {  'ext' : 'wav','voiceSelector' : '27',   'text' : text,'send' : 'play','csrfield' : csrfield,'ref-form-name' : 'VoiceTesterForm' }
            data = urllib.urlencode(values)
            headers = { 'Cookie' : response['set-cookie'],'Content-type': 'application/x-www-form-urlencoded'}
            url = 'http://www.ivona.com/let-it-speak/?setLang=us'
            response, content = http.request(url, 'POST', headers=headers, body=data)
            json_data = json.loads(content)
            audio_url = re.search("(?P<url>https?://[^\s]+)", json_data['script']).group("url")[:-2]
            urllib.urlretrieve (audio_url, filepath)
            duration = mad.MadFile(filepath).total_time()/1000 + 1
            self.speech_commands.append({'message': text, 'filepath': filepath, 'duration': duration, 'custom_file': False, 'start_time': 0})
            self.add_to_memory([{'name':'body_say','time':time.time(),'property_dict':{'duration':duration,'text':text,'time':time.time()}}])
            self.update_speech()
            
        else:
            self.speak_cnt = self.speak_cnt + 1
            filepath = "/tmp/svox_speach_" + str(self.speak_cnt) + ".wav"

            if not self.use_svox_speach:
                subprocess.Popen("espeak -v en \"" + text.replace("'", "'\\''") + "\"" + " --stdout -w \"" + filepath + "\"", shell=True)
            else:
                # Create wav file
                subprocess.Popen("pico2wave -l=en-GB -w=" + filepath + " \"" + text + "\"", shell=True)

            # Wait until wav file is actually there
            while (not os.path.isfile(filepath)):
                time.sleep(0.1)

            # Calculate duration of speach
            duration = 6250.0
            while duration == 6250.0:
                try:
                    with contextlib.closing(wave.open(filepath,'r')) as f:
                        frames = f.getnframes()
                        rate = f.getframerate()
                        duration = frames / float(rate)
                except EOFError:
                    pass
                # Wait time for slow systems
                if duration == 6250.0:
                    time.sleep(0.1)
            print duration
            # Add speech command to list of speech commands
            self.speech_commands.append({'message': text, 'filepath': filepath, 'duration': duration, 'custom_file': False, 'start_time': 0})
            self.add_to_memory([{'name':'body_say','time':time.time(),'property_dict':{'duration':duration,'text':text,'time':time.time()}}])
            self.update_speech()

    def say(self, text):
        if self.remote_speech:
            self.remote_ivona(text)
            return
        if len(self.__nao_list) > 0:
            self.__nao_list[0].say(text)
        else:
            self.tts(text)

    def is_speaking(self):
        """ returns true if speaking """
        # Check if there are speech commands
        if len(self.prev_speech_command) == 0:
            return False

        # Check if still speaking
        if time.time() < (self.prev_speech_command['start_time'] + self.prev_speech_command['duration']):
            return True

        # Check if there are more command coming
        if len(self.speech_commands) > 0:
            return True
        return False

    def stop_speaking(self, force_stop = False):
        """ removes all queued speech commands """
        # Also kill current speech command
        if force_stop:
            subprocess.Popen("killall -9 aplay", shell=True)
            self.prev_speech_command = {}

        # Remove next speech commands in list
        for speech_command in self.speech_commands:
            if not speech_command['custom_file']:
                subprocess.Popen("rm -f " + speech_command['filepath'], shell=True)

        self.speech_commands = []

    def remove_all_prev_speech_files(self):
        """ removes any previous speech files on the file system """
        for filename in os.listdir('/tmp/'):
            if filename[:11] == "svox_speach":
                subprocess.Popen("rm -f " + '/tmp/' + filename, shell=True)

    def play_wav_file(self, filepath):
        """ plays a wav file """
        # Check if file exists
        if not os.path.isfile(filepath):
            print "Error: ", filepath, "could not be found"
            return
        
        # Calculate duration of speach
        duration = 6250.0
        while duration == 6250.0:
            try:
                with contextlib.closing(wave.open(filepath,'r')) as f:
                    frames = f.getnframes()
                    rate = f.getframerate()
                    duration = frames / float(rate)
            except EOFError:
                pass
            # Wait time for slow systems
            if duration == 6250.0:
                time.sleep(0.1)

        # Add wav file to list of speech commands
        self.speech_commands.append({'message': "", 'filepath': filepath, 'duration': duration, 'custom_file': True, 'start_time': 0})
        
        self.update_speech()

    def ask(self,question,cmds,asking=True):
        if asking: self.say(question)
        self.add_to_memory([{'name':'ask_question','time':time.time(),'property_dict':{'answers':cmds,'time':time.time()}}])

    def add_to_memory(self, objects):
        """add a list of dictionaries describing objects to memory database"""
        if objects:
            for object in objects:
                self.memory.add_item(object['name'], object['time'], object['property_dict'])

    def nao(self, index):
        '''return the object that represents the specified nao'''
        if index >= len(self.__nao_list):
            self.logger.error("Requested unavailable NAO %d. Returning fake NAO" % index)
            return fake_nao.FakeNao("localhost")
        return self.__nao_list[index]

    def pioneer(self, index):
        '''return the object that represents the specified pioneer'''
        if index >= len(self.__pioneer_list):
            self.logger.error("Requested unavailable Pioneer %d. Returning fake Pioneer" % index)
            return fake_pioneer.FakePioneer("localhost", 12345)
        return self.__pioneer_list[index]

    def alice(self):
        return self.__alice_list[0] if len(self.__alice_list) > 0 else None

    def movebase(self):
        '''return the movebase object, if not exist, create it'''
        if not self.__movebase:
            self.__movebase = movebase.MoveBase()
        return self.__movebase
        
    def is_movebase_running(self):
        return True if self.__movebase else False

    def get_nr_naos(self):
        return len(self.__nao_list)

    def get_nr_pioneers(self):
        return len(self.__pioneer_list)

    def turn_head(self, angle):
        if angle > 90:
            raise Exception("Angle cannot be bigger than 90 degrees.")
        if angle < -90:
            raise Exception("Angle cannot be lower than -90 degrees.")
        try:
            ser = serial.Serial("/dev/ttyACM0", 9600)
            ser.open()
            ser.write(chr(angle + 90))
            ser.close()
        except Exception as e:
            print e

    def look_at(self, x, y, distance):
        """
        Makes the robot look at the specified x and y position.
        Will use a Nao if it is available.
        x = The x position (in meters) relative from the center of the head (while looking straight).
        y = The y position (in meters) relative from the center of the head (while looking straight).
        distance = The distance (in meters) relative to the head (positive number).
        The x and y position of the head is (0, 0). 
        if x > 0 or y > 0 --> Head will look to the right or up, left or down otherwise.
        """
        if len(self.__nao_list) > 0:
            yaw = math.atan(abs(x) / distance)
            if x < 0:
                yaw *= -1
            pitch = math.atan(abs(y) / distance)
            if y < 0:
                pitch *= -1

            if yaw > 40 / self.TO_RAD:
                yaw = 40 / self.TO_RAD
            elif yaw < -40 / self.TO_RAD:
                yaw = -40 / self.TO_RAD
            if pitch > 25 / self.TO_RAD:
                pitch = 25 / self.TO_RAD
            elif pitch < -25 / self.TO_RAD:
                pitch = -25 / self.TO_RAD
            self.nao(0).set_angles(['HeadYaw', 'HeadPitch'], [yaw, pitch], 0.2, radians=True)
        else:
            self.logger.error("No hardware available to use for looking!")


if __name__ == "__main__":
    param_dict = brain.load_config(sys.argv)

    bc = BodyController()
    bc.set_config(param_dict)

    #specify command and speed:
    speed_left = 0
    speed_right = 0
    bc.pioneer(0).set_left_right_speeds(self, speed_left, speed_right)
    #then, update the bodycontroller as many times as possible (usally done
    for i in range(10): #drive for one second
        time.sleep(0.1)
        bc.pioneer(0).set_left_right_speeds(self, speed_left, speed_right)
    bc.pioneer(0).stop_robot()

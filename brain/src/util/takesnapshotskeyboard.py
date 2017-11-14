import os
import time
from naoqi import ALProxy
import naovideo
import takeimages
import math
import cv
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Util.TakeSnapshotsKeyboard').addHandler(util.nullhandler.NullHandler())

class TakeSnapshotsKeyboard():

    def __init__(self, video_source='webcam', robot_ip="localhost", nao_mode='around', file_path=os.environ['BORG'] + '/brain/data/models/'):
        ### PARAMETERS: ###
        self.logger = logging.getLogger('Borg.Brain.Util.TakeSnapshotsKeyboard')
        ROBOT_IP = robot_ip
        PORT = 9559
        self.file_path = file_path #Path to the folder where incoming images will be stored
        self.waitTime = 1.2 #Wait n seconds after moving the neck to stabilize image.
        self.headPitchUp = math.radians(-10) #Head pitch: UP (-10, -10)
        self.headPitchDown = math.radians(20) #Head pitch: DOWN (20, 10)
        self.headYawStart = math.radians(60)
        self.headYawChange = math.radians(30)
        self.headTurnSpeed = 0.4 #Proportion relative to maximum speed possible
        self.video_source = video_source
        self.nao_mode = nao_mode

        ### INITIALIZERS: ###
        if video_source == 'nao':
            self.NaoVideo = naovideo.VideoModule(ROBOT_IP)
            self.NaoVideo.change_camera(1)
            self.TTSproxy = ALProxy("ALTextToSpeech", ROBOT_IP, PORT)
            self.motion   = ALProxy("ALMotion", ROBOT_IP, PORT)
            self.TTSproxy.say('ooo')
        elif video_source == 'webcam':
            self.takeImages = takeimages.TakeImages(cameranumber=1)
            cv.NamedWindow("PHOTO")

    def make_snapshot(self):
        if self.video_source == 'nao':

            ##### MOVE HEAD UP-DOWN FROM LEFT TO RIGHT #####
            if self.nao_mode == 'around':
                headChange = self.headYawChange
                headPosition = self.headYawStart
                for step in range(5):
                    self.motion.setAngles('HeadYaw', headPosition, self.headTurnSpeed)

                    self.motion.setAngles('HeadPitch', self.headPitchUp, self.headTurnSpeed)
                    time.sleep(self.waitTime)
                    self.save_image_nao()
                    self.motion.setAngles('HeadPitch', self.headPitchDown, self.headTurnSpeed)
                    time.sleep(self.waitTime)
                    self.save_image_nao()

                    headPosition -= headChange

                self.motion.setAngles('HeadPitch', 0, self.headTurnSpeed)
                self.motion.setAngles('HeadYaw', 0, self.headTurnSpeed)
                time.sleep(6) #Time for moving robot to next location

            ##### MOVE HEAD UP-DOWN #####
            elif self.nao_mode == 'up_down':
                self.motion.setAngles('HeadYaw', 0, self.headTurnSpeed)

                self.motion.setAngles('HeadPitch', self.headPitchUp, self.headTurnSpeed)
                time.sleep(self.waitTime)
                self.save_image_nao()
                self.motion.setAngles('HeadPitch', self.headPitchDown, self.headTurnSpeed)
                time.sleep(self.waitTime)
                self.save_image_nao()

                time.sleep(2) #Time for moving robot to next location

            ##### DONT MOVE HEAD AND WAIT FOR KEY-PRESS #####
            elif self.nao_mode == 'static_and_key-press':
                lastImage = self.NaoVideo.get_image()
                cv.ShowImage('PHOTO', lastImage)
                pressed_button = cv.WaitKey(5)
                #Save image if pressed any key:
                if pressed_button != -1:
                    filename = self.file_path + 'CAM' + str(time.time()) + '.jpg'
                    cv.SaveImage(filename, lastImage)
                    self.TTSproxy.say("Photo")
                    print 'IMAGE SAVEDDDD!!!'

        ##### WAIT FOR KEY-PRESS #####
        elif self.video_source == 'webcam':
            lastImage = self.takeImages.get_image()
            cv.ShowImage('PHOTO', lastImage)
            pressed_button = cv.WaitKey(5)
            #Save image if pressed any key:
            if pressed_button != -1:
                filename = self.file_path + 'CAM' + str(time.time()) + '.jpg'
                cv.SaveImage(filename, lastImage)
                print 'IMAGE SAVEDDDD!!!'

    def save_image_nao(self):
        lastImage = self.NaoVideo.get_image()
        filename = self.file_path + 'NAO' + str(time.time()) + '.jpg'
        cv.SaveImage(filename, lastImage)
        self.TTSproxy.say("Photo")

if __name__ == '__main__':
    ##### PARAMETERS #####
    #robot_ip = "192.168.0.100" #NON STANDARD
    robot_ip = "localhost"
    #robot_ip = "10.0.0.78"
    #robot_ip = "129.125.178.232"
    #robot_ip = "129.125.178.233"
    #robot_ip = "129.125.178.234"
    video_source = 'webcam' # 'nao' or 'webcam'
    #Only used when video_source = 'nao'.
    #Possible parameters: 'around', 'up_down' or 'static_and_key-press'.
    nao_mode = 'static_and_key-press'
    file_path = os.environ['BORG'] + '/brain/data/models/eggy/'

    ##### INITIALIZERS #####
    TSK = TakeSnapshotsKeyboard(video_source, robot_ip, nao_mode, file_path)

    while True:
        TSK.make_snapshot()

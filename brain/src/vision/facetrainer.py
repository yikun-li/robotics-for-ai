import os
import sys
import time
import termios

from abstractvisionmodule import AbstractVisionModule
import configparse
import eigenfaces
import facedetect
import logging
import util.sendsocket
import util.vidmemreader
import util.ticker
import memory

TERMIOS = termios
def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 0
    new[6][TERMIOS.VTIME] = 1

    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c

logging.getLogger('Borg.Brain.Vision.FaceTrainer').addHandler(util.nullhandler.NullHandler())
class FaceTrainer(AbstractVisionModule):
    """Work in progress.
    FaceTrainer trains on images using eigenfaces.
    This can be done on a webcam feed or on a folder.

    Based on BORG's facerecognizer.
    Joost de Kleine & Josje van Lunteren
    """
    
    def __init__(self, host, port, num_faces, update_frequency, video_source="webcam", 
                 cascade = 'haarcascade_frontalface_default', verbose=False):
        self.logger = logging.getLogger('Borg.Brain.Vision.FaceTrainer')
        super(FaceTrainer, self).__init__(host, port)
        self.__ticker = util.ticker.Ticker(update_frequency)
        self.__cascade = cascade
        self.__verbose = verbose
        self.set_known_objects(['facename', 'facetr_num', 'waitkey'])
        self.__detect = facedetect.FaceDetector(video_source = video_source)
        self.__training = False
        self.__source = video_source
        self.__eigen = eigenfaces.EigenFaces(num_faces, 0)

        # Internal representation, needs conversion for use by eigenfaces 
        self.__faceList = {'names':[], 'pics':[]}
        self.__curIdx = None
        self.__reportCnt = 0
        self.__wait = False
        self.__run = True
        self.__print_time = 0
        if self.__verbose:
            print "Facetrainer started"
    
    def getTrainingstatus(self):
        return self.__training
    
    def getFaceList(self):
        return self.__faceList
    
    def getCurrentIdx(self):
        return self.__curIdx
    
    #Verplicht
    def train(self):
        pass
    
    def run(self):
        while True:
            self._AbstractVisionModule__send_heartbeat()
            if self.__run:
                self.__ticker.tick()
                if self.__wait:
                    self.__waitkey()
                if  self.__training:
                    curr_name = self.__faceList['names'][self.__curIdx]
                    if self.__source == 'folder':
                        if self.__detect.folder_exists(self.__training, curr_name):
                            if not self.__detect.finished():
                                self.__addImg(self.__training, curr_name, self.__source)
                                self.__reportToMemory()
                            if self.__detect.finished():
                                self.add_property('name','facename')
                                self.add_property(curr_name, 'finished')
                                self.store_observation()
                        else:
                            self.add_property('name','facename')
                            self.add_property(curr_name, 'finished')
                            self.store_observation()
                    elif self.__source == 'webcam':
                        self.__addImg(self.__training, curr_name, self.__source)
                        self.__reportToMemory()
            else:
                if time.time() - self.__print_time > 5:
                    self.__print_time = time.time()
                    if self.__verbose:
                        print 'Facetrainer halting'
            self.update()
    
    def handle_custom_commands(self, msg):
        if msg:
            self.__run = True
            #print 'Message is:', msg
    
        if msg['command'] == 'begin_training_face':
            if self.__verbose:
                print 'Received begin training message for', msg['params']['name']
            if self.__source == 'folder':
                self.__detect.reset()
            try:
                self.__beginTraining(msg['params']['name'], msg['params']['source'], msg['params']['params'])
            except:
                self.__beginTraining(msg['params']['name'])
        
        elif msg['command'] == 'end_training_face':
            if self.__verbose:
                print 'Received end training message for', self.__faceList['names'][self.__curIdx]
            self.__training = False
        
        elif msg['command'] == 'rename':
            if self.__verbose:
                print 'Received rename message'
            self.__rename(msg['params']['oldname'], msg['params']['newname'])
        
        elif msg['command'] == 'finalize_training':
            if self.__verbose:
                print 'Received finalize training message'
            self.__finalizeData()
            self.__training = False
            self.__run = False
        
        elif msg['command'] == 'reset':
            if self.__verbose:
                print 'Received reset message'
            self.__faceList = {'names':[], 'pics':[]}
            self.__eigen.setSaved(False)
            self.add_property('name', 'facename')
            self.add_property('data_saved', None)
            self.store_observation()
        
        elif msg['command'] == 'waitkey':
            print 'Press y to start training:'
            self.__wait = True
        
        elif msg['command'] == 'waitkey_reset':
            if self.__verbose:
                print 'Waitkey memory reset'
            self.add_property('name', 'waitkey')
            self.add_property('proceed',False)
            self.store_observation()
    
    def __reportToMemory(self):
        """Send current amount of captured image for training target.
        Since this method gets called from the run bit that loops once
        every second, the report is sent once every 2 seconds
        """
        self.__reportCnt += 1
        if not self.__reportCnt % 2:
            self.add_property('name', 'facetr_num')
            self.add_property('num', len(self.__faceList['pics'][self.__curIdx]))
            self.store_observation()
    
    def __beginTraining(self, name, source='webcam', params=None):
        self.__training = True
        self.__source = source
        if not params is None:
            self.__eigen.setNumFaces(params[0])
            self.__eigen.setThreshold(params[1])
        try:
            self.__faceList['names'].index(name)
        except ValueError:
            if self.__verbose:
                print name + ' does not yet appear in list of names'
            self.__faceList['names'].append(name)
            self.__faceList['pics'].append([])
        self.__curIdx = self.__faceList['names'].index(name)
    
    def __addImg(self, train, name, source):
        face = self.__detect.getFaces(train, name, source)
        if face is None:
            return
        elif not (len(face) == 1):
            return  # Too many faces: unsuitable data.
        if self.__verbose:
            print "Faces detected:", len(face)
        self.__faceList['pics'][self.__curIdx].append(face[0])
    
    def __rename(self, oldlabel, newlabel):
        if self.__verbose:
            print "Renaming '" + oldlabel + "' to '" + newlabel + "'"
        try:
            idx = self.__faceList['names'].index(oldlabel)
        except ValueError:
            if self.__verbose:
                print 'Name does not exist, not renaming'
            return
        self.__faceList['names'][idx] = newlabel
    
    def __finalizeData(self):
        print 'Starting finalizing data'
        imgList = []
        for name in self.__faceList['names']:
            self._AbstractVisionModule__send_heartbeat()
            for picture in self.__faceList['pics'][self.__faceList['names'].index(name)]:
                imgList.append((name, picture))
        self.add_property('name', 'facename')
        print 'Tick'
        self._AbstractVisionModule__send_heartbeat()
        
        if self.__eigen.train(imgList):
            print 'tick'
            # Make sure the eigenfaces directory exists and create it otherwise
            if not os.path.exists(os.environ['BORG'] + '/brain/data/eigenfaces/'):
                os.makedirs(os.environ['BORG'] + '/brain/data/eigenfaces/')
            self._AbstractVisionModule__send_heartbeat()
            print 'Tock'
            self.__eigen.save(os.environ['BORG'] + '/brain/data/eigenfaces/onlineTrainedData.dat')
            temp = time.time()
            # Give enough time to store the data: onlineTrainedData.dat often is more than 100MB
            print 'Tack'
            while not self.__eigen.getSaved() and time.time() < (temp + 3):
                self._AbstractVisionModule__send_heartbeat()
            self.add_property('training_successful', ['no','yes'][self.__eigen.getSaved()])
            if self.__verbose:
                print 'Eigenfaces training successful, data saved:', self.__eigen.getSaved()
        else:
            self.add_property('training_successful', 'no')
            if self.__verbose:
                print 'Eigenfaces training failed'
        self.store_observation()
    
    def __waitkey(self):
        """Wait for a keystroke before continuing."""
        s = getkey()
        if s:
            if s == 'y':
                self.add_property('name', 'waitkey')
                self.add_property('proceed',True)
                self.store_observation()
                self.__wait = False
        return

def usage():
    print "FaceTrainer"
    print "Gets a list of images, processes them with eigenfaces for training."
    print ""
    print "You should add the configuration options on the command line."
    print ""
    print "Usage: ", sys.argv[0], "host=<controller_host> port=<controller_port> update_freq=<update frequency> num_faces=<number of eigenfaces to use> [video_source=<video source>] [cascade=<haar cascade>] [verbose=<verboseness>]"

if __name__=="__main__":
    if len(sys.argv) < 4:
        print sys.argv
        usage()
        exit(1)

    sec = "facetrainer" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)
    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    num_faces = config_dict.get_option(sec, "num_faces")
    update_frequency = config_dict.get_option(sec, "update_freq")
    source = config_dict.get_option(sec, "video_source")
    use_cascade = config_dict.get_option(sec, "cascade", "haarcascade_frontalface_default")
    use_verbose = config_dict.get_option(sec, "verbose", "False")
    sect = config_dict.get_section(sec)
    print sect
    
    if not (update_frequency and num_faces and controller_ip and controller_port):
        usage()
        exit()
    
    logging.getLogger('Borg.Brain.Vision.FaceTrainer').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain.Vision.FaceTrainer').setLevel(logging.DEBUG)
    
    facetrain = FaceTrainer(controller_ip, controller_port, num_faces, update_frequency, source, use_cascade, use_verbose)
    facetrain.connect()
    facetrain.set_socket_verbose(False,False)
    if (facetrain.is_connected()):
        facetrain.run()

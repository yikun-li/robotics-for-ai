import cv
import eigenfaces
import facedetect
import logging
import os
import sys
import time

import util.sendsocket
import util.ticker
import util.vidmemreader

from util.takeimages import TakeImages
import configparse
from abstractvisionmodule import AbstractVisionModule

logging.getLogger('Borg.Brain.Vision.FaceRecognizer').addHandler(util.nullhandler.NullHandler())
class FaceRecognizer(AbstractVisionModule):
    """Work in progress.
    FaceRecognizer loads data produced by training and recognizes faces detected through facedetector.
    This can be done on a webcam feed or on a folder.

    Based on BORG's facerecognizer.
    Joost de Kleine & Josje van Lunteren
    """
    
    def __init__(self, host, port, update_frequency, num_faces, threshold, 
                 video_source="webcam", cascade = 'haarcascade_frontalface_default', verbose=False):
        self.logger = logging.getLogger('Borg.Brain.Vision.FaceRecognizer')
        super(FaceRecognizer, self).__init__(host, port)
        self.__ticker = util.ticker.Ticker(update_frequency)
        self.__eigen = eigenfaces.EigenFaces(num_faces, threshold)
        self.__detect = facedetect.FaceDetector(video_source = video_source)
        self.__detect.reset()
        self.__cascade = cascade
        self.__verbose = verbose
        self.__num_faces = num_faces
        self.__threshold = threshold
        self.__source = video_source
        self.set_known_objects(['facename'])
        
        self.__recog = False
        self.__names = []
        self.__name_idx = 0
        self.__scores = {}
        self.__detect_idx = 0
        self.__dir_lst = {}
        self.__curr_name = None
        self.__curr_dir = None
        self.__print_time = 0
        self.__prev_thr = threshold
        if self.__verbose:
            print "Facerecognizer started"
    
    def run(self):
        while True:
            self.__ticker.tick()
            self._AbstractVisionModule__send_heartbeat()
            if self.__recog:
                if self.__source == 'webcam':
                    faces = self.__detect.getFaces(False, None, self.__source)
                    self.__facesToNames(faces)
                elif self.__source == 'folder':
                    if not self.__detect.finished():
                        self.__detect_idx = self.__detect.getIndex()
                        faces = self.__detect.getFaces(False, self.__names[self.__name_idx], self.__source)
                        # Create a list with the length of the amount of faces in each picture under each picture in dir
                        self.__curr_name = self.__names[self.__name_idx]
                        self.__curr_dir = self.__dir_lst[self.__curr_name]
                        if self.__curr_dir != []:
                            self.__scores[self.__curr_name][self.__curr_dir[self.__detect_idx]] = range(len(faces))
                            self.__facesToNames(faces)
                        else:
                            self.__name_idx += 1
                            self.__detect.reset()
                    elif self.__detect.finished():
                        # Not all given names are checked: continue to the next name
                        if self.__name_idx < (len(self.__names) - 1):
                            self.__name_idx += 1
                            self.__detect.reset()
                        # All given names are checked: save the scores and reset
                        elif self.__name_idx >= (len(self.__names) - 1):
                            self.__finalize()
                    if self.__verbose:
                        print 'Current name:', self.__curr_name
            else:
                if time.time() - self.__print_time > 5:
                    self.__print_time = time.time()
                    if self.__verbose:
                        print 'Facerecognizer halting'
            self.update()
    
    def handle_custom_commands(self, msg):
        if msg['command'] == 'train':
            if self.__verbose:
                print 'Received training message'
            self.train(os.environ['BORG'] + '/brain/data/eigenfaces/' + msg['params']['file'])
        
        # When starting a new recognize run, empty the scores and get the correct directory listing.
        elif msg['command'] == 'recognize':
            if self.__verbose:
                print 'Received recognizing message'
            self.__recog = True
            if self.__source == 'folder':
                self.__names = msg['params']['names']
                for name in self.__names:
                    try:
                        self.__dir_lst[name] = self.__detect.getDir(False, name)[0]
                    except:
                        self.__dir_lst[name] = []
                    self.__scores[name] = {}
            elif self.__source == 'webcam':
                faces = self.__detect.getFaces(False, None, self.__source)
                self.__facesToNames(faces)
        
        # When loading a new file, set the correct threshold and number of eigenfaces.
        elif msg['command'] == 'load':
            if self.__verbose:
                print 'Received loading message'
            self.__prev_thr = self.__eigen.getThreshold()
            self.__numFaces = msg['params']['params'][0]
            self.__threshold = msg['params']['params'][1]
            self.train(os.environ['BORG'] + '/brain/data/eigenfaces/' + msg['params']['file'])
    
    def __facesToNames(self, faces):
        """Retrieve the names of the recognized faces and store them in memory."""
        if not faces == []:
            names = self.__recognize(faces)
            print 'names:', names
            self.__storeNames(names)
            if self.__verbose:
                print 'Trained: ' + str(self.__eigen.getTrained()) +\
                        '. Faces detected: ' + str(len(faces)) + ': ', str(names)
        else:
            if self.__verbose:
                print 'No faces detected'
    
    def __storeNames(self, names):
        """Store the names of recognized faces in memory."""
        for name in names:
            self.add_property('name', 'facename')
            self.add_property('face', (time.time(), name))
            self.store_observation()
    
    # Work in progress. 
    def __recognize(self, faces):
        filenames = []
        for face in faces:
            name = self.__eigen.recognize(face)
            if name:
                filenames.append(name)
                if self.__verbose:
                    cv_im = cv.CreateImageHeader(cv.GetSize(face), cv.IPL_DEPTH_8U, 1)
                    cv.SetData(cv_im, face.tostring())
                    cv.ShowImage(name, cv_im)
                    cv.WaitKey(10)
            # Log the scores for each observation
            if self.__source == 'folder':
                scores = self.__eigen.getScores(face)
                if scores:
                    self.__saveScores(self.__curr_name,\
                                        self.__curr_dir[self.__detect_idx],\
                                        faces.index(face),\
                                        scores[0], scores[1], scores[2], scores[3],\
                                        name)
            else:
                if self.__verbose:
                    print 'No recognition'
        return self.__parse(filenames)
    
    def __finalize(self):
        """Write the scores to files, store in memory that recognizing
        has been completed, and reset the facedetector."""
        #self.__writeScores()
        #self.__writeMachineScores()
        #self.__summarizeScores()
        self.add_property('name','facename')
        self.add_property('recognizing done', True)
        self.store_observation()
        self.__name_idx = 0
        self.__detect.reset()
        self.__recog = False
        if self.__verbose:
            print 'Scores saved and reset'
    
    def __parse(self, filenames):
        parsed = []
        for name in filenames:
            parsed.append(name.partition("_")[0])
        return parsed
    
    def train(self, file):
        if not self.__eigen.getTrained():
            if os.path.exists(file) or not file == '':
                self.__eigen.setNumFaces(self.__num_faces)
                self.__eigen.setThreshold(self.__threshold)
                self.__eigen.load(file)
            else:
                self.__eigen.train(os.environ['BORG'] + '/brain/data/eigenfaces/training')
        elif not file == '':
            self.__eigen.setNumFaces(self.__num_faces)
            self.__eigen.setThreshold(self.__threshold)
            self.__eigen.train(file)
    
    def getTrained(self):
        return self.__eigen.getTrained()
    
    def __saveScores(self, name, picture, face, predictions, mean_score,\
                        normalized_scores, max_abs_score, recognized_name):
        """Save all relevant scores of the given face in the self.__scores variable."""
        if self.__scores[name] != {}:
            self.__scores[name][picture][face] = {'predictions':predictions,\
                                                    'mean score': mean_score,\
                                                    'normalized scores': normalized_scores,\
                                                    'max abs score': max_abs_score}
            if recognized_name:
                self.__scores[name][picture][face]['recognized name'] = recognized_name
                if recognized_name == name:
                    self.__scores[name][picture][face]['correct'] = True
                else:
                    self.__scores[name][picture][face]['correct'] = False
            else:
                self.__scores[name][picture][face]['recognized name'] = 'NA'
                self.__scores[name][picture][face]['correct'] = False
            
            if [i[0] for i in predictions].count(name) == 0:
                self.__scores[name][picture][face]['stranger'] = True
            else:
                self.__scores[name][picture][face]['stranger'] = False
    
    def __writeScores(self):
        """Write all scores to a human readable (but long!) text file."""
        f = open(os.environ['BORG'] + '/brain/data/who_is_who/logs/scores' + '_eig' +\
                    str(self.__eigen.getNumFaces()) + '_thr' + str(self.__eigen.getThreshold()) + '.txt', 'w')
        for name in self.__names:
            f.write('\n' + name)
            if self.__scores[name] != {}:
                for picture in self.__scores[name]:
                    f.write('\n  picture: "' + picture + '",')
                    if len(self.__scores[name][picture]) > 0:
                        f.write(' number of faces: ' + str(len(self.__scores[name][picture])))
                        faces = self.__scores[name][picture]
                        
                        idx = 0
                        for face in faces:
                            f.write('\n     face: ' + str(idx) +\
                                    '\n        predictions: ' + str(face['predictions']) +\
                                    '\n        mean score: ' + str(face['mean score']) +\
                                    '\n        normalized scores: ' + str(face['normalized scores']) +\
                                    '\n        max abs score: ' + str(face['max abs score']) +\
                                    '\n        recognized name: ' + str(face['recognized name']) +\
                                    '\n        correct: ' + str(face['correct']) +\
                                    '\n        stranger: ' + str(face['stranger']) +\
                                    '\n        --------------------------------------------')
                            idx += 1
                        f.write('\n     --------------------------------------------')
                    else:
                        f.write('no faces in picture \n')
                f.write('\n--------------------------------------------')
            else:
                f.write('\n  picture: NA' +\
                        '\n        --------------------------------------------' +\
                        '\n     --------------------------------------------' +\
                        '\n--------------------------------------------')
    
    def __writeMachineScores(self):
        """Write all scores to a comma separated value file.
        Create a new file for each value of self.__eigen.getNumFaces() and self.__eigen.getThreshold().
        """
        f = open(os.environ['BORG'] + '/brain/data/who_is_who/logs/machineScores' + '_eig' +\
                    str(self.__eigen.getNumFaces()) + '_thr' + str(self.__eigen.getThreshold()) + '.csv', 'w')
        f.write('Name,Picture,Face number\n') # These are the heads
        for name in self.__names:
            if self.__scores[name] != {}:
                for picture in self.__scores[name]:
                    if len(self.__scores[name][picture]) > 0:
                        f.write(name + ',' + picture + ',,number of faces,' +\
                                    str(len(self.__scores[name][picture])) + '\n')
                        faces = self.__scores[name][picture]
                        
                        idx = 0
                        for face in faces:
                            f.write(name + ',' + picture + ',' + str(idx) + ',predictions')
                            for person in face['predictions']:
                                f.write(',' + person[0] + ',' + str(person[1]) + '\n' +\
                                        name + ',' + picture + ',' + str(idx) + ',')
                            f.write('mean score,' + str(face['mean score']) + '\n' +\
                                    name + ',' + picture + ',' + str(idx) + ',normalized scores')
                            
                            for i in range(len(face['normalized scores'])):
                                f.write(',' + face['predictions'][i][0] + ',' + str(face['normalized scores'][i]) + '\n' +\
                                        name + ',' + picture + ',' + str(idx) + ',')
                                
                            f.write('max abs score,' + str(face['max abs score']) + '\n' +\
                                    name + ',' + picture + ',' + str(idx) +\
                                    ',recognized name,' + str(face['recognized name']) + '\n' +\
                                    name + ',' + picture + ',' + str(idx) +\
                                    ',correct,' + str(face['correct']) + '\n' +\
                                    name + ',' + picture + ',' + str(idx) +\
                                    ',stranger,' + str(face['stranger']) + '\n')
                            idx += 1
                    else:
                        f.write(name + ',' + picture + ',' + '-1,' +\
                                'number of faces,0\n')
                    f.write('\n')
            else:
                f.write(name + ',NA\n\n')
                
            f.write('----------------------------------------------------------------\n')
    
    def __summarizeScores(self):
        """Write a performance summary to a csv file.
        This summarizes the correct recognitions, number of false positives and
        failures to recognize of all people trained on.
        """
        # If no summary.csv file exists, create a new one with all necessary heads.
        try:
            f = open(os.environ['BORG'] + '/brain/data/who_is_who/logs/summary.csv', 'r+')
            heads = f.readline()
        except:
            heads = None
        if not heads == "Eigenfaces, Threshold, , " +\
                        "Total faces, Total strangers, Correctly recognized, False positives, Of which strangers, Not recognized," +\
                        "Perc. correct, Perc. false pos., Perc. strangers recognized, Perc. not recognized, Perc. of strangers recognized," +\
                        "1 face photos only:, " +\
                        "Total faces, Total strangers, Correctly recognized, False positives, Of which strangers, Not recognized," +\
                        "Perc. correct, Perc. false pos., Perc. strangers recognized, Perc. not recognized, Perc. of strangers recognized\n":
            f = open(os.environ['BORG'] + '/brain/data/who_is_who/logs/summary.csv', 'w')
            f.write("Eigenfaces, Threshold, , " +\
                    "Total faces, Total strangers, Correctly recognized, False positives, Of which strangers, Not recognized," +\
                    "Perc. correct, Perc. false pos., Perc. strangers recognized, Perc. not recognized, Perc. of strangers recognized," +\
                    "1 face photos only:, " +\
                    "Total faces, Total strangers, Correctly recognized, False positives, Of which strangers, Not recognized," +\
                    "Perc. correct, Perc. false pos., Perc. strangers recognized, Perc. not recognized, Perc. of strangers recognized\n")
            f.close()
        
        face_total = 0 # Total detected faces
        face_total_one = 0 # Total detected faces in pictures with only 1 face
        corr_all = 0 # Total correctly recognized faces
        corr_one = 0 # Total correctly recognized faces in pictures with only 1 face
        false_pos = 0  # Total falsely recognized faces
        false_pos_one = 0 # Total falsely recognized faces in pictures with only 1 face
        strangers_total = 0 # Total strangers
        strangers_total_one = 0 # Total strangers in pictures with only 1 face
        strangers = 0 # Total strangers recognized
        strangers_one = 0 # Total strangers recognized in pictures with only 1 face
        not_recognized = 0 # Total detected faces not recognized at all
        not_recognized_one = 0 # Total detected faces not recognized at all in pictures with only 1 face
        
        for name in self.__names:
            if self.__scores[name] != {}:
                for picture in self.__scores[name]:
                    if len(self.__scores[name][picture]) > 0:
                        faces = self.__scores[name][picture]
                        face_total += len(faces)
                        if len(faces) == 1:
                            face_total_one += len(faces)
                        
                        for face in faces:
                            if face['stranger']:
                                strangers_total += 1
                                if len(faces) == 1:
                                    strangers_total_one += 1
                            if face['recognized name'] == 'NA':
                                not_recognized += 1
                                if len(faces) == 1:
                                    not_recognized_one += 1
                            else:
                                if face['correct']:
                                    corr_all += 1
                                    if len(faces) == 1:
                                        corr_one += 1
                                elif not face['correct']:
                                    false_pos += 1
                                    if face['stranger']:
                                        strangers += 1
                                    if len(faces) == 1:
                                        false_pos_one += 1
                                        if face['stranger']:
                                            strangers_one += 1
        
        # Append the scores to the existing summary.csv file.
        f = open(os.environ['BORG'] + '/brain/data/who_is_who/logs/summary.csv', 'a')
        if not self.__eigen.getThreshold() == self.__prev_thr:
            f.write('\n')
        
        if face_total == 0 or face_total_one == 0:
            f.write(str(self.__eigen.getNumFaces()) + ',' + str(self.__eigen.getThreshold()) + ',,' +\
                    str(face_total) + ',' + str(strangers_total) + ',' + str(corr_all) + ',' +\
                    str(false_pos) + ',' + str(strangers) + ',' + str(not_recognized) +\
                    ',NA,NA,NA,NA,NA,,' +\
                    str(face_total_one) + ',' + str(strangers_total_one) + ',' + str(corr_one) + ',' +\
                    str(false_pos_one) + ',' + str(strangers_one) + ',' + str(not_recognized_one) +\
                    ',NA,NA,NA,NA,NA\n')
        else:
            f.write(str(self.__eigen.getNumFaces()) + ',' + str(self.__eigen.getThreshold()) + ',,' +\
                    str(face_total) + ',' + str(strangers_total) + ',' + str(corr_all) + ',' +\
                    str(false_pos) + ',' + str(strangers) + ',' + str(not_recognized) + ',' +\
                    str(float(corr_all) / face_total * 100) + ',' +\
                    str(float(false_pos) / face_total * 100) + ',' +\
                    str(float(strangers) / face_total * 100) + ',' +\
                    str(float(not_recognized) / face_total * 100) + ',' +\
                    str(float(strangers) / strangers_total * 100) + ',,' +\
                    str(face_total_one) + ',' + str(strangers_total_one) + ',' + str(corr_one) + ',' +\
                    str(false_pos_one) + ',' + str(strangers_one) + ',' + str(not_recognized_one) + ',' +\
                    str(float(corr_one) / face_total_one * 100) + ',' +\
                    str(float(false_pos_one) / face_total_one * 100) + ',' +\
                    str(float(strangers_one) / face_total_one * 100) + ',' +\
                    str(float(not_recognized_one) / face_total_one * 100) + ',' +\
                    str(float(strangers_one) / strangers_total_one * 100) + '\n')

def usage():
    print "FaceRecognizer"
    print "Recognizes faces and sends names to the communicator."
    print ""
    print "You should add the configuration options on the command line."
    print ""
    print "Usage: ", sys.argv[0], "host=<controller_host> port=<controller_port> update_freq=<update_freq> [video_source=<video source>] [cascade=<haar cascade>] num_faces=<eigenfaces_number> threshold=<threshold> [verbose=<verboseness>]"

if __name__ == "__main__":
    if len(sys.argv) < 5:
        print sys.argv
        usage()
        exit(1)

    sec = "facerecognizer" #section in config_dict
    args = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(args, sec)
    controller_ip = config_dict.get_option(sec, "host")
    controller_port = config_dict.get_option(sec, "port")
    update_frequency = config_dict.get_option(sec, "update_freq")
    source = config_dict.get_option(sec, "video_source")
    use_cascade = config_dict.get_option(sec, "cascade", "haarcascade_frontalface_default")
    num_faces = config_dict.get_option(sec, "num_faces")
    threshold = config_dict.get_option(sec, "threshold")
    use_verbose = config_dict.get_option(sec, "verbose", "False")
    sect = config_dict.get_section(sec)
    print sect
    
    if not (update_frequency and num_faces and threshold and controller_ip and controller_port):
        usage()
        exit()
        
    logging.getLogger('Borg.Brain.Vision.FaceRecognizer').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain.Vision.FaceRecognizer').setLevel(logging.DEBUG)
    
    facerec = FaceRecognizer(controller_ip, controller_port, update_frequency, num_faces, threshold, source, use_cascade, use_verbose)
    facerec.connect()
    facerec.set_socket_verbose(False,False)
    if (facerec.is_connected()):
        facerec.run()

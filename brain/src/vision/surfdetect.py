"""example stand-alone vision module using SURF features"""

#imports
import sys
import cv
import time
#import util.naovideo
import util.imgsource
import util.testsocket
import util.sendsocket
import random
import pyflann
import numpy
import numpy.linalg as linalg
import math
import glob
import configparse
import os
import util.vidmemreader
import logging
import util.nullhandler
import cPickle

logging.getLogger('Borg.Brain.Vision.SurfDetect').addHandler(util.nullhandler.NullHandler())

class SurfDetect(object):
    """
    Detects objects that look like the given example image in SURF space.
    Constructor parameters:
        float: update_frequency (frequency to update images),
        util.sendsocket.SendSocket: send_socket (socket to return data),
        string: video_source_name (video source nao or webcam),
        string: distance_type (distance measure used in nearest neigbor classification,
                    possible values are: euclidean, manhattan, minkowski, max_dist, hik, hellinger, cs, kl
                    default is 'euclidean', RTFM!

    Returns:
        string: 'SURF-DEST-entity': 'self.displaynames[name]', (where DEST is the
                    intended destination name, should be changed by you,
                    contains training image name in this case,
                    default value is SURF-default-entity)
        float: 'time': time.time(),
        tuple: 'property_dict': {   array: 'pixel_location': location (contains top left corner????),
                                    string: 'model':model (detected training image model name)
                                }

    !!! Make sure you use function set_destination_name(string destination_name)
    to change the returned name of the detection returned to the memory, so you
    know what you should look for in the memory when processing the detections.

    The training is not done by default, you have to call the train_system(folder)
    in order to train the system before you use it.

    To start detection, use the start(bool visualize=False, int minimum_matches=3) function.
    """
    def __init__(self, update_frequency, send_socket, video_source_name, distance_type = 'euclidean', vision_window_name='VISION', detection_window_name='DETECTION', surf_params=(0, 300, 3, 4), filter=False):
        """Initialize vision system with update frequency, ip adress, and example image"""
        self.logger = logging.getLogger("Borg.Brain.Vision.SurfDetect")
        self.update_frequency = update_frequency
        self.surf_params = surf_params
        self.n_octaves = self.surf_params[2]
        self.names = []
        self.sizes = {}
        self.images = {}
        self.keypoints = []
        self.descriptors = []
        self.displaynames = {}
        self.modelKeypoints = {}
        self.destination = 'default'
        self.vid_mem_reader = util.vidmemreader.VidMemReader([video_source_name])
        self.send_socket = send_socket
        pyflann.set_distance_type(distance_type)
        self.vision_window = vision_window_name
        self.detection_window = detection_window_name
        self.imageSize = ()
        self.filter = filter #If False SURF detections won't be filtered by get_ratio()
        self.reasonable_ratio = True
        self.min_ratio = .05
        self.max_ratio = 3
        self.min_angle = math.radians(70)
        self.max_angle = math.radians(110)
        #Object with the update() function to be called (TODO: Cleanup this code):
        self.update_object = None

    def set_destination_name(self, name):
        self.destination = name

    def train_system(self, training_dir, extended=0, hessian_threshold=300, octaves=3, octave_layers=4, load_models=False, save_models=False):
        """
        Train system with files from specified directory and parameters:
        Arguments:
            String training_dir: location to the directory of the training images
            Binary extended: 0 means basic descriptors (64 elements each), 1 means extended descriptors (128 elements each)
            Integer hessian_threshold: only features with hessian larger than that are extracted.
            Integer octaves: the number of octaves to be used for extraction. With each next octave the feature size is doubled (3 by default)
            Integer octave_layers: The number of layers within each octave (4 by default)
        """
        if load_models == False:
            self.n_octaves = octaves
            self.surf_params = (extended, hessian_threshold, self.n_octaves, octave_layers)

            print "getting contents of " + training_dir
            new_training_dir = glob.glob(training_dir + '/*.*')
            for file in new_training_dir:
                print "FILE: " + file
                display_name = file.split('/')[-2]
                training_filenames = glob.glob(file + '*')
                for filename in training_filenames:
                    extension = filename.split('/')
                    if extension[-1][-3:] != 'pkl':
                        if self.update_object:
                            self.update_object.update()
                        try:
                            self.train_on_file(filename, display_name)
                        except:
                            print '\nImage does not contain features to train:'
                            print filename
            database = numpy.array(self.descriptors)
            self.NNdatabase = pyflann.FLANN()
            print database
            self.db_params = self.NNdatabase.build_index(database)
            if save_models == True:
                file = open(training_dir + '/keypoints.pkl', 'wb')
                cPickle.dump(self.keypoints, file, 2)
                file.close()
                file = open(training_dir + '/descriptors.pkl', 'wb')
                cPickle.dump(self.descriptors, file, 2)
                file.close()
                file = open(training_dir + '/displaynames.pkl', 'wb')
                cPickle.dump(self.displaynames, file, 2)
                file.close()
                file = open(training_dir + '/names.pkl', 'wb')
                cPickle.dump(self.names, file, 2)
                file.close()
                file = open(training_dir + '/sizes.pkl', 'wb')
                cPickle.dump(self.sizes, file, 2)
                file.close()
                file = open(training_dir + '/modelKeypoints.pkl', 'wb')
                cPickle.dump(self.modelKeypoints, file, 2)
                file.close()

        elif load_models == True:
            ### LOAD IMAGES ###
            new_training_dir = glob.glob(training_dir + '/*.*')
            for file in new_training_dir:
                training_filenames = glob.glob(file + '*')
                for filename in training_filenames:
                    extension = filename.split('/')
                    if extension[-1][-3:] != 'pkl':
                        try:
                            image_gray = cv.LoadImage(filename, iscolor=cv.CV_LOAD_IMAGE_GRAYSCALE)
                            object_image = cv.CreateImage(cv.GetSize(image_gray), cv.IPL_DEPTH_8U, 3)
                            cv.CvtColor(image_gray, object_image, cv.CV_GRAY2RGB)
                            self.images[filename] = object_image
                        except:
                            print '\nFile may not be an image:'
                            print filename
            file = open(training_dir + '/keypoints.pkl', 'rb')
            self.keypoints = cPickle.load(file)
            file.close()
            file = open(training_dir + '/descriptors.pkl', 'rb')
            self.descriptors = cPickle.load(file)
            file.close()
            file = open(training_dir + '/displaynames.pkl', 'rb')
            self.displaynames = cPickle.load(file)
            file.close()
            file = open(training_dir + '/names.pkl', 'rb')
            self.names = cPickle.load(file)
            file.close()
            file = open(training_dir + '/sizes.pkl', 'rb')
            self.sizes = cPickle.load(file)
            file.close()
            file = open(training_dir + '/modelKeypoints.pkl', 'rb')
            self.modelKeypoints = cPickle.load(file)
            file.close()
            database = numpy.array(self.descriptors)
            self.NNdatabase = pyflann.FLANN()
            self.db_params = self.NNdatabase.build_index(database)
        return True

    def train_on_file(self, filename, display_name):
        """train the system on the file in filename with the given name"""
        print "loading image %s" % filename
        example_img_gray = cv.LoadImage(filename, iscolor=cv.CV_LOAD_IMAGE_GRAYSCALE)
        (object_keypoints, object_descriptors) = cv.ExtractSURF(example_img_gray, None, cv.CreateMemStorage(), self.surf_params)
        self.keypoints += object_keypoints
        self.descriptors += object_descriptors
        self.displaynames[filename] = display_name
        self.names += len(object_keypoints)*[filename]
        object_image = cv.CreateImage(cv.GetSize(example_img_gray), cv.IPL_DEPTH_8U, 3)
        cv.Set(object_image,0)
        cv.CvtColor(example_img_gray, object_image, cv.CV_GRAY2RGB)
        self.images[filename] = object_image
        self.sizes[display_name] = cv.GetSize(example_img_gray)
        self.modelKeypoints[display_name] = object_keypoints

    def process_image(self, image, visualize=False, min_matches=3, min_distance=0.05, close_objects=[]):
        """
        Process the next image, trying to recognize any trained objects
        Parameters:
            int min_matches: number of minimum keypoint mathches
            int min_distances: minimum distance between keypoints matches
        """
        self.imageSize = cv.GetSize(image)
        
        
        if image.nChannels > 1:
            image_gray = cv.CreateImage(self.imageSize, cv.IPL_DEPTH_8U, 1)
            cv.Set(image_gray,0)
            cv.CvtColor(image, image_gray, cv.CV_RGB2GRAY)
        else:
            image_gray = image
        try:
            (keypoints2, descriptors2) = cv.ExtractSURF(image_gray, None, cv.CreateMemStorage(), self.surf_params)
            neighbours, dists = self.NNdatabase.nn_index(numpy.array(descriptors2), checks=self.db_params["checks"])
            close_enough = self.get_close_indexes(dists, min_distance)
            keys_per_name = self.get_keys_per_name(neighbours, close_enough)
            deletes_counter = 0
            if close_objects != []:
                for key in keys_per_name.keys():
                    if key not in close_objects:
                        del(keys_per_name[key])
                        deletes_counter += 1
                print 'Deleted KPN:\t', deletes_counter
                print 'Remaining KPN:\t', len(keys_per_name)
            detections = self.detect_objects(keys_per_name, self.keypoints, keypoints2, dists, neighbours, image, visualize, min_matches)
            return detections
        except:
            #If no objects detected only show image:
            if visualize:
                cv.ShowImage(self.vision_window, image)
                cv.WaitKey(10)
            return []

    def detect_objects(self, keys_per_name, keypoints, keypoints2, dists, neighbours, image, visualize=False, min_matches=3):
        """
        Iterate over all trained object. For each of these objects, see if there
        is enough evidence that this object is in the image, than return this
        data.
        @param keys_per_name dictionary which maps from object name to all
        keys matching with that object
        @param keypoints list of all keypoints extracted from train images
        @param keypoints2 list of all keypionts extracted from incoming image
        @param dists n-th item in this list specifies how close n-th item in
        keypoints2 is to its nearest match in keypoints
        @param neighbours n-th item in this list is an index m. This is the
        index of the point in keypoints that is the closest match to the n-th
        item in keypoints2
        @param image the incoming image
        @param visualize boolean that indicates whether the result should be
        visualized
        """
        last_detections = []
        detections = []
        for name in keys_per_name:
            (found, corners, transKeypoints) = self.check_for_object(name, keys_per_name[name], keypoints, keypoints2, dists, neighbours, image, min_matches)
            if found and self.reasonable_ratio:
                location = numpy.array(corners[0]+corners[1]+corners[2]+corners[3])/4
                model = name.split('/')[-1][:-4]
                last_detections.append(self.displaynames[name])
                detections.append( {'name':self.displaynames[name], 'time': time.time(), 'property_dict':{'corners':corners, \
                                                                                         'pixel_location':location,
                                                                                         'model':model, \
                                                                                         'destination':'surf_'+self.destination, \
                                                                                         'transKeypoints':transKeypoints}} )
            if visualize:
                cv.ShowImage(self.vision_window, image)
                cv.WaitKey(10)
                if found and self.reasonable_ratio:
                    cv.ShowImage(self.detection_window, self.images[name])
                    cv.WaitKey(10)
        if last_detections != []:
            #Keep track in the memory of the last detected items:
            detections.append( {'name':'last_surf_detection', 'time':time.time(), 'property_dict':{'object_names': last_detections}} )
        return detections

    def get_keys_per_name(self, neighbours, close_enough):
        """
        From selected keys, make a list of them per name (which is retrieved from
        the name connected to the nearest neighbour
        """
        keys_per_name = {}
        for key_idx in close_enough:
            try:
                #if the list already exists, append the new value
                keys_per_name[self.names[neighbours[key_idx]]].append(key_idx)
            except:
                #if that failes (list does not exist yet)
                #start a new list.
                keys_per_name[self.names[neighbours[key_idx]]] = [key_idx]
        return keys_per_name

    def get_close_indexes(self, dists, threshold=0.05):
        """
        Get they keys of all nearest neighbour that are less than a
        certain distance apart
        """
        close_enough = []
        for key_idx in range(len(dists)):
            if dists[key_idx] < threshold:
                close_enough.append(key_idx)
        return close_enough
               
    def check_for_object(self, name, close_enough, ex_keypoints, te_keypoints, dists, neighbours, image, min_matches=3):
        """
        Check if object 'name' is present in image.
        @param close_enough: list of all indexes in te_keypoints that have
        a small enough distance to their nearest match in ex_keypoints
        @param ex_keypoints list of all keypoints extracted from train images
        @param te_keypoints list of all keypionts extracted from incoming image
        @param dists n-th item in this list specifies how close n-th item in
        keypoints2 is to its nearest match in keypoints
        @param neighbours n-th item in this list is an index m. This is the
        index of the point in keypoints that is the closest match to the n-th
        item in keypoints2
        @param image the incoming image
        """
        largest_bin = self.hough_transform_keypoints(name, close_enough, ex_keypoints, te_keypoints, dists, neighbours, image)
        if len(largest_bin) <= min_matches: #NO DETECTION
            return (False, None, None)
        try:
            model = self.modelfit(largest_bin, ex_keypoints, te_keypoints, neighbours)
        except Exception: #modelfit doesn't work because there's not enough data
            return (False, None, None)

        te_corners = self.get_test_corners(name, model, image=None)
        max_dim = max(linalg.norm(numpy.array(te_corners[0])-numpy.array(te_corners[2])), linalg.norm(numpy.array(te_corners[1])-numpy.array(te_corners[3])))
        corresponding_to_model = self.modelcheck(largest_bin, model, ex_keypoints, te_keypoints, max_dim, neighbours)
        if len(corresponding_to_model) <= min_matches: #NO DETECTION
            return (False, None, None)
        try:
            new_model = self.modelfit(corresponding_to_model, ex_keypoints, te_keypoints, neighbours)
        except Exception: #modelfit doesn't work because there's not enough data
            return (False, None, None)

        new_te_corners = self.get_test_corners(name, new_model, image) #DETECTION
        new_te_keypoints = self.get_test_points(name, model, image) #DETECTION
        self.visualize_matching_keypoints(name, corresponding_to_model, ex_keypoints, te_keypoints, neighbours, image)
        return (True, new_te_corners, new_te_keypoints)

    ##############################################################################################################
    ############################## START EXCLUSIVE METHODS FROM: check_for_object() ##############################
    def get_test_corners(self, name, model, image=None):
        """
        Use the model to transform the corners of the example image to
        coordinates in the test image. Returns numpy array containing
        those coordinates.
        """
        ex_x = cv.GetSize(self.images[name])[0]
        ex_y = cv.GetSize(self.images[name])[1]
        ex_corners = [(0 ,0), (ex_x,0), (ex_x,ex_y), (0,ex_y)]
        te_corners = self.transform_corners(ex_corners, model)
        if image:
            self.get_ratio(te_corners)
            if self.reasonable_ratio:
                self.visualize_corners(name, ex_corners, te_corners, image)
                font = cv.InitFont(cv.CV_FONT_HERSHEY_COMPLEX, 1.0, 1.0)
                location = sum(numpy.array(te_corners))/4
                text = name.split('/')[-1][:-4]
                #cv.PutText(image, self.displaynames[name], (location[0], location[1]), font, (255,0,0))
                cv.PutText(image, text, (location[0], location[1]), font, (255,0,0))

        return numpy.array(te_corners)

    def transform_corners(self, points, model):
        """
        Transform the corners of the train image according to the best model
        of the object in the incoming image
        """
        tpoints = []
        for point in points:
            transformed = numpy.dot(numpy.array([[model[0], model[1]], [model[2], model[3]]]), numpy.array(point)) + numpy.array([model[4],model[5]])
            corner = (int(round(transformed.tolist()[0])), int(round(transformed.tolist()[1])))
            tpoints.append(corner)
        return tpoints

    def visualize_corners(self, name, ex_corners,te_corners, image):
        """
        Show the transformed rectangle where the best match of the object name
        was found
        """
        for i in range(4):
            thickness = 2
#            color1 = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
            color2 = (0,255,0)
#            cv.Line(self.images[name], ex_corners[i], ex_corners[(i+1)%4], color1, thickness)
            cv.Line(image, te_corners[i], te_corners[(i+1)%4], color2, thickness)
        
    def get_test_points(self, filename, model, image=None):
        """
        Use the model to transform the corners of the example image to
        coordinates in the test image. Returns numpy array containing
        those coordinates.
        """
        name = self.displaynames[filename]
        ex_points = self.modelKeypoints[name]
        te_points = self.transform_points(ex_points, model)
        if image and self.reasonable_ratio:
            self.visualize_points(filename, ex_points, te_points, image)
        #return numpy.array(te_points)
        return te_points

    def transform_points(self, ex_points, model):
        """
        Transform the corners of the train image according to the best model
        of the object in the incoming image
        """
        #Create array of old points:
        tpoints = []
        for point in ex_points:
            transformed = numpy.dot(numpy.array([[model[0], model[1]], [model[2], model[3]]]) , numpy.array([point[0][0], point[0][1]])) + numpy.array([model[4],model[5]])
            newPoint = (int(round(transformed.tolist()[0])), int(round(transformed.tolist()[1])))
            tpoints.append(newPoint)
        return tpoints

    def visualize_points(self, filename, ex_points, te_points, image):
        """
        Show the transformed rectangle where the best match of the object name
        was found
        """
        #Visualize keypoints in training image:
        for point in range(len(ex_points)):
            thickness = 2
            color2 = (0,255,0)
            x = int(round(ex_points[point][0][0]))
            y = int(round(ex_points[point][0][1]))
            cv.Line(self.images[filename], (x-1,y), (x+1,y), color2, thickness)
            cv.Line(self.images[filename], (x,y-1), (x,y+1), color2, thickness)

        #Visualize keypoints in testing image:
        for point in range(len(te_points)):
            try:
                thickness = 2
                color2 = (0,255,0)
                x = te_points[point][0]
                y = te_points[point][1]
                cv.Line(image, (x-1,y), (x+1,y), color2, thickness)
                cv.Line(image, (x,y-1), (x,y+1), color2, thickness)
            except:
                pass
    ############################## FINISH EXCLUSIVE METHODS FROM: check_for_object() ##############################
    ###############################################################################################################

    def modelcheck(self,indexes, model, keypoints, keypoints2, max_dim, neighbours):
        """
        Check which of the keypoints in indexes are corresponding close enough
        to the model
        @param model parameters of the transformation from test image to
        current incoming image
        @param keypoints list of all keypoints extracted from train images
        @param keypoints2 list of all keypionts extracted from incoming image
        @param neighbours n-th item in this list is an index m. This is the
        index of the point in keypoints that is the closest match to the n-th
        item in keypoints2
        """
        max_relative_distance = 0.05
        max_distance = max_relative_distance * max_dim
        corresponding_to_model = []
        for key_idx in indexes:
            location1 = numpy.array((int(round(keypoints2[key_idx][0][0])), int(round(keypoints2[key_idx][0][1]))))
            location2 = numpy.array((int(round(keypoints[neighbours[key_idx]][0][0])), int(round(keypoints[neighbours[key_idx]][0][1]))))

            #project location2 to expected location in example image
            transformed = numpy.dot(numpy.array([[model[0], model[1]], [model[2], model[3]]]), location2) + numpy.array([model[4],model[5]])
            distance = numpy.linalg.norm(transformed - location1)
            if distance < max_distance:
                corresponding_to_model.append(key_idx)
        return corresponding_to_model

    def hough_transform_keypoints(self, name, close_enough, keypoints, keypoints2, dists, neighbours, image, visualization=False):
        """
        Put each keypoint in a bin correspoding to scale difference, orientation
        difference, and expected location of the object center. Only keypoints that
        between them agree on these points, will be considered in matching the model
        
        @param close_enough: list of all indexes in te_keypoints that have
        a small enough distance to their nearest match in ex_keypoints
        @param keypoints list of all keypoints extracted from train images
        @param keypoints2 list of all keypionts extracted from incoming image
        @param dists n-th item in this list specifies how close n-th item in
        keypoints2 is to its nearest match in keypoints
        @param neighbours n-th item in this list is an index m. This is the
        index of the point in keypoints that is the closest match to the n-th
        item in keypoints2
        @param image the incoming image
        @param visualization whether the result should be visualizes
        """
        bins_spatial = 5
        bins_ori = 12
        scale_binwidth = 1.7 #maximal factor for two point to be in same bin
        max_scale_factor = int(round(math.log(2**self.n_octaves,scale_binwidth)))
        n_scale_bins = 1 + 2*max_scale_factor
        hist = [[[[[] for i in range(n_scale_bins)] for j in range(bins_ori)] for x in range(bins_spatial)] for y in range(bins_spatial)]
        test_size = cv.GetSize(image)
        largest_bin = []
        for test_idx in close_enough:            
            test_keypoint = keypoints2[test_idx]
            example_keypoint = keypoints[neighbours[test_idx]]
            bin = self.get_hough_bin(name, test_keypoint, example_keypoint, bins_spatial, bins_ori, scale_binwidth, test_size)
            (y_bin, x_bin, orientation_bin, scale_bin) = bin
#            if visualization:
#                cv.Circle(self.example_img, example_keypoint[0], int(round(keypoints2[testIdx][2])), (128,128,128))
#                cv.Circle(image,test_keypoint[0], int(round(keypoints2[testIdx][2])), (128,128,128))
#                cv.Line(image, test_keypoint[0], (test_keypoint[0][0]+te_to_center[0], test_keypoint[0][1]+te_to_center[1]), (0,0,0))
            if(x_bin >= 0 and x_bin < bins_spatial and y_bin >= 0 and y_bin < bins_spatial):
                hist[y_bin][x_bin][orientation_bin][scale_bin].append(test_idx)
                if len(hist[y_bin][x_bin][orientation_bin][scale_bin]) > len(largest_bin):
                    largest_bin = hist[y_bin][x_bin][orientation_bin][scale_bin]
        return largest_bin

    def get_hough_bin(self, name, test_keypoint, example_keypoint, bins_spatial, bins_ori, scale_binwidth, test_size):
        """
        Get the bin in the histogram where this keypoint has to go
        """
        max_scale_factor = int(round(math.log(2**self.n_octaves,scale_binwidth)))
        ex_center = 0.5*numpy.array(cv.GetSize(self.images[name]))       
        scale_factor = test_keypoint[2] / float(example_keypoint[2])
        scale_bin = min(int(math.log(scale_factor,scale_binwidth)),max_scale_factor) + max_scale_factor
        orientation_dif = (example_keypoint[3] - test_keypoint[3]) % 360
        ori_bin = int(orientation_dif / float(360) * bins_ori)
        ex_to_center = numpy.array(ex_center) - numpy.array(example_keypoint[0])
        rot = orientation_dif /180.0 * math.pi
        rot_matrix = numpy.array([[math.cos(rot), -math.sin(rot)],[math.sin(rot), math.cos(rot)]])
        te_to_center = scale_factor*numpy.dot(rot_matrix, ex_to_center)
        proj_center = te_to_center + numpy.array(test_keypoint[0])
        x_bin = int(proj_center[0] * bins_spatial / float(test_size[0]))
        y_bin = int(proj_center[1] * bins_spatial / float(test_size[1]))
        return (y_bin, x_bin, ori_bin, scale_bin)

    def visualize_matching_keypoints(self, name, indexes, keypoints, keypoints2, neighbours, image):
        """
        Show the corresponding keypoints of the training image and the test image
        """
        if self.reasonable_ratio:
            for idx in indexes:
                #location1 = (int(round(keypoints2[idx][0][0])), int(round(keypoints2[idx][0][1]))) #VISION WINDOW
                location2 = (int(round(keypoints[neighbours[idx]][0][0])), int(round(keypoints[neighbours[idx]][0][1]))) #DETECTION WINDOW
                #color = (random.randint(0,255), random.randint(0,255), random.randint(0,255)) #Different color for each keypoint
                color = (0, 0, 255) #Red keypoints
                #cv.Circle(image,location1, int(round(keypoints2[idx][2]/2)), color) #VISION WINDOW
                cv.Circle(self.images[name], location2, int(round(keypoints[neighbours[idx]][2]/2)), color) #DETECTION WINDOW

    def modelfit(self, largest_bin, keypoints, keypoints2, neighbours):
        """
        Fit a model to the keypoint matches. Least square method as
        explained in:
        Lowe 2002 (Distinctive image features from scale-invariant keypoints)
        Single-letter variables are used here to keep correspondence with the
        explanation in the paper
        """
        #vector b contains the coordinates in the test image
        #Matrix A contains the coordinates in the original image       
        A = []
        b = []
        for testIdx in largest_bin:
            test_location = (int(round(keypoints2[testIdx][0][0])), int(round(keypoints2[testIdx][0][1])))
            training_location = (int(round(keypoints[neighbours[testIdx]][0][0])), int(round(keypoints[neighbours[testIdx]][0][1])))
            A.append([training_location[0], training_location[1],0,0,1,0])
            A.append([0,0,training_location[0], training_location[1],0,1])
            b.append(test_location[0])
            b.append(test_location[1])
        A = numpy.array(A)
        b = numpy.array(b)
        x = numpy.dot(numpy.dot(linalg.inv(numpy.dot(A.T, A)), A.T), b)
        # x contains the four numbers from the transformation matrix and
        # to transform coordinates (pt[0], pt[1]), do:
        #
        # | x[0] x[1] |   | pt[0] |   | x[4] |
        # | x[2] x[3] | . | pt[1] | + | x[5] |
        #
        # or in python code:
        # numpy.dot(numpy.array([[x[0], x[1]], [x[2], x[3]]]), numpy.array([pt[0], pt[1])) + numpy.array([x[4],x[5]])
        return x

    def start(self, visualize=False, min_matches=3, min_distances=0.05):
        """
        Starts detection from the selected source specified in the constructor.
        Parameters:
            int min_matches: number of minimum keypoint mathches
            int min_distances: minimum distance between keypoints matches
        """
        while True:
            start_time = time.time()
            img = self.get_new_image()
            results = self.process_image(img, visualize, min_matches, min_distances)
            self.write_message(results)
            time_spent = time.time() - start_time
            sleep_time = 1.0/self.update_frequency -time_spent
            time.sleep(max(sleep_time,0))

    def write_message(self, message):
        """will be used to write detections on TCP/IP socket"""
        self.send_socket.send(message)

    def get_new_image(self):
        """get new image from robot"""
        return self.vid_mem_reader.get_latest_image()[0]

    def angle(self, v1, v2):
        #SUGGESTION: Measurable improvement in speed at no coding or readability cost:
        #math.sqrt(x)  is equivalent to x**0.5   and   math.pow(x,y) is equivalent to x**y
        #       For '**' interpreter goes directly to bytecode BINARY_POWER
        #VS.    Lookup 'math', access attribute 'sqrt', and slow bytecode CALL_FUNCTION
        dot_product = sum((a*b) for a, b in zip(v1, v2))
        length = (dotproduct(v, v))**0.5
        return math.acos(dot_product(v1, v2) / (length(v1) * length(v2)))

    def slope_angle(self, x1, y1, x2, y2):
        return math.atan( float(y2-y1) / (x2-x1) )

    def lines_angle(self, slope_1, slope_2):
        return math.atan( (math.tan(slope_2) - math.tan(slope_1)) / (1 + math.tan(slope_1) * math.tan(slope_2)) )

    def get_ratio(self, corners):
        if self.filter:
            width_x  = float(corners[0][0] - corners[1][0])
            width_y  = float(corners[0][1] - corners[1][1])
            height_x = float(corners[1][0] - corners[2][0])
            height_y = float(corners[1][1] - corners[2][1])
            width_slope_angle =  self.slope_angle(corners[0][0], corners[0][1], corners[1][0], corners[1][1])
            height_slope_angle = self.slope_angle(corners[1][0], corners[1][1], corners[2][0], corners[2][1])

            width =  ( width_x**2 +  width_y**2)**0.5
            height = (height_x**2 + height_y**2)**0.5
            model_ratio = height/width
            image_width_ratio =   width / self.imageSize[0]
            image_height_ratio = height / self.imageSize[1]
            detection_square_angle = abs( self.lines_angle(width_slope_angle, height_slope_angle) )

            if             (model_ratio < self.max_ratio) and            (model_ratio > self.min_ratio) and \
                     (image_width_ratio < self.max_ratio) and      (image_width_ratio > self.min_ratio) and \
                    (image_height_ratio < self.max_ratio) and     (image_height_ratio > self.min_ratio) and \
                (detection_square_angle < self.max_angle) and (detection_square_angle > self.min_angle):
                self.reasonable_ratio = True
            else:
                self.reasonable_ratio = False


def usage():
    """print usage information"""
    print "Configuration file is a mandatory arguments, and should contain"
    print "the right options:"
    print ""
    print "Usage: ", sys.argv[0], " config file"
    print ""
    print "Config file should contain: controller_ip, surfdetect (port nr), "
    print "update_frequency, and video_source_name"
    print "Where videosource = [nao] (other sources to be implemented...)"

if __name__ == "__main__":
    print os.path.abspath(os.path.dirname(__file__)+'/../../data/training_img')    
    if len(sys.argv) < 2:
        usage()
        exit()

    config_dict = configparse.ParameterDict()
    configparse.parse_config(sys.argv[1], config_dict)
    vsec = "vision_controller" #section in config_dict
    gsec = "general"
    module_name = "surfdetect"
    modules = config_dict.get_option(vsec, "modules")
    if module_name not in modules.split():
        print "not using", module_name
        exit()

    update_frequency_s = config_dict.get_option(vsec, "update_frequency")
    robot_ip = config_dict.get_option(gsec, "robot_ip")
    controller_ip = config_dict.get_option(vsec, "controller_ip")
    controller_port_s = config_dict.get_option(vsec, module_name)
    video_source_name = config_dict.get_option(vsec, "video_source_name")
    training_dir = os.path.join( os.path.abspath(os.environ['BORG']) , config_dict.get_option(vsec, "training_dir"))
    if not (update_frequency_s and robot_ip and controller_ip and
        controller_port_s and video_source_name):
        usage()
        exit()

    update_frequency = float(update_frequency_s)
    controller_port = int(controller_port_s)    
    send_socket = util.sendsocket.SendSocket(controller_ip, controller_port)
    #send_socket = util.testsocket.TestSocket(to_print=True)
    #training_dir = os.path.abspath(os.environ['BORG'] + '/Brain/data/training_img')
    vision = SurfDetect(update_frequency, send_socket, video_source_name)
    if vision.train_system(training_dir):
        print "Training complete"
        vision.start(True)
    else:
        print "Training failed"

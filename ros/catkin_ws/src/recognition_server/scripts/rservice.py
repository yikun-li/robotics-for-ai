#!/usr/bin/env python
from __future__ import print_function

import actionlib
import cv2
import numpy as np
import rospy
import copy
import time
import tensorflow as tf
import os
from alice_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from recognition_server.msg import *
from sensor_msgs.msg import Image

from network import Network


class RService:
    def __init__(self):
        config = tf.ConfigProto(device_count={"GPU": 0})
        self.sess = tf.InteractiveSession(config=config)
        self.client = actionlib.SimpleActionClient("ObjectROI", ObjectROIAction)
        print('Waiting ROI server')
        self.client.wait_for_server()
        print('Connected to ROI server')
        self.counter = 0

        self.bridge = CvBridge()  # Use CvBridge for converting
        self.image = rospy.wait_for_message("/front_xtion/rgb/image_raw", Image)
        # self.labelPath = '/home/student/dataset/labels.txt'

        # load neural network and labels for Google Neural Network
        self.labels = self.read_labels(os.environ['BORG'] + "/ros/ckpt/output_labels.txt")
        self.sess = self.loadNetwork(os.environ['BORG'] + "/ros/ckpt/output_graph.pb")

        self.action_server = actionlib.SimpleActionServer("image_server", ProcessAction, self.callback, False)
        self.action_server.start()

        self.height = 32
        self.width = 32
        self.nClasses = 10

        # self.CHECKPOINT_DIR = "./ckpt/network.ckpt"
        # self.network = Network()
        # self.network.load_checkpoint(self.CHECKPOINT_DIR)

        self.list_label = [
            'Fanta',
            'SportsDrink',
            'ChickenSoup',
            'TomatoSoup',
            'Shampoo',
            'EraserBox',
            'Coke',
            'Salt',
            'Chips',
            'YellowContainer']

        self.count = 0
        self.success = 0

    def callback(self, rec):
        if rec.state == 0:
            self.action_server.set_aborted('Please send the correct command!')

        # receiving image here
        self.image = rospy.wait_for_message("/front_xtion/rgb/image_raw", Image)
        # self.labelPath = '/home/student/dataset/labels.txt'
        rgb_image = self.convert_image_cv(self.image, state=rec.state)

        goal = ObjectROIGoal()  # Create a goal message
        goal.action = "findObjects"  # 'findObjects' is currently the only action that can be done
        self.client.send_goal(goal)  # Send a goal to start the action server to find ROIs

        self.client.wait_for_result(
            rospy.Duration.from_sec(60))

        ROIs = None
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            # print('Success')
            client_data = self.client.get_result()
            ROIs = client_data.roi

        elif self.client.get_state() == actionlib.GoalStatus.ABORTED:
            print('It most likely could not find any objects!')
            return

        list_objects = []
        list_confidence = []
        list_position = []
        list_scanned_objects = []
        if ROIs is None:
            return

        for i in range(len(ROIs)):
            top = ROIs[i].top
            bottom = ROIs[i].bottom
            left = ROIs[i].left
            right = ROIs[i].right
            # print('top', top)
            # print('bottom', bottom)
            # print('left', left)
            # print('right', right)

            height = bottom - top
            width = right - left
            position = (right + left) / 2.0

            if width > 200:
                break

            padding1, padding2 = 1, 1
            if height > width:
                padding2 += (height - width) / 2
            else:
                padding1 += (width - height) / 2

            # obj = rgb_image[ROIs[i].top - padding1: ROIs[i].bottom + padding1,
            #       ROIs[i].left - padding2: ROIs[i].right + padding2]
            # obj = rgb_image[ROIs[i].top - padding:ROIs[i].bottom + padding,
            #       ROIs[i].lROIs[i].top - padding1: ROIs[i].bottom + padding1,

            new_top = ROIs[i].top - padding1
            new_bottom = ROIs[i].bottom + padding1
            new_left = ROIs[i].left - padding2
            new_right = ROIs[i].right + padding2

            new_top = new_top if new_top >= 0 else 0
            new_bottom = new_bottom if new_bottom >= 0 else 0
            new_left = new_left if new_left >= 0 else 0
            new_right = new_right if new_right >= 0 else 0
            cv_image = self.bridge.imgmsg_to_cv2(self.image, 'bgr8')  # use "bgr8" if its a color image
            rgb_obj = cv_image[new_top: new_bottom, new_left: new_right]

            obj = rgb_image[new_top: new_bottom, new_left: new_right]

            # data = obj.astype('float')
            # data /= 255
            result, labIdx = -1, -1

            if rec.state == 1:
                i = self.preprocess(obj)
                # result = self.network.feed_batch(i)
                # print(result)
                # print('Object: ', self.list_label[int(np.argmax(result))])

                # self.count += 1
                # if int(np.argmax(result)) == 7:
                #     self.success += 1
                # print('Rate: ' + str(float(self.success) / float(self.count)))

            elif rec.state == 2:
                i = self.preprocess_g(obj)
                out, labIdx = self.run_inference_on_image(i)
                list_objects.append(self.labels[labIdx])
                list_confidence.append(out[labIdx])
                list_scanned_objects.append(rgb_obj)
                list_position.append(str(position))

            else:
                self.action_server.set_aborted('Input parameter is wrong!')
                return

        if len(list_objects) != 3:
            rtn = ProcessResult()
            rtn.obj = 'Objects are not 3!'
            self.action_server.set_aborted(rtn)

        try:
            for ind, obj in enumerate(list_scanned_objects):
                cv2.imshow('image', obj)
                cv2.waitKey(1000)
                cv2.destroyAllWindows()
                cv2.imwrite('./images/' + list_objects[ind] + ' + ' + str(list_confidence[ind]) + ' + ' +
                            str(time.time()).split('.')[0] + ".png", obj)

            rtn = ProcessResult()
            if rec.state == 1:
                pass
                # rtn.obj = self.list_label[int(np.argmax(result))]
            elif rec.state == 2:
                rtn.obj = '|'.join(list_objects)

            rtn.result = '|'.join(list_position)
            print(rtn.obj)
            print(rtn.result)
            self.action_server.set_succeeded(rtn)
            return
        except Exception as e:
            print(e)

        rtn = ProcessResult()
        rtn.obj = 'Did not find the object!'
        self.action_server.set_aborted(rtn)

    # convert from sensor_msg format to opencv format (Numpy)
    def convert_image_cv(self, data, i_type="rgb8", state="1"):  # use type = "8UC1" for grayscale images
        cv_image = None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, i_type)  # use "bgr8" if its a color image
            if state == 1:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        except CvBridgeError as e:
            print(e)
        return cv_image

    def preprocess(self, image):
        image = cv2.resize(image, (self.height, self.width))
        image = image.reshape((-1, self.height, self.width, 3))
        image = image.astype('uint8')
        return image

    # This function will process one image such that it is usable by Mobilenet.
    def preprocess_g(self, img):  # input is a numpy image
        # First we'll resize the image such that the network can process it. Possible imagesizes are: '224', '192',
        # '160', or '128' (See retrain.py line 84 comments) By default, 'run.sh' uses the 224 network. You may choose
        # anotherone, by changing the 224 part in 'run.sh' to another valid size.

        img = cv2.resize(img, (224, 224))
        img = img.astype(float) / 255  # goto [0.0,1.0] instead of [0,255]
        img = np.reshape(img, (1, 224, 224, 3))
        return img

    # This function will load a trained network file (probably called output_graph.pb in ./tmp/) It will return a tf
    # session, which should be stored as a member variable (in your class). I use it globally here as an example.
    def loadNetwork(self, pb_file_location):
        f = tf.gfile.FastGFile(pb_file_location, 'rb')
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
        _ = tf.import_graph_def(graph_def, name='')
        # print graph_def
        return tf.Session()

    # This will read the label file, and return a list of labels, in the same index order the output layer reasons in.
    def read_labels(self, label_file):
        with open(label_file) as f:
            content = f.readlines()
        content = [x.strip() for x in content]
        return content

    # This function will run
    def run_inference_on_image(self, image_data):
        softmax_tensor = self.sess.graph.get_tensor_by_name('final_result:0')  # resolve layer from .pb file
        predictions = self.sess.run(softmax_tensor, {'input:0': image_data})  # Run network on your input
        predictions = np.squeeze(predictions)  # make sure output dims are correct
        #    print predictions
        norm = predictions / np.sqrt(predictions.dot(predictions))  # normalize output data (make softmax out of it)
        # print("out: ", norm, " label = ", np.argmax(norm))
        return norm, np.argmax(norm)


if __name__ == '__main__':
    try:
        rospy.init_node("recognition_service")
        server = RService()
        rospy.spin()
    except Exception as e:
        print(e)

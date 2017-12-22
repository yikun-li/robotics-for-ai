#!/usr/bin/env python
from __future__ import print_function

import actionlib
import cv2
import numpy as np
import rospy
import tensorflow as tf
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
        self.labelPath = '/home/student/dataset/labels.txt'

        self.action_server = actionlib.SimpleActionServer("image_server", ProcessAction, self.callback, False)
        self.action_server.start()

        self.height = 32
        self.width = 32
        self.nClasses = 10
        self.CHECKPOINT_DIR = "./ckpt/network.ckpt"

        self.network = Network()
        self.network.load_checkpoint(self.CHECKPOINT_DIR)

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

    def callback(self, goal):
        if goal.state == 0:
            self.action_server.set_aborted('Please send the correct command!')

        # receiving image here
        rgb_image = self.convert_image_cv(self.image)

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

        for i in range(len(ROIs)):
            top = ROIs[i].top
            bottom = ROIs[i].bottom
            left = ROIs[i].left
            right = ROIs[i].right

            height = bottom - top
            width = right - left

            if width > 200:
                break

            padding1, padding2 = 5, 5
            if height > width:
                padding2 += (height - width) / 2
            else:
                padding1 += (width - height) / 2

            obj = rgb_image[ROIs[i].top - padding1: ROIs[i].bottom + padding1,
                  ROIs[i].left - padding2: ROIs[i].right + padding2]
            # obj = rgb_image[ROIs[i].top - padding:ROIs[i].bottom + padding,
            #       ROIs[i].left - padding:ROIs[i].right + padding]

            # data = obj.astype('float')
            # data /= 255

            i = self.preprocess(obj)
            result = self.network.feed_batch(i)
            # print(result)
            print(self.list_label[int(np.argmax(result))])

            self.count += 1
            if int(np.argmax(result)) == 6:
                self.success += 1
            print('Rate: ' + str(float(self.success) / float(self.count)))

            cv2.imshow('image', obj)
            cv2.waitKey(1000)
            cv2.destroyAllWindows()

            try:
                rtn = ProcessResult()
                rtn.obj = self.list_label[int(np.argmax(result))]
                rtn.result = '{0:.2f}'.format(float(self.success) / float(self.count))
                self.action_server.set_succeeded(rtn)
                return
            except Exception as e:
                print(e)

        self.action_server.set_aborted('Did not find the object!')

    # convert from sensor_msg format to opencv format (Numpy)
    # goal.image.encoding)
    def convert_image_cv(self, data, i_type="rgb8"):  # use type = "8UC1" for grayscale images
        cv_image = None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, i_type)  # use "bgr8" if its a color image
            # print cv_image
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        except CvBridgeError as e:
            print(e)
        return cv_image

    def preprocess(self, image):
        image = cv2.resize(image, (self.height, self.width))
        image = image.reshape((-1, self.height, self.width, 3))
        image = image.astype('uint8')
        return image


if __name__ == '__main__':
    try:
        rospy.init_node("recognition_service")
        server = RService()
        rospy.spin()
    except Exception as e:
        print(e)

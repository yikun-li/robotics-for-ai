#!/usr/bin/env python
from __future__ import print_function

import actionlib
import cv2
import numpy as np
import rospy
import tensorflow as tf
from alice_msgs.msg import *
from recognition_server.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class RService:
    def __init__(self):
        config = tf.ConfigProto(device_count={"GPU": 0})
        self.sess = tf.InteractiveSession(config=config)
        self.client = actionlib.SimpleActionClient("ObjectROI", ObjectROIAction)
        print('Waiting ROI server')
        self.client.wait_for_server()
        print('Connected to ROI server')

        self.bridge = CvBridge()  # Use CvBridge for converting
        self.image = rospy.wait_for_message("/front_xtion/rgb/image_raw", Image)
        self.labelPath = '/home/student/dataset/labels.txt'

        self.action_server = actionlib.SimpleActionServer("image_server", ProcessAction, self.callback, False)
        self.action_server.start()

    def callback(self, goal):
        # if self.counter < 30:
        #     self.counter += 1;

        #     return;
        # else:
        #     self.counter = 0;

        # receiving image here
        rgb_image = self.convert_image_cv(self.image)

        goal = ObjectROIGoal()  # Create a goal message
        goal.action = "findObjects"  # 'findObjects' is currently the only action that can be done
        self.client.send_goal(goal)  # Send a goal to start the action server to find ROIs

        ### EXAMPLE FOR TESTING ONLY
        # self.client.wait_for_result(
        #     rospy.Duration.from_sec(60));  # DON'T DO THIS IN YOUR BEHAVIOUR!!!!!!!!!!!!!!!!!!!!
        ### END EXAMPLE FOR TESTING ONLY

        ROIs = None
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            client_data = self.client.get_result()
            ROIs = client_data.roi

        elif self.client.get_state() == actionlib.GoalStatus.ABORTED:
            print('It most likely could not find any objects!')
            return

        for i in range(len(ROIs)):
            padding = 1
            obj = rgb_image[ROIs[i].top - padding:ROIs[i].bottom + padding, ROIs[i].left -
                                                                            padding:ROIs[i].right + padding]

            obj2 = np.zeros((224, 224))
            obj2 = cv2.normalize(obj, obj2, -1, 1)
            print(np.max(obj2), np.min(obj2))
            obj = cv2.resize(obj2, (224, 224))

            answer = self.run_inference_on_image([obj])
            cv2.putText(obj, answer, (0, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1, 1)
            cv2.imshow("object", obj)
            print(answer)

            cv2.waitKey(2000)
            cv2.destroyAllWindows()

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

    def run_inference_on_image(self, img):
        softmax_tensor = self.sess.graph.get_tensor_by_name('final_result:0')
        predictions = self.sess.run(softmax_tensor,
                                    {'input:0': img})

        predictions = np.squeeze(predictions)

        top_k = predictions.argsort()[-5:][::-1]  # Getting top 5 predictions
        f = open(self.labelPath, 'rb')
        lines = f.readlines()
        labels = [str(w).replace("\n", "") for w in lines]
        print('\n New run: \n')
        for node_id in top_k:
            human_string = labels[node_id]
            score = predictions[node_id]
            print('%s (score = %.5f)' % (human_string, score))

        answer = labels[top_k[0]]
        return answer


if __name__ == '__main__':
    try:
        rospy.init_node("recognition_service")
        server = RService()
        rospy.spin()
    except Exception as e:
        print(e)

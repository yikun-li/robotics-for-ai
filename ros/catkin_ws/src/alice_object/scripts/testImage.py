import actionlib
import cv2
import numpy as np
import rospy
import tensorflow as tf
from alice_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

modelFullPath = '/home/rik/CNN/tmp/output_graph.pb'
labelsFullPath = '/home/rik/CNN/tmp/output_labels.txt'


class Classifier(object):

    def __init__(self):
        self.counter = 0;
        config = tf.ConfigProto(device_count={"GPU": 0});
        # config.gpu_options.allow_growth = True;
        self.sess = tf.InteractiveSession(config=config);

        self.client = actionlib.SimpleActionClient("ObjectROI",
                                                   ObjectROIAction);  # ObjectROIAction comes from alice_msgs

        print 'Waiting for ROI action server';
        self.client.wait_for_server();  # no timeout is given, we can wait as long as needed
        print 'Connected to ROI action server';

        # self.sess = tf.InteractiveSession();
        self.create_graph();
        self.bridge = CvBridge();

        # sub = rospy.Subscriber("/kinect2/qhd/image_color", Image, callback, queue_size=1);
        self.sub = rospy.Subscriber("/front_xtion/rgb/image_raw", Image, self.callback, queue_size=1);

    def create_graph(self):
        """Creates a graph from saved GraphDef file and returns a saver."""
        # Creates graph from saved graph_def.pb.
        with tf.gfile.FastGFile(modelFullPath, 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
            _ = tf.import_graph_def(graph_def, name='')

    def run_inference_on_image(self, img):

        softmax_tensor = self.sess.graph.get_tensor_by_name('final_result:0')
        predictions = self.sess.run(softmax_tensor,
                                    {'input:0': img})

        predictions = np.squeeze(predictions)

        top_k = predictions.argsort()[-5:][::-1]  # Getting top 5 predictions
        f = open(labelsFullPath, 'rb')
        lines = f.readlines()
        labels = [str(w).replace("\n", "") for w in lines]
        print '\n New run: \n';
        for node_id in top_k:
            human_string = labels[node_id]
            score = predictions[node_id]
            print('%s (score = %.5f)' % (human_string, score))

        answer = labels[top_k[0]]
        return answer

    ## convert from sensor_msg format to opencv format (Numpy)
    def convert_image_cv(self, data, type="rgb8"):  # use type = "8UC1" for grayscale images
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, type)  # use "bgr8" if its a color image
        except CvBridgeError, e:
            print e

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB);

        return cv_image

    def callback(self, data):

        if (self.counter < 30):
            self.counter += 1;
            return;
        else:
            self.counter = 0;
        # receiving image here
        rgbImage = self.convert_image_cv(data);

        goal = ObjectROIGoal();  # Create a goal message
        goal.action = "findObjects";  # 'findObjects' is currently the only action that can be done

        self.client.send_goal(goal);  # Send a goal to start the action server to find ROIs

        ### EXAMPLE FOR TESTING ONLY
        self.client.wait_for_result(rospy.Duration.from_sec(60));  # DON'T DO THIS IN YOUR BEHAVIOUR!!!!!!!!!!!!!!!!!!!!
        ### END EXAMPLE FOR TESTING ONLY

        if (self.client.get_state() == actionlib.GoalStatus.SUCCEEDED):
            client_data = self.client.get_result();
            ROIs = client_data.roi;

        elif (self.client.get_state() == actionlib.GoalStatus.ABORTED):
            print 'It most likely could not find any objects!';
            return;

        for i in range(len(ROIs)):
            padding = 1
            obj = rgbImage[ROIs[i].top - padding:ROIs[i].bottom + padding,
                  ROIs[i].left - padding:ROIs[i].right + padding];

            obj2 = np.zeros((224, 224));
            obj2 = cv2.normalize(obj, obj2, -1, 1);
            print np.max(obj2), np.min(obj2)
            obj = cv2.resize(obj2, (224, 224));

            answer = self.run_inference_on_image([obj]);
            cv2.putText(obj, answer, (0, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1, 1);
            cv2.imshow("object", obj);

            print answer
            cv2.waitKey(2000);
            cv2.destroyAllWindows();


if __name__ == "__main__":

    rospy.init_node("Object_Recog_test");
    classifier = Classifier();

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting Down"

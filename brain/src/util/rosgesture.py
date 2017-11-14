import rospy
import std_msgs
import logging
import brain
import util.nullhandler
import time

logging_namespace = 'Borg.Brain.Util.RosGesture'
logging.getLogger(logging_namespace).addHandler(util.nullhandler.NullHandler())

class RosGesture(object):

    def __init__(self, topic):
        self.__logger = logging.getLogger('Borg.Brain.Util.RosGesture')
        self.__topic = topic
        self.__latest_ros_gesture = None
        self.rosgesture_subscriber = rospy.Subscriber(self.__topic, std_msgs.msg.String, self.ros_subscriber_callback)
        self.__latest_ros_gesture_time = 0

    def ros_subscriber_callback(self, data):
        self.__logger.debug("Receiving data from ROS Gesture Recognizer from subscriber for topic: %s" % self.__topic)
        self.__latest_ros_gesture = data
        self.__latest_ros_gesture_time = time.time()

    def get_latest_gesture(self):
        return (self.__latest_ros_gesture_time, self.__latest_ros_gesture)

if __name__ == "__main__":
    brain.setup_logging(logging.getLogger(logging_namespace), None, None, "DEBUG")
    rosgesture = RosGesture("/gesture_recognizer")
    while True:
        time.sleep(1)

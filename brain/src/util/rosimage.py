import rospy
import roslib; roslib.load_manifest('borg_pioneer')
import sensor_msgs

#To convert between open cv and ros image messages:
import cv_bridge

import logging
import cv

class RosImage(object):

    def __init__(self, topic, encoding = "passthrough"):
        self.__logger = logging.getLogger('Borg.Brain.Util.RosImage')

        self.__cv_bridge = cv_bridge.CvBridge()

        self.__topic = topic
        self.__encoding = encoding

        rospy.init_node('rosimage', anonymous=True)

        self.image_subscriber = rospy.Subscriber(self.__topic, sensor_msgs.msg.Image, self.ros_subscriber_callback)
        #Only use cv_bridge if needed:
        self.__latest_ros_image = None
        self.__latest_processed_ros_image = None
        self.__latest_cv_image = None
        self.__latest_time = 0

    def get_time(self):
        return self.__latest_time

    def get_image(self):
        #self.__logger.debug("Retreiving ROS image for topic: %s" % self.__topic)
        if self.__latest_processed_ros_image == self.__latest_ros_image:
            return self.__latest_cv_image
        #TODO: Make encoding ('passthrough', 'bgr8', etc. ) configurable?
        self.__latest_cv_image = cv.GetImage(self.__cv_bridge.imgmsg_to_cv(self.__latest_ros_image, self.__encoding))
        self.__latest_processed_ros_image = self.__latest_ros_image
        return self.__latest_cv_image

    def ros_subscriber_callback(self, data):
        #self.__logger.debug("Receiving ROS image from subscriber for topic: %s" % self.__topic)
        self.__latest_ros_image = data
        self.__latest_time = data.header.stamp.to_sec()

    def store_harddisk(self,filename,image):
        cv.SaveImage(filename, image)

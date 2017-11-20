#!/usr/bin/env python
import rospy
from part2.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import actionlib
import cv2
import numpy


class ProcessImageServer():
    """docstring for ProcessImageServer"""
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer("image_server", ProcessAction, self.callback, False)
        self.action_server.start()

    def callback(self, goal):
        try:
	    # print goal
            bridge = CvBridge() # Use CvBridge for converting
            cv_image = bridge.imgmsg_to_cv2(goal.image, goal.image.encoding) # use "bgr8" if its a color image
            # print cv_image

            if (goal.image.encoding == 'bgr8'):
                l = numpy.array([goal.min, goal.min, goal.min], dtype = 'uint8')
                u = numpy.array([goal.max, goal.max, goal.max], dtype = 'uint8')
            else:
                l = numpy.array([goal.min], dtype = 'uint8')
                u = numpy.array([goal.max], dtype = 'uint8')

            mask = cv2.inRange(cv_image, l, u)
            sum = cv2.countNonZero(mask)
            print 'Amount of the pixes in duration %d-%d is %d.' %(goal.min, goal.max, sum)
            print "Sum of range is %d" %sum

        except CvBridgeError, e:
            print e


        try:
            rtn = ProcessResult()
            rtn.sum = sum
            self.action_server.set_succeeded(rtn)
        except Exception as e:
            print e
            #self.action_server.set_failled()


if __name__ == '__main__':
    try:
        rospy.init_node("part2")
        server = ProcessImageServer()
        rospy.spin()
    except Exception as e:
        print e


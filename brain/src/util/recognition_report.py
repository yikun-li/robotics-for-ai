#!/usr/bin/env python

import random
import time
import os.path
import subprocess

import cv2 as cv
#import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy 

import rospy

latex_header = """
\\documentclass[12pt]{article}
\\title{Recognition Report}
\\author{The BORG Team}
\\date{\\today}
\\usepackage{graphicx}
\\begin{document}
\\maketitle
"""

latex_footer = "\\end{document}"

class RecognitionReport(object):
    
    def __init__(self, rgb_topic = None, start_time = None, tmp_location = "/tmp/recognition_report/"):
        self.__item_list = []
        self.__tmp_location = tmp_location 
        if not start_time:
            self.__start_time = time.time()
        #Reset temporary location to produce pdf:
        if os.path.exists(self.__tmp_location):
            self.execute("rm -r %s" % self.__tmp_location)
        self.execute("mkdir -p %s/figs" % self.__tmp_location)
        #Subscribe to rgb topic for the purpose of annotation:
        if rgb_topic:
            self.__image_sub = rospy.Subscriber(rgb_topic, Image, self.image_callback)
            self.__bridge = CvBridge()
        self.__last_img = None
        self.__last_img_time = 0.0

    def image_callback(self, ros_image):
        try:
            img = self.__bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        #np_img = numpy.array(img, dtype = numpy.uint8)
        self.__last_img = img
        self.__last_img_time = rospy.get_time()

    def execute(self, cmd):
        try:
            subprocess.Popen(cmd, shell = True).wait()
        except:
            print "Command \"%s\" failed!" % cmd
       
    def add_item(self, name, bounding_box):
        if bounding_box[0][0] >= bounding_box[1][0]:
            rospy.logwarn("Top left corner is on the right of bottom right corner, not adding image!")
            return False
        if bounding_box[0][1] >= bounding_box[1][1]:
            rospy.logwarn("Top left corner is lower than the bottom right corner, not adding image!")
            return False
        item = {"name":name, "bounding_box":bounding_box, "image":None}
        #Combine with latest rgb image (if it exists):
        if self.__last_img == None:
            rospy.logwarn("No RGB image available to put in the report!")
        else:
            time_diff = rospy.get_time() - self.__last_img_time
            if time_diff > 1.0:
                rospy.logwarn("RGB image available, but it is very old (%f seconds)!" % time_diff)
            item["image"] = self.__last_img
            #Check max/min of bounding box!
            width, height, depth = item["image"].shape
            if bounding_box[1][0] > width:
                rospy.logwarn("Bounding box exceeds the limits of the width of the image, not adding image!")
                item["image"] = None
            if bounding_box[1][1] > height:
                rospy.logwarn("Bounding box exceeds the limits of the height of the image, not adding image!")
                item["image"] = None
        self.__item_list.append(item)
        return True

    def produce_latex(self, end_time = None):
        if not end_time:
            end_time = time.time()
        #Header:
        latex = latex_header

        #Summary of all recognized objects:
        latex += "\\section{Summary}\n"
        latex += "\\begin{enumerate}\n"
        latex += "    \\item[\\textbf{Start Time:}] %s\n" % time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.__start_time))
        latex += "    \\item[\\textbf{Duration:}] %.2f minutes\n" % round((end_time - self.__start_time) / 60, 2)
        latex += "    \\item[\\textbf{No. of objects:}] %d\n" % len(self.__item_list)
        latex += "\\end{enumerate}\n"

        #Section with list of recognized objects:
        latex +=  "\\section{Recognized Objects}\n"
        for idx, item in enumerate(self.__item_list):
            latex += "    \\subsection{%s}\n" % item["name"]
            if not item["image"] == None:
                #Draw bounding box on figure:
                cv2.rectangle(item["image"], item["bounding_box"][0], item["bounding_box"][1], (255, 255, 0), thickness = 2)
                #Save figure to temp dir:
                cv2.imwrite("%s/figs/item_%d.png" % (self.__tmp_location, idx), item["image"])
                #Add figure in latex:
                latex += "\\begin{figure}[h]"
                latex += "\\centering"
                latex += "\\includegraphics[width=0.45\\textwidth]{figs/item_%d.png}" % idx
                latex += "\\caption{Object ``%s''.}" % item["name"]
                latex += "\\label{fig:item_%d}" % idx
                latex += "\\end{figure}"
            else:
                latex += "Unable to process image, sorry, see log for details."

        #Footer:
        latex += latex_footer
        return latex 

    def produce_pdf(self, destination = None, title = None):
        if not title:
            title = time.strftime("%Y-%m-%d_%H%M-%S", time.localtime(time.time()))
        if not destination:
            destination = "/tmp"
        #Store latex in tmp dir:
        texfile = open("%s/%s.tex" % (self.__tmp_location, title), 'w')
        texfile.write(self.produce_latex())
        texfile.close()
        #Produce actual PDF:
        self.execute("cd %s && pdflatex %s" % (self.__tmp_location, "%s.tex" % title))
        self.execute("cd %s && pdflatex %s" % (self.__tmp_location, "%s.tex" % title))
        #Copy to final location:
        self.execute("cp %s/%s.pdf %s" % (self.__tmp_location, title, destination))

        
if __name__ == "__main__":
    rospy.init_node('recognition_report_test', anonymous=True)

    report = RecognitionReport(rgb_topic = "/platform/camera2/rgb/image_raw")

    items_to_add = [
        {"interval":0.5, "name":"cola", "bounding_box":((15, 5), (100, 200))},
        {"interval":0.5, "name":"beer", "bounding_box":((0, 0), (10, 10))},
        {"interval":0.5, "name":"cake", "bounding_box":((50, 50), (100, 70))},
        {"interval":0.5, "name":"cake", "bounding_box":((50, 50), (1000, 70))},
    ]
    item_idx = 0

    last_add_time = rospy.get_time()
    while not rospy.is_shutdown():
        if (rospy.get_time() - last_add_time) > items_to_add[item_idx]["interval"]:
            report.add_item(items_to_add[item_idx]["name"], items_to_add[item_idx]["bounding_box"])
            last_add_time = rospy.get_time()
            item_idx += 1
            if item_idx >= len(items_to_add):
                break;
        
    report.produce_latex()
    report.produce_pdf()

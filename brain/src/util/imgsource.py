# To change this template, choose Tools | Templates
# and open the template in the editor.

__author__="hcvhoof"
__date__ ="$Nov 19, 2010 3:50:36 PM$"

import cv
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Util.ImgSource').addHandler(util.nullhandler.NullHandler())

class ImgSource(object):
    """Can be used as a video stream to test vision modules"""
    def __init__(self, filename):
        """
        creates the image source that will load the image in filename to pass
        through to the vision module
        """
        self.logger = logging.getLogger('Borg.Brain.Util.ImgSource')
        self.filename = filename

    def set_image_file(self, filename):
        """
        Set a new file to be loaded and passed the next time get_image is called
        """
        self.filename = filename

    def get_image(self):
        """
        Load the image and pass it to the vision module
        """
        return cv.LoadImage(self.filename)

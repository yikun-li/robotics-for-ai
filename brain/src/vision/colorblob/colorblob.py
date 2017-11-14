#!/usr/bin/python


import pygame
import time
import cv
import pickle
import sys

import glob 
import os 


def convert_cv_to_pg(cv_im):
    src_im = cv.CreateMat(cv_im.height, cv_im.width, cv.CV_8UC3)
    cv.CvtColor(cv_im, src_im, cv.CV_BGR2RGB)
    return pygame.image.frombuffer(src_im.tostring(), cv.GetSize(src_im), "RGB")


class ColorBlob(object):
    """
    A colorblob object holds a list of colorcubes and uses the detect function to detect 
    both the size and location of the colorblob.
    """

    def __init__(self, colorlist):
        self.__colorlist = colorlist
        self.__cube_size = colorlist.get_cube_size()

        self.__final_surface = None
        self.__temp_surface = None
        self.__centroid = None
        self.__size = None


    def detect(self, surface, cube_size = None):
        if cube_size == None:
            cube_size = self.__cube_size

        if self.__final_surface == None:
            self.__final_surface = pygame.Surface((surface.get_width(), surface.get_height()))
        if self.__temp_surface == None:
            self.__temp_surface = pygame.Surface((surface.get_width(), surface.get_height()))

        self.__final_surface.fill((0, 0, 0, 255))

        #TODO: Use only mask.from_threshold instead of both mask.from_threshold and transform.threshold (will speed up the detection):
        #Interate of the color (cube) list and sum the resulting thresholded image to the final thresholded image: 
        threshold = (cube_size, cube_size, cube_size)
        diff_color = (0, 0, 0, 255)
        for color in self.__colorlist.get_list():
            pygame.transform.threshold(self.__temp_surface, surface, color, threshold, diff_color, 2)
            self.__final_surface.blit(self.__temp_surface, (0, 0), None, pygame.BLEND_ADD)

        #Convert the thresholded image to a mask:
        mask = pygame.mask.from_threshold(self.__final_surface, (0, 0, 0), (1, 1, 1, 255))
        mask.invert()

        self.__centroid = mask.centroid()
        #TODO: Detect and bounding rectangle around blob instead of just the sum of all pixels (put ignore noise!):
        #Relative size of the blob:
        self.__size = float(mask.count()) / (surface.get_width() * surface.get_height())


    def get_centroid(self):
        if self.__centroid == None:
            raise Exception("Improper usage; call detect() first!")
        return self.__centroid


    def get_size(self):
        if self.__size == None:
            raise Exception("Improper usage; call detect() first!")
        return self.__size


    def get_final_surface(self):
        if self.__final_surface == None:
            raise Exception("Improper usage; call detect() first!")
        return self.__final_surface


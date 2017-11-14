#!/usr/bin/python


import pygame
import time
import cv
import pickle
import sys

import glob 
import os 

import math

def load_color_file_list(location):
    if not os.path.exists(location):
        raise Exception("Sorry, but that location does not exist! (%s)" % location)
    color_file_list = []
    for color_file_location in glob.glob(os.path.join(location + "/*.colorblob")):
        print "About to import colorblob file: %s" % color_file_location
        color_file_list.append(ColorList(color_file_location))
    if len(color_file_list) == 0:
        raise Exception("Sorry, but no *.colorblob files were found!")
    return color_file_list


class ColorList(object):
    
    def __init__(self, color_file = None, cube_size = 3):
        self.__color_list = []
        self.__cube_size = cube_size

        self.__color_file = color_file
        if not color_file == None:
            self.load(color_file)


    def get_list(self):
        return self.__color_list


    def get_file_location(self):
        return self.__color_file


    def get_file_basename(self):
        return os.path.basename(self.__color_file)


    def load(self, color_file):
        #Create an empty list for an empty file, unpickle otherwise:
        if os.path.getsize(color_file) == 0:
            self.__color_list = [] 
        else:
            f = open(color_file, 'r')
            data = pickle.load(f)
            self.__cube_size = data[0]
            self.__color_list = data[1]
            f.close()


    def save(self, color_file = None):
        if color_file == None:
            if self.__color_file == None:
                raise Exception("There is no location specified to save to!")
            color_file = self.__color_file
        f = open(color_file, 'w')
        data = [self.__cube_size, self.__color_list]
        pickle.dump(data, f)
        f.close()


    def is_in_cube_list(self, current_color, cube_size):
        for color in self.__color_list:
            if current_color[0] > (color[0] - cube_size) \
                    and current_color[0] < (color[0] + cube_size) \
                    and current_color[1] > (color[1] - cube_size) \
                    and current_color[1] < (color[1] + cube_size) \
                    and current_color[2] > (color[2] - cube_size) \
                    and current_color[2] < (color[2] + cube_size):
                return True
        return False


    def remove_last(self):
        if len(self.__color_list) > 0:
            self.__color_list.pop()


    def set_cube_size(self, cube_size):
        if cube_size > 255:
            cube_size = 255
        if cube_size < 1:
            cube_size = 0
        self.__cube_size = cube_size


    def get_cube_size(self):
        return self.__cube_size


    def filter_outliers(self):
        #TODO: Inefficient code, should be improved:
        if len(self.__color_list) < 10:
            return
        no_outliers = int(len(self.__color_list) * 0.2)
        #Calculate the average:
        sum_color = [0, 0, 0, 0]
        for color in self.__color_list:
            sum_color[0] += color[0]
            sum_color[1] += color[1]
            sum_color[2] += color[2]
        sum_color[0] /= len(self.__color_list)
        sum_color[1] /= len(self.__color_list)
        sum_color[2] /= len(self.__color_list)
        mean_color = sum_color
        #Calculate distance to mean for every color:
        color_distance_list = []
        for color in self.__color_list:
            #Distance:
            summ = pow(mean_color[0] - color[0], 2)
            summ += pow(mean_color[1] - color[1], 2)
            summ += pow(mean_color[2] - color[2], 2)
            distance = math.sqrt(summ)
            color_distance_list.append((distance, color))
        #Sort:
        color_distance_list = sorted(color_distance_list, key=lambda color_distance: color_distance[0])   
        #Remove last few:
        for i in range(no_outliers):
            color_distance_list.pop()
        #Create new list:
        self.__color_list = []
        for color in color_distance_list:
            self.__color_list.append(color[1])


    def add_color(self, color):
        if not self.is_in_cube_list(color, self.__cube_size):
            self.__color_list.append(color)
            return True
        return False


    def empty_list(self):
        self.__color_list = []




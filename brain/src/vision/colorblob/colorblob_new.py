#!/usr/bin/python


import pygame
import time
import cv2
import pickle
import sys
import numpy
import copy

import glob 
import os

########################################################################
### NEW STUFF!
###

def process_HSV(image, to_be_detected, denoise_value):
    '''
    This will process each image in HSV space, trying to detect blobs of the colors in to_be_detected
    image is an openCV HSV image.
    to_be_detected a list of dicts.
    the dicts contain 'h_lower', 'h_upper', 's_lower', 's_upper', 'v_lower', 'v_upper'.
    '''
    
    # Split channels
    (hue, saturation, value) = cv2.split(image)
    
    # Next, find blobs of each color.
    
    # Set up an empty image to merge with
    merge_image = None
    
    if len(to_be_detected) < 1:
        return None
    
    # For each color cube:
    for cube in to_be_detected:
    
        # Filter all three channels
        # Filter Hue
        (_,hue_1) = cv2.threshold(hue, cube['h_lower'],1,cv2.THRESH_BINARY)
        (_,hue_2) = cv2.threshold(hue, cube['h_upper'],1,cv2.THRESH_BINARY_INV)
        # Combine filtered images
        # Hue range may wrap around 0.
        if cube['h_upper']<cube['h_lower']:
            hue_combined = hue_1 | hue_2
            #hue_combined = hue_1 or hue_2
        else:
            hue_combined = hue_1 & hue_2
            #hue_combined = hue_1 and hue_2
                
        # Filter Saturation
        (_,saturation_1) = cv2.threshold(hue, cube['s_lower'],1,cv2.THRESH_BINARY)
        (_,saturation_2) = cv2.threshold(hue, cube['s_upper'],1,cv2.THRESH_BINARY_INV)
        # Combine filtered images
        saturation_combined = saturation_1 | saturation_2
        
        # Filter Value
        (_,value_1) = cv2.threshold(hue, cube['v_lower'],1,cv2.THRESH_BINARY)
        (_,value_2) = cv2.threshold(hue, cube['v_upper'],1,cv2.THRESH_BINARY_INV)
        # Combine filtered images
        value_combined = value_1 | value_2
         
        # Combine combined filtered images
        filtered = (hue_combined & saturation_combined) & value_combined
        
        # Add filtered to the combined image
        if merge_image is None:
            merge_image = copy.deepcopy(filtered)
        else:
            merge_image = (merge_image | filtered)
    
    
    
    # Remove noise through a closing
    if not denoise_value == 0:
        denoised = cv2.morphologyEx(merge_image, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (denoise_value,denoise_value)))
        denoised = cv2.morphologyEx(denoised, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (denoise_value,denoise_value)))
    else:
        denoised = merge_image
    
    # Detect blobs
    found = cv2.findContours(denoised, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if found:
        contours, hierarchy = found
    
        # Return the blobs
        return contours
    else:
        return None

#!/usr/bin/env python

'''
Created by Noel Luneburg, 2013.
See docs/util/annotator.tex for documentation.
'''

import os
from os.path import expanduser
import sys
from operator import itemgetter
import ast
import random
import time
import math
import copy
import pygame
import numpy as np
from pygame.locals import *
from PIL import Image

class Main(object):
    def init(self, map_location, yaml_location, objects_location, save_location, load_location, mode):
        self.MAX_FPS = 30 # The maximum (probably standard) framerate
        
        # Initialising the colours used
        self.GREEN = (0,255,0)
        self.BLUE = (0,0,255)
        self.YELLOW = (255,255,0)
        self.RED = (255,0,0)
        self.BLACK = (0,0,0)
        self.GREY = (100,100,100)
        self.BACKGROUND = self.BLACK
        self.C_KEY = (255,0,255) # Colour key for transparency in images
        
        # Constant names
        self.CREATE_NEW = 0
        self.CREATING = 5
        self.SELECTED = 1
        self.POINTS = 2
        self.NAME = 6
        self.HEIGHT = 3
        self.OBJECTS = 4
        self.MENU = 7
        
        # Currently selected annotation
        self.current_idx = None
        
        # Input event states
        self.mouse = {'l_down': -1, 'l_up': -1, 'r_down': -1, 'r_up': -1, 'pos': (0,0)}
        self.key = None
        # Scrolling trackers
        self.scroll_object = 0
        self.scroll_annot = 0 # Unused for now
        
        # State variables
        self.annotate_mode = 'regions' # default mode
        if mode in ['regions', 'locations']:
            self.annotate_mode = mode # Indicates which mode should be used, single points or polygons
        self.mode = self.CREATE_NEW # indicates the state of the program
        self.rotating = False # Whether the mouse is held down to define an orientation
        self.property = None # Used as indication of which property of annotation is being edited
        
        # Other variables
        self.direction_length = 20 # How big the indicator for the orientation should be
        
        # Annotations dicts with the following properties:
        # name = The name of the annotation
        # points = List of tuples of (X,Y)-coordinates of all the polygon vertices
        # height = Height of the annotation in centimeters
        # objects = The objects that can occur in this annotation and their chance of occurrence in the format: {'coke': 3}
        # colour = The colour of the annotation that is displayed in the tool
        # transform = Whether or not the points of the polygon should be transformed when saving
        self.annotations = []
        # Locations dicts with the following properties:
        # name = The name of the location
        # point = The point on the map
        # angle = The orientation in degrees
        # transform = Whether or not the point of the location should be transformed to real world coordinates when saving
        self.locations = []
        # List of objects
        self.objects = []
        # List of original points stored for loaded annotations to avoid rounding errors
        self.original_points = []
     
	# Sets the positions of all the UI images
    def set_UI_positions(self, start_x, screen_w, screen_h):
        # Positions of UI elements
        self.obj_sX = start_x + (screen_w - self.object_row.get_size()[0])/2 # object rows
        self.obj_sY = 35
        self.name_sX = start_x + (screen_w - self.name_field.get_size()[0])/2 # name input field
        self.name_sY = screen_h - 64
        self.instr_del_sX = start_x + (screen_w - self.instruction_field.get_size()[0])/2 # delete annotation instruction field
        self.instr_del_sY = screen_h - 64
        self.instr_enter_sX = start_x + (screen_w - self.instruction_field.get_size()[0])/2 # edit annotation instruction field
        self.instr_enter_sY = screen_h - 32
        self.height_sX = start_x + (screen_w - self.name_field.get_size()[0])/2 # height input field
        self.height_sY = screen_h - 32
        self.menu_sX = start_x + 1 # menu buttons
        self.menu_sY = 1
        self.key_sX = 12 # key list
        self.key_sY = 8
        self.key_spacing = 15 # Space between annotations in key
    
	# Makes sure the entered locations are an actual readable path
    def rectify_locations(self, map, yaml, obj, save, load):
        map = str(map)
        yaml = str(yaml)
        obj = str(obj)
        save = str(save)
        load = str(load)
        home = expanduser('~')
        
        # Check if mandatory parameters are there
        if map == '':
            print '[rectify_locations] Map location was not entered correctly'
            sys.exit()
        if (yaml == '' or yaml == 'None') and self.annotate_mode == 'locations':
            print '[rectify_locations] Yaml file locations was not entered correctly'
            sys.exit()
        if (obj == '' or obj == 'None') and self.annotate_mode == 'regions':
            print '[rectify_locations] Objects location was not entered correctly'
            sys.exit()
        if save == '':
            print '[rectify_locations] Save location was not entered correctly'
            sys.exit()
        
        '''# Strip /
        if map[0] == '/':
            map = map[1:]
        if obj[0] == '/':
            obj = obj[1:]
        if save[0] == '/':
            save = save[1:]
        if not load == '' and load[0] == '/':
            load = load[1:]'''
            
        if map[0] == '~':
            map = map[1:] # Strip ~
            map = map[::-1] # Reverse name
            map += home[::-1] # Add home directory
            map = map[::-1] # Reverse back to original
        if yaml == '' or yaml == None:
            yaml = None
        elif yaml[0] == '~':
            yaml = yaml[1:]
            yaml = yaml[::-1]
            yaml += yaml[::-1]
            yaml = yaml[::-1]
        if obj[0] == '~':
            obj = obj[1:]
            obj = obj[::-1]
            obj += home[::-1]
            obj = obj[::-1]
        if save[0] == '~':
            save = save[1:]
            save = save[::-1]
            save += home[::-1]
            save = save[::-1]
        if load == '' or load == 'None':
            load = None
        elif load[0] == '~':
            load = load[1:]
            load = load[::-1]
            load += load[::-1]
            load = load[::-1]
                        
        return (map,yaml,obj,save,load)
	
	# Moves the points of a polygon vertically down by a specified amount
    def shift_down(self, polygon, amount):
        for i,(x,y) in enumerate(polygon):
            y += amount
            polygon[i] = (x,y)

        return polygon

    # Sort the annotations by a specific key in the dicts
    def sort_annotations(self, value, annotations):
        new_points = []
        old_annotations = copy.deepcopy(annotations)
        annotations = sorted(annotations, key=itemgetter(value))
        # Sort the original_points list according to how the annotations were sorted
        for i,an in enumerate(annotations):
            idx = 0
            for j,old in enumerate(annotations):
                if old_annotations[j] == annotations[i]:
                    idx = j
            new_points.append(self.original_points[idx])
            
        self.original_points = copy.deepcopy(new_points)
        
    # Convert map pixel point to real world location      
    def convert_point_pixel_to_real(self, p, res, o):
        p = p[0]
        point = (p[0]*res, p[1]*res)
        point = (point[0]+o[0], (point[1]+o[1]))
            
        return point
        
    # Convert real world location to map pixel point   
    def convert_point_real_to_pixel(self, p, res, o):
        p = p[0]
        point = (p[0]-o[0], (p[1]-o[1]))
        point = (point[0]/res, point[1]/res)
            
        return point

	# Scales and transforms the points of annotations to original dimension or to dimensions used in the tool
    def adjust_annotations(self, annotations, original=True):
        if original:
            scale = 1.0 / self.scale
            x_off = self.map_region.x
            y_off = self.map_region.y
        else:
            scale = self.scale
            x_off = -self.map_region.x
            y_off = -self.map_region.y

        new = []        
        # Copy all annotations into a new dict and offset the coordinates according to format     
        for i, annot in enumerate(annotations):
            new.append(copy.deepcopy(annot))
            if self.annotate_mode == "regions":
                for j, point in enumerate(annot['points']):
                    if original:
                        # When saving, check if points need to be transformed
                        if annot['transform']:
                            new[i]['points'][j] = (new[i]['points'][j][0] - self.screen_x_start,new[i]['points'][j][1])
                            new[i]['points'][j] = (new[i]['points'][j][0] * scale,new[i]['points'][j][1] * scale)
                            new[i]['points'][j] = (new[i]['points'][j][0] + x_off,new[i]['points'][j][1] + y_off)
                            new[i]['points'][j] = (int(new[i]['points'][j][0]),int(new[i]['points'][j][1]))
                        else:
                            new[i]['points'][j] = self.original_points[i][j]
                    else:
                        new[i]['points'][j] = (new[i]['points'][j][0] + x_off,new[i]['points'][j][1] + y_off)
                        new[i]['points'][j] = (new[i]['points'][j][0] * scale,new[i]['points'][j][1] * scale)
                        new[i]['points'][j] = (int(new[i]['points'][j][0]),int(new[i]['points'][j][1]))
                        new[i]['points'][j] = (new[i]['points'][j][0] + self.screen_x_start,new[i]['points'][j][1])
            else:
                (res,o) = self.load_yaml(self.yaml_location)
                if original:
                    # When saving, check if points need to be transformed
                    if annot['transform']:
                        new[i]['point'][0] = (new[i]['point'][0][0] - self.screen_x_start, new[i]['point'][0][1])
                        new[i]['point'][0] = (new[i]['point'][0][0] * scale, new[i]['point'][0][1] * scale)
                        new[i]['point'][0] = (new[i]['point'][0][0] + x_off, new[i]['point'][0][1] + y_off)
                        new[i]['point'][0] = (int(new[i]['point'][0][0]), int(new[i]['point'][0][1]))
                        new[i]['point'] = [self.convert_point_pixel_to_real(new[i]['point'], res, o)]
                    else:
                        new[i]['point'][0] = self.original_points[i][0]
                else:
                    new[i]['point'] = [self.convert_point_real_to_pixel(new[i]['point'], res, o)]
                    new[i]['point'][0] = (new[i]['point'][0][0] + x_off, new[i]['point'][0][1] + y_off)
                    new[i]['point'][0] = (new[i]['point'][0][0] * scale, new[i]['point'][0][1] * scale)
                    new[i]['point'][0] = (int(new[i]['point'][0][0]),int(new[i]['point'][0][1]))
                    new[i]['point'][0] = (new[i]['point'][0][0] + self.screen_x_start,new[i]['point'][0][1])
                    
        return new
    
	# Saves the annotations to a specified file
    def export_annotations(self):
        try:
            f = open(self.save_location, 'w')
        except:
            print '[export_annotations] Could not open save location: %s' % self.save_location
            return
        transformed_annotations = None
        
        if self.annotate_mode == "regions":            
            transformed_annotations = self.adjust_annotations(self.annotations)
            for a in transformed_annotations:
                f.write('# name\n') 
                f.write(str(a['name']) + '\n')
                f.write('# colour\n')
                f.write(str(a['colour']) + '\n')
                f.write('# height\n')
                f.write(str(a['height']) + '\n')
                f.write('# points\n')
                f.write(str(a['points']) + '\n')
                f.write('# objects\n')
                f.write(str(a['objects']) + '\n')
                f.write('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
            f.write('# end')
        else:
            f.write('name,x,y,angle\n') # standard line for each locations file
            transformed_annotations = self.adjust_annotations(self.locations)
            for a in transformed_annotations:
                f.write(str(a['name']) + ',')
                f.write(str(a['point'][0][0]) + ',')
                f.write(str(a['point'][0][1]) + ',')
                f.write(str(a['angle']) + '\n')
            
        f.close()
    
    # Loads the annotations from a file
    def load_annotations(self, load):
        try:
            f = open(load,'r')
        except:
            print '[load_annotations] Could not load from location: %s' % load
            sys.exit()
            
        if self.annotate_mode == "regions":
            temp_annots = [] # List of the annotations
            temp_annot = {'name': '', 'points': [], 'height': 0, 'objects': {}, 'colour': (0,0,0), 'transform': False} # Prototype of annotation
            f.seek(0)
            next = None
            for line in f:
                if next == 'name':
                    temp = []
                    for c in line:
                        if c == '\n':
                            break
                        else:
                            temp.append(c)
                    temp_annot['name'] = ''.join(temp)
                    next = None
                elif next == 'colour':
                    temp_annot['colour'] = ast.literal_eval(line)
                    next = None
                elif next == 'height':
                    temp_annot['height'] = int(line)
                    next = None
                elif next == 'points':
                    self.original_points.append(ast.literal_eval(line)) # Store the points in original format when loading
                    temp_annot['points'] = ast.literal_eval(line)
                    next = None
                elif next == 'objects':
                    temp_annot['objects'] = ast.literal_eval(line)
                    next = None
                elif '# name' in line:
                    next = 'name'
                elif '# colour' in line:
                    next = 'colour'
                elif '# height' in line:
                    next = 'height'
                elif '# points' in line:
                    next = 'points'
                elif '# objects' in line:
                    next = 'objects'
                elif '~~~' in line:
                    temp_annots.append(copy.deepcopy(temp_annot))
                    next = None
                elif '# end' in line:
                    next = None
                else:
                    print '[load_annotations] unrecognisable format in load file'
                    sys.exit()
        else:
            temp_annots = [] # List of the locations
            temp_annot = {'name': '', 'point': [], 'angle': 0, 'colour': (255,0,0), 'transform': False} # Prototype of annotation
            f.seek(0)
            next = None
            for line in f:
                temp_annot['point'] = [] # clear point data
                l = line
                
                if 'name,x,y,angle' in l or l == '\n':
                    # standard first line or empty lines can be ignored
                    continue
                
                # name
                idx = l.find(',')
                temp_annot['name'] = l[:idx]
                l = l[idx+1:]
                # point x
                idx = l.find(',')
                x = ast.literal_eval(l[:idx])
                l = l[idx+1:]
                # point y
                idx = l.find(',')
                y = ast.literal_eval(l[:idx])
                l = l[idx+1:]
                temp_annot['point'].append((x,y))
                self.original_points.append([(x,y)])
                # orientation
                idx = l.find(',')
                temp_annot['angle'] = ast.literal_eval(l[:idx])                
                # end of line
                temp_annots.append(copy.deepcopy(temp_annot))

        f.close()
                
        temp_annots = self.adjust_annotations(temp_annots, False)
        return temp_annots
        
    # Reads the resolution and origin from a .yaml file and returns these
    def load_yaml(self, file_name):
        try:
            f = open(file_name)
        except:
            print '[read_yaml] Cannot open file: %s' % file_name
        res = None
        origin = None
        for line in f:
            test = line.find('resolution: ')
            if test > -1:
                res = ast.literal_eval(line[test+len('resolution: '):])
            test = line.find('origin: ')
            if test > -1:
                origin = ast.literal_eval(line[test+len('origin: '):])
                          
        return (res,tuple(origin))
    
	# Resets the mouse so that an input that lasts for more than one frame will not be active more than once
    def reset_mouse(self):
        self.mouse['l_down'] = -1 
        self.mouse['l_up'] = -1
        self.mouse['r_down'] = -1
        self.mouse['r_up'] = -1
        
    # Specifies whether or not the mouse pointer is within the map region of the screen
    def mouse_on_map(self,pos):
        if pos[0] >= self.screen_x_start:
            return True
       
        return False
    
	# Takes in key events and returns the corresponding character
    def convert_key_to_char(self,key):
        if key >= K_a and key <= K_z: 
            character = '%c' % key
            return character
        elif key >= K_0 and key <= K_9:
            digit = key - K_0
            return str(digit)
        elif key >= K_KP0 and key <= K_KP9:
            digit = key - K_KP0
            return str(digit)
        elif key == K_UNDERSCORE or key == K_MINUS:
            return '_'
        elif key == K_PERIOD or key == K_KP_PERIOD:
            return '.'
        else:
            return None
    
	# Takes in key events and returns the corresponding numbers
    def convert_key_to_int(self, key):
        if key >= K_0 and key <= K_9:
            digit = key - K_0
            return digit
        elif key >= K_KP0 and key <= K_KP9:
            digit = key - K_KP0
            return digit
        else:
            return None
            
    # Covert radians to degrees
    def rad_to_deg(self, rad):
        return (int)(rad * (180/math.pi))
        
    # Convert degrees to radians
    def deg_to_rad(self, deg):
        return deg * (math.pi/180)
        
    # Limits an angle to a certain range
    def cap_angle(self, value, min_val, max_val):
        angle = value
        if value < min_val:
            angle = angle + (abs(min_val) + abs(max_val))
        elif value > max_val:
            angle = angle - (abs(min_val) + abs(max_val))
        
        return angle
            
    # Returns the selected annotation in the key list, None if there is nothing selected
    # Box is a 4-point polygon in point order:
    # 0   1
    # 3   2       
    def get_key_entry(self, pos, box):
        selected = None
        if pos[0] >= box[0][0] and pos[0] < box[1][0]: # mouse is inside x-values
            height = box[3][1] - box[0][1] # height of a polygon
            space = self.key_spacing - height
            offset_y = pos[1] - self.key_sY # transform mouse y to top of the screen
            index = offset_y / (height + space) # int value of section selected
            if offset_y < (index*(height+space) + height): # filter out when pointer is inside a spacing (selected nothing)
                selected = index
                
        return selected
    
	# Returns the place of a newly created annotation in the list
    def get_empty_idx(self, annotations):
        for i in range(len(annotations)):
            if annotations[i]['name'] == '*new*':
                return i
                
    # Returns row and column in which the mouse cursor is positioned inside the object rows
    # Boundaries contains the region in which the object rows are positioned
    def get_position(self,m,boundaries,dimensions):
        obj = ((m[1]-boundaries[0][1])/dimensions[1]) + self.scroll_object*25 # find the object that is being pointed at
        prob = (m[0]-boundaries[0][0])/dimensions[0] # find the probability that is being pointed at
        return (obj,3-prob) # Probability value is inversed on the x-axis and in range [0,3]
    
    # Horizontal intersecting rays method to check if a target point lies within a polygon, see:
	# http://alienryderflex.com/polygon/
    def point_in_polygon(self,points,target):
        j = len(points)-1
        odd = False
        
        for i in range(len(points)):
            if (points[i][1] < target[1] and points[j][1] >= target[1]) or \
               (points[j][1] < target[1] and points[i][1] >= target[1]):
                if points[i][0] + (float(target[1])-points[i][1])/(points[j][1]-points[i][1])*(points[j][0]-points[i][0]) < target[0]:
                    odd = not odd
            j = i
            
        if odd:
            return True
        else:
            return False
    
	# Finds out which, if any, polygon is under the current mouse position
    def get_polygon_idx(self,m):
        index = None
        for i in range(len(self.annotations)):
            if self.point_in_polygon(self.annotations[i]['points'],m):
                index = i
        return index
    
	# Crop and zoom the map to reduce it to the useful part
    def transform_image(self,map_image,pixels,im):
        pygame_pixels = pygame.surfarray.array2d(map_image)
        start_time = time.time()
        bg_val = pixels[0,0] # Consider the first pixel as background value
        background = bg_val - 50 # make the background colour a bit darker than the map's background
        if background < 0:
            background = 0
        self.BACKGROUND = (background,background,background)
        
        img = np.array(im)
        back_idx = np.argwhere(img != bg_val)
        
        # The outer points of the map itself
        top = back_idx[np.argwhere(back_idx[:,0] == np.min(back_idx[:,0]))] # First row containing the map pixels
        top = top[0,0,:]
        top = tuple(top[::-1]) # Changing format from (Y,X) to (X,Y)
        left = back_idx[np.argwhere(back_idx[:,1] == np.min(back_idx[:,1]))] # First column containing the map pixels
        left = left[0,0,:]
        left = tuple(left[::-1])
        right = back_idx[np.argwhere(back_idx[:,1] == np.max(back_idx[:,1]))] # Last column containing the map pixels
        right = right[0,0,:]
        right = tuple(right[::-1])
        bottom = back_idx[np.argwhere(back_idx[:,0] == np.max(back_idx[:,0]))] # Last row containing the map pixels
        bottom = bottom[0,0,:]
        bottom = tuple(bottom[::-1])
        
        time_taken = time.time() - start_time
        print 'Time to load and transform image: %f' % time_taken
        
        cropped_width = (right[0]-left[0])
        cropped_height = (bottom[1]-top[1])
        # Region in which the map exists
        self.map_region = pygame.Rect(left[0],top[1],cropped_width,cropped_height)
        self.scale = 1.0 / max(cropped_width/float(self.screen_width),float(cropped_height)/self.screen_height)
        
        cropped_image = pygame.Surface((cropped_width,cropped_height))
        cropped_image.blit(map_image,(0,0),self.map_region)
        
        # Scale the image to fit the screen
        return pygame.transform.scale(cropped_image,(int(cropped_width*self.scale),int(cropped_height*self.scale)))
	
	# Generates a random RGB colour tuple
    def generate_colour(self):
        return (random.randint(30,245),random.randint(30,245),random.randint(30,245))
    
	# Gives the newest annotation a random colour
    def set_new_colour(self, annotations):
        for i in range(len(annotations)):
            if annotations[i]['name'] == '*new*':
                annotations[i]['colour'] = self.generate_colour()
                
    # Adds the orientation to the location, relative to the location's position
    def store_orientation(self, pos, annotations):
        idx = self.get_empty_idx(annotations)
        # Compute angle
        deltaX = pos[0] - annotations[idx]['point'][0][0]
        deltaY = pos[1] - annotations[idx]['point'][0][1]
        angle = self.rad_to_deg(math.atan2(-deltaY,deltaX)) - 90 # - 90 to make facing upwards return 0
        angle = self.cap_angle(angle, -180, 180)
        annotations[idx]['angle'] = angle

	# Adds the current mouse position as a polygon point to the newest annotation
    def add_point(self, pos, annotations):
        points = 'points'
        if self.annotate_mode == 'locations':
            points = 'point'    
    
        # Add point to current annotation being created
        for i in range(len(annotations)):
            if annotations[i]['name'] == '*new*':
                annotations[i][points].append(pos)
                
	# Removes the last point from the newest annotation 
    def remove_last_point(self, annotations):
        points = 'points'
        if self.annotate_mode == 'locations':
            points = 'point'

        for i in range(len(annotations)):
            if annotations[i]['name'] == '*new*':
                if len(annotations[i][points]) > 0:
                    annotations[i][points].pop()
     
	# Returns whether there are no points associated with the newest annotation and removes that from the list
    def no_points(self, annotations):
        points = 'points'
        if self.annotate_mode == 'locations':
            points = 'point'
    
        for i in range(len(annotations)):
            if annotations[i]['name'] == '*new*':
                if not annotations[i][points]:
                    annotations.remove(annotations[i]) # remove empty annotations
                    self.original_points.remove(self.original_points[i])
                    return True
                else:
                    return False
                  
	# Handles the naming of a specific annotation
    def name_annotation(self, idx, annotations):
        # Get the character array of the corresponding name
        name = annotations[idx]['name']
        if name == '*new*':
            name = ''
        
        if not self.key == None:
            item = self.convert_key_to_char(self.key)
            if not item == None:
                name += item
            if self.key == K_BACKSPACE and not name == '':
                name = name[:(len(name)-1)] # remove last character
                
        annotations[idx]['name'] = name
        
	# Handles the editing of a specific annotation's height
    def change_height(self, idx, annotations):
        height = annotations[idx]['height']
        height_str = str(annotations[idx]['height'])
        if height == 0:
            height_str = ''
        
        if not self.key == None:
            item = self.convert_key_to_int(self.key)
            if not item == None:
                height_str += str(item)
            if self.key == K_BACKSPACE and not height_str == '':
                height_str = height_str[:(len(height_str)-1)] # remove last character
        
        if height_str == '':
            height_str = '0' # set to 0 if empty
        annotations[idx]['height'] = int(height_str)
        
	# Handles the editing of the occurence chances of objects
    def select_objects(self,idx):
        if self.mouse['l_down'] == 1:
            # The region in which the object rows are positioned
            boundaries = [(self.obj_sX+412,self.obj_sY), \
                          (self.obj_sX+self.object_row.get_size()[0],self.obj_sY+(self.object_row.get_size()[1]*len(self.object_range)))]
            # If mouse is in the correct area:
            if self.mouse['pos'][0] >= boundaries[0][0] and self.mouse['pos'][0] < boundaries[1][0] and \
               self.mouse['pos'][1] >= boundaries[0][1] and self.mouse['pos'][1] < boundaries[1][1]:
                # Get the selected row and column
                (obj,probability) = self.get_position(self.mouse['pos'],boundaries,(22,22))
                if obj <= len(self.objects):
                    # Change the probability on left-click
                    if probability > 0:
                        self.annotations[idx]['objects'][self.objects[obj]] = probability
                    elif probability == 0:
                        if self.objects[obj] in self.annotations[idx]['objects']:
                            del self.annotations[idx]['objects'][self.objects[obj]]
                    else:
                        print "[select_objects] probability selected lower than 0"
            else:
                # Check if the mouse is on one of the scroll buttons
                x_max_l = self.arrow_l_sX + self.arrow_l.get_size()[0]
                x_max_r = self.arrow_r_sX + self.arrow_r.get_size()[0]
                y_max = self.arrows_sY + self.arrow_l.get_size()[1]
                if self.mouse['pos'][0] >= self.arrow_l_sX and self.mouse['pos'][0] < x_max_l and \
                   self.mouse['pos'][1] >= self.arrows_sY and self.mouse['pos'][1] < y_max:
                    # left scroll button clicked
                    if (self.scroll_object-1)*25 >= 0:
                        self.scroll_object += -1
                elif self.mouse['pos'][0] >= self.arrow_r_sX and self.mouse['pos'][0] < x_max_r and \
                     self.mouse['pos'][1] >= self.arrows_sY and self.mouse['pos'][1] < y_max:
                    # right scroll button clicked
                    if (self.scroll_object+1)*25 < len(self.objects):
                        self.scroll_object += 1
                    
	# Handles what happens when the menu state is active
    def handle_menu(self, m, k):
        if k == K_ESCAPE:
            self.mode = self.CREATE_NEW # exit menu state
        elif m['pos'][0] >= self.menu_sX and m['pos'][0] < (self.menu_sX + self.menu_button.get_size()[0]) and \
             m['pos'][1] >= self.menu_sY and m['pos'][1] < (self.menu_sY + self.menu_button.get_size()[1]) and \
             m['l_down'] == 1:
            # Save button was clicked
            print "saving"
            self.export_annotations()
            self.mode = self.CREATE_NEW # exit menu state
            
    # Handles the input when in 'locations' mode        
    def handle_inputs_single(self):
        if self.mode == self.CREATE_NEW:
            if self.mouse['l_down'] == 1 and self.mouse_on_map(self.mouse['pos']): # Left-click
                # Create new location
                self.locations.append({'name': '*new*', 'point': [], 'angle': 0, 'colour': (0,0,0), 'transform': True})
                self.original_points.append([]) # Empty original points to follow the locations' indices
                self.set_new_colour(self.locations)
                # Add point
                self.add_point(self.mouse['pos'], self.locations)
                self.mode = self.CREATING
            elif (self.mouse['l_down'] == 1 or self.mouse['r_down'] == 1) and not self.mouse_on_map(self.mouse['pos']):
                # Check if an annotation key is clicked
                select = self.get_key_entry(self.mouse['pos'],self.key_polygon)
                if not select == None:
                    self.current_idx = select
                    self.mode = self.SELECTED
                    self.property = None
            elif self.key == K_ESCAPE: # Escape
                self.key = None # Key reset so it doesn't escape out of menu straight away
                self.mode = self.MENU
                
        if self.mode == self.CREATING:
            if self.mouse['l_down'] == 1 and self.mouse_on_map(self.mouse['pos']): # Left click
                # Rotation point
                self.rotating = True
            if self.rotating and self.mouse_on_map(self.mouse['pos']):
                self.store_orientation(self.mouse['pos'], self.locations)
                if self.mouse['l_up'] == 1:
                    # Finish rotation
                    self.rotating = False
            elif self.key == K_BACKSPACE:
                # Remove point
                self.remove_last_point(self.locations)
                if self.no_points(self.locations):
                    self.mode = self.CREATE_NEW
                
            elif self.key == K_RETURN or self.key == K_KP_ENTER:
                # End drawing
                self.current_idx = -1
                self.mode = self.NAME
                
        elif self.mode == self.NAME:
            if self.current_idx == -1:
                self.current_idx = self.get_empty_idx(self.locations)            
            self.name_annotation(self.current_idx, self.locations)
            # Enter to finish the location    
            if (self.key == K_RETURN or self.key == K_KP_ENTER) and len(self.locations[self.current_idx]['name']) > 0:
                self.mode = self.CREATE_NEW

        # Selected an annotation
        elif self.mode == self.SELECTED:
            if self.property == None:
                if self.key == K_DELETE:
                    # delete annotation
                    self.locations.remove(self.locations[self.current_idx])
                    self.original_points.remove(self.original_points[self.current_idx])
                    self.mode = self.CREATE_NEW
                elif self.key == K_RETURN or self.key == K_KP_ENTER:
                    # edit annotation
                    self.property = self.NAME
                elif self.key == K_ESCAPE:
                    # cancel selection
                    self.mode = self.CREATE_NEW
                       
            elif self.property == self.NAME:
                self.name_annotation(self.current_idx, self.locations)
                if self.key == K_RETURN or self.key == K_KP_ENTER:
                    self.property = None
                    self.mode = self.CREATE_NEW
                    
        elif self.mode == self.MENU:
            self.handle_menu(self.mouse,self.key)
        
    # Handles the input when in 'regions' mode    
    def handle_inputs_polygon(self):
        # Default mode
        if self.mode == self.CREATE_NEW:
            if self.mouse['l_down'] == 1 and self.mouse_on_map(self.mouse['pos']): # Left-click
                # Create new empty annotation
                self.annotations.append({'name': '*new*', 'points': [], 'height': 0, 'objects': {}, 'colour': (0,0,0), 'transform': True})
                self.original_points.append([]) # Empty original points to follow the annotations's indices
                self.set_new_colour(self.annotations)
                self.mode = self.CREATING
            elif (self.mouse['l_down'] == 1 or self.mouse['r_down'] == 1) and not self.mouse_on_map(self.mouse['pos']):
                # Check if an annotation key is clicked
                select = self.get_key_entry(self.mouse['pos'],self.key_polygon)
                if not select == None:
                    self.current_idx = select
                    self.mode = self.SELECTED
                    self.property = None
            elif self.mouse['r_down'] == 1: # Right-click
                # Find the polygon at the position of the mouse, if any
                idx = self.get_polygon_idx(self.mouse['pos'])
                if not idx == None:
                    self.current_idx = idx;
                    self.mode = self.SELECTED
                    self.property = None
            elif self.key == K_ESCAPE: # Escape
                self.key = None # Key reset so it doesn't escape out of menu straight away
                self.mode = self.MENU
                
        if self.mode == self.CREATING:
            if self.mouse['l_down'] == 1:
                # Add point
                self.add_point(self.mouse['pos'], self.annotations)
            elif self.key == K_BACKSPACE:
                # Remove last point
                self.remove_last_point(self.annotations)
                if self.no_points(self.annotations):
                    self.mode = self.CREATE_NEW
                
            elif self.key == K_RETURN or self.key == K_KP_ENTER:
                # End drawing
                self.current_idx = -1
                self.mode = self.NAME
                
        elif self.mode == self.NAME:
            if self.current_idx == -1:
                self.current_idx = self.get_empty_idx(self.annotations)            
            self.name_annotation(self.current_idx, self.annotations)
            # Enter to continue to next state     
            if (self.key == K_RETURN or self.key == K_KP_ENTER) and len(self.annotations[self.current_idx]['name']) > 0:
                self.mode = self.HEIGHT
                
        elif self.mode == self.HEIGHT:
            if self.current_idx == -1:
                self.current_idx = self.get_empty_idx(self.annotations)            
            self.change_height(self.current_idx, self.annotations)
            # Enter to continue to next state  
            if self.key == K_RETURN or self.key == K_KP_ENTER:
                self.mode = self.OBJECTS
                
        elif self.mode == self.OBJECTS:
            if self.current_idx == -1:
                self.current_idx = self.get_empty_idx(self.annotations)                
            self.select_objects(self.current_idx)            
            # Enter completes creating the annotation
            if self.key == K_RETURN or self.key == K_KP_ENTER:
                self.scroll_object = 0
                self.mode = self.CREATE_NEW
                self.sort_annotations('height', self.annotations)
                
        elif self.mode == self.MENU:
            self.handle_menu(self.mouse,self.key)
                
        # Selected an annotation
        elif self.mode == self.SELECTED:
            if self.property == None:
                if self.key == K_DELETE:
                    # delete annotation
                    self.annotations.remove(self.annotations[self.current_idx])
                    self.original_points.remove(self.original_points[self.current_idx])
                    self.mode = self.CREATE_NEW
                elif self.key == K_RETURN or self.key == K_KP_ENTER:
                    # edit annotation
                    self.property = self.NAME
                elif self.key == K_ESCAPE:
                    # cancel selection
                    self.mode = self.CREATE_NEW
                       
            elif self.property == self.NAME:
                self.name_annotation(self.current_idx, self.annotations)
                if self.key == K_RETURN or self.key == K_KP_ENTER:
                    self.property = self.HEIGHT
                
            elif self.property == self.HEIGHT:
                self.change_height(self.current_idx, self.annotations)
                if self.key == K_RETURN or self.key == K_KP_ENTER:
                    self.property = self.OBJECTS
                
            elif self.property == self.OBJECTS:
                self.select_objects(self.current_idx)
                # Enter finishes editing the annotation
                if self.key == K_RETURN or self.key == K_KP_ENTER:
                    self.scroll_object = 0
                    self.sort_annotations('height')
                    self.property = None
                    self.mode = self.CREATE_NEW
       
	# Handles the key and mouse operations and changes the state accordingly
    def handle_annotations(self):
        if self.annotate_mode == 'regions':
            self.handle_inputs_polygon()
        else:
            self.handle_inputs_single()
        
	# Draws the polygons
    def draw_annotations(self, screen, annotations):
        selected = -1
        
        if self.annotate_mode == "regions":
            for i in range(len(annotations)):
                if len(annotations[i]['points']) > 2:
                    pygame.draw.polygon(screen, annotations[i]['colour'], annotations[i]['points'])
                    if self.mode == self.SELECTED and i == self.current_idx:
                        selected = i
                else:
                    # Draw the points when there aren't enough to form a polygon
                    for (px,py) in annotations[i]['points']:
                        pygame.draw.rect(screen, annotations[i]['colour'], (px, py, 4, 4))
            if selected > -1:
                # Draw the edges of the polygon that is selected in red
                pygame.draw.polygon(screen, self.RED, annotations[selected]['points'], 2)
                
        else:
            if self.mode == self.SELECTED:
                selected = self.current_idx
            for i in range(len(annotations)):
                (px,py) = annotations[i]['point'][0]
                # Draw orientation
                start = (px,py)
                ex = px - math.cos(self.deg_to_rad(annotations[i]['angle'] - 90)) * self.direction_length
                ey = py + math.sin(self.deg_to_rad(annotations[i]['angle'] - 90)) * self.direction_length
                end = (ex,ey)
                pygame.draw.line(screen, annotations[i]['colour'], (px,py), end, 1)
                # Draw location
                pygame.draw.rect(screen, annotations[i]['colour'], (px-1, py-1, 4, 4), 3)
                if selected > -1 and i == selected:
                    # Draw edge when location is selected
                    pygame.draw.rect(screen, self.BLUE, (px-1, py-1, 4, 4), 7)
                
    
    # Draws the list of annotations with their colours to the left        
    def draw_key(self, screen, annotations):
        # annotation icon; a rectangle with 4 points
        self.key_polygon = [(self.key_sX,self.key_sY),(self.key_sX+25,self.key_sY),(self.key_sX+25,self.key_sY+9),(self.key_sX,self.key_sY+9)]
        
        (tx,ty) = (self.key_polygon[1][0] + 5,self.key_polygon[0][1] - 4)
        for i,an in enumerate(annotations):
            pygame.draw.polygon(screen,an['colour'],self.key_polygon)
            if self.mode == self.SELECTED and i == self.current_idx:
                pygame.draw.polygon(screen,self.RED,self.key_polygon,1)
            
            string = an['name']
            text = self.normal.render(string,True,self.BLACK)
            screen.blit(text,(tx,ty))
            
            # Display the next annotations after some y-spacing
            ty += self.key_spacing
            self.key_polygon = self.shift_down(self.key_polygon,self.key_spacing)
            
	# Draws the box which includes the name
    def draw_name_editor(self, screen, annotations, focus=False):
        # Draw the box
        screen.blit(self.name_field,(self.name_sX,self.name_sY))
        if focus == True:
            screen.blit(self.name_field_border,(self.name_sX,self.name_sY))
            
        # Display name in the box
        string = annotations[self.current_idx]['name']
        text = self.normal.render(string,True,self.BLACK)
        t_pos_x = self.name_sX + 5
        t_pos_y = self.name_sY + 7
        screen.blit(text,(t_pos_x,t_pos_y))
        
	# Draws the box which includes the height
    def draw_height_editor(self, screen, focus=False):
        # Draw the box
        screen.blit(self.name_field,(self.height_sX,self.height_sY))
        if focus == True:
            screen.blit(self.name_field_border,(self.height_sX,self.height_sY))
            
        # Display unit name
        (w,h) = self.normal.size("cm")
        text = self.normal.render("cm",True,self.BLACK)
        t_pos_x = self.height_sX + (self.name_field_border.get_size()[0] - w - 5)
        t_pos_y = self.height_sY + 8
        screen.blit(text,(t_pos_x,t_pos_y))
        
        # Display height in the box
        string = str(self.annotations[self.current_idx]['height'])
        text = self.normal.render(string,True,self.BLACK)
        t_pos_x = self.height_sX + 5
        t_pos_y = self.height_sY + 7
        screen.blit(text,(t_pos_x,t_pos_y))
        
	# Draws the list of objects and their chance of occurence
	# 25 objects per scroll page
    def draw_object_selector(self,screen):
        length = len(self.objects)
        height = self.object_row.get_size()[1]		
        (sx,sy) = (self.obj_sX,self.obj_sY)
        
        # Draw indicator key
        (tx,ty) = (sx+415,sy-25)
        text = self.large.render("+",True,self.BLUE)
        screen.blit(text,(tx,ty))
        (tx,ty) = ((sx+self.object_row.get_size()[0]-15),sy-25)
        text = self.large.render("-",True,self.BLUE)
        screen.blit(text,(tx,ty))
        
        (tx,ty) = (sx+5,sy+3)
        
        # Specify range of objects to display
        lower_bound = self.scroll_object*25
        upper_bound = (self.scroll_object+1)*25
        if lower_bound > length:
            lower_bound = length
        if upper_bound > length:
            upper_bound = length
        self.object_range = range(lower_bound,upper_bound)
        for i in self.object_range:
            # Draw rows
            screen.blit(self.object_row,(sx,sy))
            
            # Draw object names
            string = self.objects[i]
            text = self.normal.render(string,True,self.BLACK)
            screen.blit(text,(tx,ty))
            
            # Draw probability indicator
            if string in self.annotations[self.current_idx]['objects']:
                if self.annotations[self.current_idx]['objects'][string] == 3:
                    # Guaranteed
                    screen.blit(self.guaranteed,(sx,sy))
                elif self.annotations[self.current_idx]['objects'][string] == 2:
                    # Likely
                    screen.blit(self.likely,(sx,sy))
                elif self.annotations[self.current_idx]['objects'][string] == 1:
                    # Unlikely
                    screen.blit(self.unlikely,(sx,sy))
            else:
                # Impossible
                screen.blit(self.impossible,(sx,sy))

            sy += height
            ty += height
            
        # Draw the scrolling arrows
        self.arrow_l_sX = self.obj_sX
        self.arrow_r_sX = self.arrow_l_sX + (self.object_row.get_size()[0] - self.arrow_r.get_size()[0])
        self.arrows_sY = sy + 5        
        if (self.scroll_object-1)*25 >= 0:
            screen.blit(self.arrow_l,(self.arrow_l_sX,self.arrows_sY))
        if (self.scroll_object+1)*25 < length:
            screen.blit(self.arrow_r,(self.arrow_r_sX,self.arrows_sY))
            
	# Draw the instructions when an annotation is selected
    def draw_selected(self,screen):
        # Draw the boxes
        screen.blit(self.instruction_field,(self.instr_del_sX,self.instr_del_sY))
        screen.blit(self.instruction_field,(self.instr_enter_sX,self.instr_enter_sY))
            
        # Display instructions in the box
        string = "press [DEL] to fully delete this annotation."
        (w,h) = self.normal.size(string)
        text = self.normal.render(string,True,self.BLACK)
        t_pos_x = self.instr_del_sX + (self.instruction_field.get_size()[0] - w)/2
        t_pos_y = self.instr_del_sY + 7
        screen.blit(text,(t_pos_x,t_pos_y))
        string = "press [ENTER] to edit this annotation's properties."
        (w,h) = self.normal.size(string)
        text = self.normal.render(string,True,self.BLACK)
        t_pos_x = self.instr_del_sX + (self.instruction_field.get_size()[0] - w)/2
        t_pos_y = self.instr_enter_sY + 7
        screen.blit(text,(t_pos_x,t_pos_y))
    
	# Draw what happens when escape is pressed
    def draw_menu(self,screen):
        # Draw button
        screen.blit(self.menu_button,(self.menu_sX,self.menu_sY))
        
        # Display text
        string = "Save annotations"
        text = self.normal.render(string,True,self.RED)
        t_pos_x = self.menu_sX + 7
        t_pos_y = self.menu_sY + 10
        screen.blit(text,(t_pos_x,t_pos_y))
    
	# The main sequential process of the program
    def main(self):        
        pygame.init()
        
        info = pygame.display.Info()
        self.screen_width = int(info.current_w * 0.9)
        self.screen_height = int(info.current_h * 0.9)
        # Set the resolution to 90% of the current desktop resolution
        window_dimensions = (self.screen_width,self.screen_height)
        self.screen_width = int(0.75 * self.screen_width) # use the right 3/4th of the screen for the actual map
        self.screen_x_start = window_dimensions[0] - self.screen_width
        DISPLAYSURF = pygame.display.set_mode(window_dimensions)    
        pygame.display.set_caption('Annotator')
        
        pygame.mouse.set_visible
        
        # Make sure the '~' token is replaced by the home location
        (self.map_location, self.yaml_location, self.objects_location, self.save_location, self.load_location) = self.rectify_locations(map_location,yaml_location,objects_location,save_location,load_location)
        
        # Load fonts
        self.normal = pygame.font.SysFont(None,20)
        self.large = pygame.font.SysFont(None,35)
        
        # Load UI elements
        try:
            self.menu_button = pygame.image.load("UI/menu_button.bmp")
            self.name_field = pygame.image.load("UI/name_field.bmp")
            self.instruction_field = pygame.image.load("UI/instruction_field.bmp")
            self.name_field_border = pygame.image.load("UI/name_field_border.bmp")
            self.object_row = pygame.image.load("UI/object_row.bmp")
            self.guaranteed = pygame.image.load("UI/object_guaranteed.bmp")
            self.likely = pygame.image.load("UI/object_likely.bmp")
            self.unlikely = pygame.image.load("UI/object_unlikely.bmp")
            self.impossible = pygame.image.load("UI/object_impossible.bmp")
            self.arrow_r = pygame.image.load("UI/arrow_r.bmp")
            self.arrow_l = pygame.image.load("UI/arrow_l.bmp")
        except:
            print "[Main] Missing UI image files."
            sys.exit()
        self.name_field_border.set_colorkey(self.C_KEY)
        self.guaranteed.set_colorkey(self.C_KEY)
        self.likely.set_colorkey(self.C_KEY)
        self.unlikely.set_colorkey(self.C_KEY)
        self.impossible.set_colorkey(self.C_KEY)
        self.arrow_l.set_colorkey(self.C_KEY)
        self.arrow_r.set_colorkey(self.C_KEY)
        
        self.set_UI_positions(self.screen_x_start,self.screen_width,self.screen_height)
        
        if self.annotate_mode == "regions":
            # Load objects
            f = None
            try:
                f = open(self.objects_location,'r')
            except:
                print "[Main] Failed to load objects file."
                sys.exit()
            f.seek(0)
            for line in f:
                name = []
                # Filter the newline from each line
                for c in line:
                    if c == '\n':
                        break
                    else:
                        name.append(c)
                name = "".join(name)
                # Filter empty lines
                if not name == "":
                    self.objects.append(name)
            f.close()
                
        # Load map
        try:
            map_image = pygame.image.load(self.map_location)
            im = Image.open(self.map_location)
            pixels = im.load() # PIL format
        except:
            print "[Main] Failed to load image."
            sys.exit()
        
        # Crop and scale the image to keep the interesting part
        cropped_image = self.transform_image(map_image,pixels,im)
        
        # Load annotations if applicable
        if not self.load_location == None:
            if self.annotate_mode == "regions":
                self.annotations = copy.deepcopy(self.load_annotations(self.load_location))
            else:
                self.locations = copy.deepcopy(self.load_annotations(self.load_location))
        
        quit = False
        # Repeating loop
        while not quit:
            frame_time = pygame.time.get_ticks()
            
            # Clear inputs each frame
            self.reset_mouse()
            self.key = None
            
            # Handle input events
            for event in pygame.event.get():
                if event.type == QUIT:
                    quit = True
                    pygame.quit()
                    sys.exit()
                else:
                    try:
                        self.mouse['pos'] = event.dict['pos']
                    except:
                        self.mouse['pos'] = (0,0)
                    if event.type == MOUSEBUTTONDOWN:
                        if event.dict['button'] == 1:
                            self.mouse['l_down'] = 1
                        elif event.dict['button'] == 3:
                            self.mouse['r_down'] = 1
                    elif event.type == MOUSEBUTTONUP:
                        if event.dict['button'] == 1:
                            self.mouse['l_up'] = 1
                        elif event.dict['button'] == 3:
                            self.mouse['r_up'] = 1
                    if event.type == KEYDOWN:
                        self.key = event.dict['key']
                            
            self.handle_annotations()            
            
            DISPLAYSURF.fill(self.BACKGROUND)
            DISPLAYSURF.blit(cropped_image,(self.screen_x_start,0))
            if self.annotate_mode == "regions":
                self.draw_annotations(DISPLAYSURF, self.annotations)
                self.draw_key(DISPLAYSURF, self.annotations)
            else:
                self.draw_annotations(DISPLAYSURF, self.locations)
                self.draw_key(DISPLAYSURF, self.locations)            
            
            if self.mode == self.NAME or self.property == self.NAME:
                if self.annotate_mode == "regions":
                    self.draw_name_editor(DISPLAYSURF, self.annotations, True)
                    self.draw_height_editor(DISPLAYSURF)
                else:
                    self.draw_name_editor(DISPLAYSURF, self.locations, True)
            elif self.mode == self.HEIGHT or self.property == self.HEIGHT:
                self.draw_name_editor(DISPLAYSURF, self.annotations)
                self.draw_height_editor(DISPLAYSURF,True)
            elif self.mode == self.OBJECTS or self.property == self.OBJECTS:
                self.draw_object_selector(DISPLAYSURF)
            elif self.mode == self.SELECTED and self.property == None:
                self.draw_selected(DISPLAYSURF)
            elif self.mode == self.MENU:
                self.draw_menu(DISPLAYSURF)
        
            pygame.display.update()
            
            frame_time = pygame.time.get_ticks() - frame_time
            if frame_time < 1000/self.MAX_FPS:
                pygame.time.delay(1000/self.MAX_FPS - frame_time)

if __name__ == '__main__':
    # See http://www.ros.org/wiki/ROS/Tutorials/WritingServiceClient%28python%29 for how to deal with input parameters
    # Read parameters from the command line
    # Note: first parameter is not visible and is always there
    if len(sys.argv) >= 5 and sys.argv[1] == "locations":
        mode = "locations"
        map_location = sys.argv[2]
        yaml_location = sys.argv[3]
        objects_location = None
        save_location = sys.argv[4]
        load_location = None
        if len(sys.argv) == 6:
            load_location = sys.argv[5]
    elif len(sys.argv) >= 5 and sys.argv[1] == "regions":
        mode = "regions"
        map_location = sys.argv[2]
        yaml_location = None
        objects_location = sys.argv[3]
        save_location = sys.argv[4]
        load_location = None
        mode = "regions"
        if len(sys.argv) == 6:
            load_location = sys.argv[5]
    else:
        # There were not enough parameters on the command line, ask for them instead
        mode = raw_input("What mode should be used for annotations? Choose between [regions] and [locations]?: ")
        map_location = raw_input('Location of the map image file: ')
        if mode == "regions":
            objects_location = raw_input('Location of the objects list file: ')
        else:
            yaml_location = raw_input('Location of the .yaml file of the map: ')
        save_location = raw_input('Where would you like to save the annotation (include file name)?: ')
        load_location = raw_input('Which annotation file would you like to load (can be left empty)?: ')
    
    c = Main()
    c.init(map_location, yaml_location, objects_location, save_location, load_location, mode)
    c.main()

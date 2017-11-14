import os
import sys
import random
import time
import math
import copy
import ast
import numpy as np
from PIL import Image
from operator import itemgetter

class Tools():
    ## --- Functions to be used from outside --- ##
    
    def __init__(self):
        pass    
        
    '''Estimate center of gravity of a polygon'''
    def estimate_COG(self, polygon):
        bounding_box = self.form_bounding_box(polygon)
        ##print bounding_box
        filtered = []
        # Required are at least 2 sample points to form into an average (for small polygons)
        while len(filtered) < 2:
            points = self.generate_points(bounding_box)
            filtered = self.keep_points_inside(points,polygon)
        center = self.compute_average(filtered)
        
        print 'center: ', center
        return center
        
    def distance_point_to_line(self, v, w, p):
        length = math.pow((v[0]-w[0]),2) + math.pow((v[1]-w[1]),2)
        if length == 0.0:
            return self.calculate_distance(p,v)
        t = self.dot(self.subtract(p,v),self.subtract(w,v)) / length
        if t < 0:
            return self.calculate_distance(p,v)
        elif t > 1:
            return self.calculate_distance(p,w)
        projection = self.add(v,(self.mult(self.subtract(w,v),t)))
        return self.calculate_distance(p,projection)
        
    def distance_to_polygon(self, polygon, p, min_thresh=0.25, max_thresh=1.5):
        min_distance = self.distance_point_to_line(polygon[0],polygon[1],p)
        for i in range(len(polygon)-1):
            dist = self.distance_point_to_line(polygon[i],polygon[i+1],p)
            if dist < min_distance:
                min_distance = dist
        dist = self.distance_point_to_line(polygon[len(polygon)-1],polygon[0],p)
        if dist < min_distance:
            min_distance = dist
                
        if min_distance >= min_thresh and min_distance <= max_thresh:
            return True
        else:
            return False
        
    '''Generates a number of random points near an annotation (inside the bounding box)
    and returns the closest one to the current location.
    The gap parameter specifies how many pixels/meters to extend the bounding box from the polygon.
    The margin parameter specifies how far the point must lie from the border of the polygon'''     
    def point_near_annotation(self, annotation, current, gap, margin=0.20):
        box = self.form_bounding_box(annotation['points'],gap)
        #edge = {'x': (box['x']+gap)-margin, 'y': (box['y']+gap)-margin, 'width': (box['width']-2*gap)+2*margin, 'height': (box['height']-2*gap)+2*margin}
        ##print 'bounding box: ', box
        ##print 'edge: ', edge
        ##print annotation['name'] + '[points]: ', annotation['points']
        points = []
        # Generate x points around the annotation
        for i in range(200):
            point = self.generate_point(box)
            while point in points or self.point_in_polygon(annotation['points'],point) or not self.distance_to_polygon(annotation['points'],point,margin,gap):
                point = self.generate_point(box)
            points.append(point)
            
        ##print 'generated points: ', points            
        # Keep the closest one
        best_point_idx = 0
        min_distance = 999999999999
        for i,p in enumerate(points):
            distance = self.calculate_distance(p, current)
            if distance < min_distance:
                min_distance = distance
                best_point_idx = i
                
        ##print 'closest point: ', points[best_point_idx]
        return points[best_point_idx]
        
    '''Generates a given number of random points near an annotation (inside the bounding box)
    and sorts these in clockwise or counterclockwise order.
    The gap parameter specifies how many pixels/meters to extend the bounding box from the polygon.
    The margin parameter specifies how far the point must lie from the border of the polygon'''     
    def points_near_annotation(self, annotation, current, gap, margin=0.20, amount=20, clockwise=True):
        box = self.form_bounding_box(annotation['points'],gap)
        #edge = {'x': (box['x']+gap)-margin, 'y': (box['y']+gap)-margin, 'width': (box['width']-2*gap)+2*margin, 'height': (box['height']-2*gap)+2*margin}
        ##print 'bounding box: ', box
        ##print 'edge: ', edge
        points = []
        # Generate x points around the annotation
        for i in range(amount):
            point = self.generate_point(box)
            while point in points or self.point_in_polygon(annotation['points'],point) or not self.distance_to_polygon(annotation['points'],point,margin,gap):
                point = self.generate_point(box)
            points.append({'point': point})
            
        # Sort the points in (counter)clockwise order
        center = self.estimate_COG(annotation['points'])
        for p in points:
            deltaX = center[0] - p['point'][0]
            deltaY = p['point'][1] - center[1] # Reversed because y = 0 is at the top
            angle = math.atan2(deltaY,deltaX)
            p['angle'] = angle
            
        ##print points
        if clockwise:
            points = sorted(points, key=itemgetter('angle'))
        else:
            points = sorted(points, key=itemgetter('angle'), reverse=True)
        point_list = []
        point_list.append(current) # Set the current position as the first point
        for p in points:
            point_list.append(p['point'])
        ##print 'points:'
        ##print point_list
        
        return point_list
        
    '''Returns the angle that the robot must be turned towards to be facing the center of an annotation.
    Range is [-180,180]'''
    def angle_to_center(self, annotation, current):
        center = self.estimate_COG(annotation['points'])
        deltaX = center[0] - current['x']
        deltaY = current['y'] - center[1] # Reversed because y = 0 is at the top
        angle_radians = math.atan2(deltaY,deltaX)
        angle_degrees = angle_radians * (180 / math.pi)
        
        return angle_degrees
        
    '''Calculates Euclidean distance between two 2D points'''
    def calculate_distance(self, p1, p2):
        (px1,py1) = p1
        (px2,py2) = p2
        dx = px1-px2
        dy = py1-py2
        return math.sqrt(dx*dx + dy*dy)
        
    '''Horizontal intersecting rays method to check if a target point lies within a polygon, see:
	http://alienryderflex.com/polygon/'''
    def point_in_polygon(self, polygon, target):
        j = len(polygon)-1
        odd = False
        
        for i in range(len(polygon)):
            if (polygon[i][1] < target[1] and polygon[j][1] >= target[1]) or \
               (polygon[j][1] < target[1] and polygon[i][1] >= target[1]):
                if polygon[i][0] + (float(target[1])-polygon[i][1])/(polygon[j][1]-polygon[i][1])*(polygon[j][0]-polygon[i][0]) < target[0]:
                    odd = not odd
            j = i
            
        if odd:
            return True
        else:
            return False
            
    '''Loads locations from a file'''
    def load_locations(self, load, yaml):
        try:
            f = open(load,'r')
        except:
            print '[load_locations] Could not load from location: %s' % load
        try:
            y = open(yaml, 'r')
            y.close()
        except:
            print '[load_locations] Could not open the yaml map file: %s' % yaml
      
        (resolution,origin) = self.read_yaml(yaml)
            
        temp_annots = [] # List of the annotations
        temp_annot = {'name': '', 'point': [], 'angle': 0} # Prototype of annotation
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
            elif next == 'point':
                temp_annot['point'] = ast.literal_eval(line)
                temp_annot['point'] = self.convert_points(temp_annot['point'], resolution, origin)
                next = None
            elif next == 'angle':
                temp_annot['angle'] = ast.literal_eval(line)
                next = None
            elif next == 'colour':
                next = None
            elif '# name' in line:
                next = 'name'
            elif '# colour' in line:
                next = 'colour'
            elif '# orientation':
                next = 'angle'
            elif '# point' in line:
                next = 'point'
            elif '~~~' in line:
                temp_annots.append(copy.deepcopy(temp_annot))
                next = None
            elif '# end' in line:
                next = None
            else:
                print '[load_locations] unrecognisable format in load file'
                sys.exit()
        
        ##print temp_annots
        return temp_annots
        
    '''Loads the annotations from a file'''
    def load_annotations(self, load, yaml):
        try:
            f = open(load,'r')
        except:
            print '[load_annotations] Could not load from location: %s' % load
        try:
            y = open(yaml, 'r')
            y.close()
        except:
            print '[load_annotations] Could not open the yaml map file: %s' % yaml
      
        (resolution,origin) = self.read_yaml(yaml)
            
        temp_annots = [] # List of the annotations
        temp_annot = {'name': '', 'points': [], 'height': 0, 'objects': {}} # Prototype of annotation
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
                next = None
            elif next == 'height':
                temp_annot['height'] = int(line)
                next = None
            elif next == 'points':
                temp_annot['points'] = ast.literal_eval(line)
                temp_annot['points'] = self.convert_points(temp_annot['points'],resolution,origin)
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
        
        ##print temp_annots
        return temp_annots
    
    ## --- Functions that should not need to be accessed by outside sources --- ##
    
    def dot(self, x, y):
        return x[0]*y[0] + x[1]*y[1]
        
    def add(self, x, y):
        return ((x[0]+y[0]),(x[1]+y[1]))     
        
    def subtract(self, x, y):
        return ((x[0]-y[0]),(x[1]-y[1]))
     
    def mult(self, x, s):
        return (x[0]*s,x[1]*s)
     
    '''Sets a rectangular bounding box around a polygon with an optinonal size increase in pixels'''
    def form_bounding_box(self, polygon, size_increase=0):
        box = {'x': 0, 'y': 0, 'width': 0, 'height': 0}
        box['x'] = sorted(polygon,key=lambda k: k[0])[0][0] - size_increase
        box['y'] = sorted(polygon,key=lambda k: k[1])[0][1] - size_increase
        box['width'] = (sorted(polygon,key=lambda k: k[0],reverse=True)[0][0] - box['x']) + size_increase
        box['height'] = (sorted(polygon,key=lambda k: k[1],reverse=True)[0][1] - box['y']) + size_increase
        
        return box
    
    '''Transforms a bounding box as a dictionary to a set of coordinates'''
    def bounding_box_to_polygon(self, box):
        points = []
        points.append((box['x'],box['y']))
        points.append((box['x']+box['width'],box['y']))
        points.append((box['x'],box['y']+box['height']))
        points.append((box['x']+box['width'],box['y']+box['height']))
        
        return points
    
    '''Coverts the points from pixels to map coordinates
    Map is represented with an origin (zero point) in the center
    The resolution equals the amount of meters per pixel'''
    def convert_points(self, points, res, o):
        new = []
        for p in points:
            point = (p[0]*res,p[1]*res)
            ##print point
            point = (point[0]+o[0],(point[1]+o[1])*-1)
            new.append(point)
            
        return new
    
    '''Generate one point inside a bounding box'''
    def generate_point(self, bounding_box):
        r_x = random.uniform(bounding_box['x'], bounding_box['x']+bounding_box['width'])
        r_y = random.uniform(bounding_box['y'], bounding_box['y']+bounding_box['height'])
        
        return (r_x,r_y)
    
    '''Generates n random points inside a region. The amount of points depends on the size of the region'''
    def generate_points(self, bounding_box):
        n = bounding_box['width']*bounding_box['height'] / 4 # amount of points is equal to 1/4th of the surface size
        ##print 'n: %d' % n
        # Limit number from 15 to 10000 points
        if n > 10000:
            n = 10000
        elif n < 15:
            n = 15
        points = []
        for i in range(n):
            r_x = random.uniform(bounding_box['x'], bounding_box['x']+bounding_box['width'])
            r_y = random.uniform(bounding_box['y'], bounding_box['y']+bounding_box['height'])
            points.append((r_x,r_y))
        
        ##print '----points----'    
        ##print points
        ##print '-------'
        return points
        
    '''Keep the points that are inside of a given polygon'''     
    def keep_points_inside(self, points, polygon):
        kept = []
        for p in points:
            if self.point_in_polygon(polygon,p):
                kept.append(p)
        
        ##print '----points kept-----'
        ##print kept
        ##print '----------------'
        return kept
    
    '''Compute the average (or center) of a group of 2D points'''          
    def compute_average(self, points):
        cumulative_x = 0
        cumulative_y = 0
        print 'number of points: %d' % len(points)
        for (x,y) in points:
            cumulative_x += x
            cumulative_y += y
        
        mean_x = cumulative_x / len(points)
        mean_y = cumulative_y / len(points)
        
        return (mean_x,mean_y)
        
    '''Reads the resolution and origin from a .yaml file and returns these'''    
    def read_yaml(self, file_name):
        try:
            f = open(file_name)
        except:
            print '[read_yaml] Cannot open file: %s' % file_name
        res = None
        origin = None
        for line in f:
            ##print line
            test = line.find('resolution: ')
            ##print 'test: %s' % test
            if test > -1:
                res = ast.literal_eval(line[test+len('resolution: '):])
            test = line.find('origin: ')
            ##print 'test: %s' % test
            if test > -1:
                origin = ast.literal_eval(line[test+len('origin: '):])
                
        #print res, origin          
        return (res,tuple(origin))
                
                
                
                
                

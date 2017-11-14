#!/usr/bin/env python

import os
from os.path import expanduser
import sys
import tempfile
import subprocess
import fnmatch
import shutil
import time
import ast

class FindXtion(object):
    def parsePath(self, path):
        path = str(path)
        
        if path == '':
            print '[parsePath] Path was not entered correctly.'
            sys.exit()            
       
        home = expanduser('~')
            
        if path[0] == '~':
            path = path[1:] # Strip ~
            path = path[::-1] # Reverse name
            path += home[::-1] # Add home directory
            path = path[::-1] # Reverse back to original
                        
        return path
        
    def findAndSpecify(self, path):
        process = subprocess.Popen(['lsusb >> usb_devices'], shell=True)
        process.wait()
        
        print path
        
        try:
            f_devices = open('usb_devices', 'r')
        except:
            print '[findAndSpecify] Could not open usb data.'
            return
        # Find (empty) Xtion entry
        xtion_line = None
        for line in f_devices:
            if ' \n' in line:
                xtion_line = line
        '''for line in f_devices:
            if 'ID' in l'''

        if not xtion_line:
            # Xtion not found
            print '[findAndSpecify] No Xtion entry found.'
            f_devices.close()
            return
        # Deconstruct into parts
        elements = xtion_line.split(' ')
        bus = int(elements[1])
        device = int(elements[3][:-1])

        f_devices.close()
        
        try:
            openni = open(path, 'r+')
            openni2 = open('openni_new', 'w')
        except:
            print '[findAndSpecify] Could not find openni launch file at specified location: ' + path
            return
        
        # Find the correct line in the openni file
        line_counter = 0
        for line in openni:
            if 'arg name="device_id" default=' in line:
                break;
            line_counter += 1
            
        # Construct new line entry
        place = line.find('default=')
        new_line = line[:place + len('default=')] 
        new_line = new_line + '"' + str(bus) + '@' + str(device) + '"'
        new_line = new_line + ' />\n'
        
        # Replace the old line in a new file
        openni.seek(0)
        for i, line in enumerate(openni):
            if i == line_counter:
                openni2.write(new_line)
            else:
                openni2.write(line)
        
        openni.close()
        openni2.close()
        
        # Remove old file
        process = subprocess.Popen(['rm ' + path], shell=True)
        process.wait()
        # Rename new file
        process = subprocess.Popen(['mv openni_new ' + path], shell=True)
        process.wait()
        # Remove usb data file
        process = subprocess.Popen(['rm usb_devices'], shell=True)
        process.wait()
            
if __name__ == '__main__':
    # Default path
    path = '~/sudo/ros/catkin_ws/src/borg_2dnav/openni_xtion.launch'
    if len(sys.argv) >= 2:
        path = sys.argv[1]
  
    c = FindXtion()
    path = c.parsePath(path)   # Make sure the path is readable
    c.findAndSpecify(path)

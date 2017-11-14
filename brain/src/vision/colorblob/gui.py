#!/usr/bin/python


import pygame
import time
import cv
import pickle
import sys

import glob 
import os 

import vision.colorblob.colorblob as colorblob
import vision.colorblob.colorlist as colorlist

class Gui(object):

    def __init__(self, colorlist_list):
        pygame.init()

        self.__run = True
        self.__freeze = False
        self.__show_detect = False
        self.__show_all = False

        self.__colorlist_list = colorlist_list
        #The currently selected colorlist:
        self.__colorlist_index = 0 
        self.select_colorlist(self.__colorlist_list[self.__colorlist_index])

        self.__display_surface = None


    def get_cv_image(self):
        return cv.QueryFrame(self.__capture)


    def select_colorlist(self, colorlist):
        print "Selecting colorlist with location: %s" % colorlist.get_file_location()
        self.__colorlist = colorlist


    def select_next_colorlist(self):
        self.__colorlist_index += 1
        if (self.__colorlist_index >= len(self.__colorlist_list)):
            self.__colorlist_index = 0
        self.select_colorlist(self.__colorlist_list[self.__colorlist_index])


    def select_previous_colorlist(self):
        self.__colorlist_index -= 1
        if (self.__colorlist_index < 0):
            self.__colorlist_index = len(self.__colorlist_list) - 1
        self.select_colorlist(self.__colorlist_list[self.__colorlist_index])


    def handle_events(self):
        for event in pygame.event.get():
            #Handle keyboard input:
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_q:
                    print "Quit program..."
                    self.__run = False
                if event.key == pygame.K_f:
                    print "Freezing image..."
                    self.__freeze = not self.__freeze
                if event.key == pygame.K_s:
                    print "Saving current selected color list..."
                    self.__colorlist.save()
                if event.key == pygame.K_k:
                    print "Saving all color lists..."
                    for colorlist in self.__colorlist_list:
                        colorlist.save()
                if event.key == pygame.K_l:
                    print "(Re)loading current selected color list..."
                    self.__colorlist.load()
                if event.key == pygame.K_d:
                    print "Deleting color list..."
                    self.__colorlist.empty_list()
                if event.key == pygame.K_a:
                    self.__colorlist.set_cube_size(self.__colorlist.get_cube_size() + 1)
                    print "Increased cube size to: %d" % self.__colorlist.get_cube_size()
                if event.key == pygame.K_z:
                    self.__colorlist.set_cube_size(self.__colorlist.get_cube_size() - 1)
                    print "Decreased cube size to: %d" % self.__colorlist.get_cube_size()
                if event.key == pygame.K_u:
                    self.__colorlist.remove_last()
                    print "Removed previous color, color list now holds %d colors." % len(self.__colorlist.get_list())
                if event.key == pygame.K_p:
                    self.select_next_colorlist()
                if event.key == pygame.K_o:
                    self.select_previous_colorlist()
                if event.key == pygame.K_r:
                    if self.__show_all:
                        print "Displaying all colors..."
                        self.__show_all = False
                    else:
                        print "NOT Displaying all colors..."
                        self.__show_all = True
                if event.key == pygame.K_t:
                    if self.__show_detect:
                        print "Toggling show detecting mode OFF"
                        self.__show_detect = False
                    else:
                        print "Toggling show detecting mode ON"
                        self.__show_detect = True


            #Handle mouse input:                        
            if event.type == pygame.MOUSEBUTTONUP:
                (m_x, m_y) = pygame.mouse.get_pos()
                if m_x < 0 or m_x > self.__source_surface.get_width() \
                        or m_y < 0 or m_y > self.__source_surface.get_height():
                    return

                current_color = self.__source_surface.get_at((m_x, m_y))

                if self.__colorlist.add_color(current_color):
                    print "Adding RGB color to list: (%d, %d, %d), now holds %d item(s)." \
                            % (current_color[0], current_color[1], current_color[2], len(self.__colorlist.get_list()) + 1)
                else:
                    print "This RGB color is already included by a cube: %d, %d, %d" % (current_color[0], current_color[1], current_color[2])


    def run(self):
        while self.__run:
            cv_im = self.get_cv_image()
            self.run_iterate(cv_im)


    def run_iterate(self, surface):
        if not self.__freeze or self.__source_surface == None:
            self.__source_surface = surface

        if self.__display_surface == None:
            width = 2 * self.__source_surface.get_width()
            height = self.__source_surface.get_height()
            if width < 400:
                width = 400
            self.__display_surface = pygame.display.set_mode((width, height), pygame.HWSURFACE)

        if not self.__show_all:
            #Detection:
            blob = colorblob.ColorBlob(self.__colorlist)
            blob.detect(self.__source_surface)
            #Display processed final surface:
            self.__display_surface.blit(blob.get_final_surface(), (self.__source_surface.get_width(), 0))
            if self.__show_detect:
                print "center: x=%d, y=%d, size: %f" % (blob.get_centroid()[0], blob.get_centroid()[1], blob.get_size())
        else:
            first = True
            for colorlist in self.__colorlist_list:
                #Detection:
                blob = ColorBlob(colorlist)
                blob.detect(self.__source_surface)
                #Display processed final surface:
                if first:
                    self.__display_surface.blit(blob.get_final_surface(), (self.__source_surface.get_width(), 0))
                    first = False
                else:
                    self.__display_surface.blit(blob.get_final_surface(), (self.__source_surface.get_width(), 0), None, pygame.BLEND_ADD)

        #Display input image:
        self.__display_surface.blit(self.__source_surface, (0, 0))

        #Display colorblob file, no. colors, cube_size:
        label_font = pygame.font.SysFont("courier", 12)
        file_name = os.path.basename(self.__colorlist.get_file_location())
        label = "%s, no colors: %d, cube_size %d" % (file_name, len(self.__colorlist.get_list()), self.__colorlist.get_cube_size())
        label_surface = label_font.render(label, 1, (0, 0, 0))
        bg_surface = pygame.Surface((label_surface.get_width(), label_surface.get_height()))
        bg_surface.fill((255, 255, 255))
        self.__display_surface.blit(bg_surface, (0, 0))
        self.__display_surface.blit(label_surface, (0, 0))

        pygame.display.flip()

        #Don't fry the CPU:
        time.sleep(0.01)
        
        self.handle_events()
    

def print_usage():
    print "Usage: blob.py [color_list_file]"
    print "Click on a pixel to add it to the color list."
    print "Press 'q' to quit."
    print "Press 'f' to freeze the current image."
    print "Press 'l' to load the (specified) color cube list."
    print "Press 's' to save the (specified) color cube list."
    print "Press 'd' to empty the (specified) color cube list."
    print "Press 'a' to increase the cube size."
    print "Press 'z' to decrease the cube size."
    print "Press 'u' to remove the last newly added color from the color cube list."
    print "Press 'b' to (try) to detect a color blob."
    print "Press 'p' to select the next *.colorblob file."
    print "Press 'o' to select the previous *.colorblob file."


if __name__ == "__main__":
    print_usage()

    if len(sys.argv) != 2:
        raise Exception("You need to specify the location of the directory where all *.colorblob files are located!")
        quit()

    gui = Gui(colorlist.load_color_file_list(sys.argv[1]))
    gui.run()


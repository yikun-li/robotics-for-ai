#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pickle
import copy

'''
This small piece of software will help you set up a targetblobs file for the new blob detector.

Just run it and you'll be given directions.

Written by: Marko Doornbos
'''

def get_color_values():
    
    # Ask for the name
    h_lower = int(raw_input("What should the lower bound for hue be? (0-180) "))
    h_upper = int(raw_input("What should the upper bound for hue be? (0-180) "))
    s_lower = int(raw_input("What should the lower bound for saturation be? (0-255) "))
    s_upper = int(raw_input("What should the upper bound for saturation be? (0-255) "))
    v_lower = int(raw_input("What should the lower bound for value be? (0-255) "))
    v_upper = int(raw_input("What should the upper bound for value be? (0-255) "))
    
    target_color_dict = {'h_lower': h_lower, 'h_upper': h_upper, 's_lower': s_lower, 's_upper': s_upper, 'v_lower': v_lower, 'v_upper': v_upper}
    
    return copy.deepcopy(target_color_dict)
    
def get_cube_dict():
    target_name = raw_input("What should the memory label for this color be? ")
    dict_list = []
    num_cubes = int(raw_input("How many different cubes should this color use? "))
    while num_cubes > 0:
        print str(num_cubes) + " more cubes to go for color " + target_name + "."
        dict_list.append(get_color_values())
        num_cubes -= 1
    return copy.deepcopy((target_name, dict_list))

def main():
    
    # Ask for a file name.
    goal = raw_input("Where should the target color file be stored? ")
    
    # Ask for number of target colors.
    nr_of_targets = int(raw_input("How many colours should be searched for? "))
    
    # Get the appropriate data.
    target_colors = []
    cnt = range(nr_of_targets)
    
    for el in cnt:
        print "Now asking for color " + str(el)
        target_colors.append(copy.deepcopy(get_cube_dict()))
    
    print target_colors
    # Next, save this dict at the target location.
    output=open(goal, 'wb')
    pickle.dump(target_colors, output)
    output.close() 

if __name__ == '__main__':
    main()

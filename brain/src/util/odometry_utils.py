# To change this template, choose Tools | Templates
# and open the template in the editor.

__author__="s1717928"
__date__ ="$Jun 6, 2011 2:17:34 PM$"

import math
import time

last_time = 0

def get_direction_to_goal(currentLocation, goalLocation):
    """ returns the direction towards a goal location from the current location
        of the robot.
        Direction > 0 => clockwise
        Direction < 0 => counterclockwise
    """
    
    # current direction the pioneer is looking (> 0 = counterclockwise, < 0 = clockwise)
    pioneerDirection = float(currentLocation['angle'])

    # revert the direction
    pioneerDirection *= -1

    # differences in coordinates between locations
    relativeY = float(goalLocation['x']) - float(currentLocation['x'])
    relativeX = float(currentLocation['y']) - float(goalLocation['y'])

    # direction to the goal when the pioneer was looking forward (0 degrees)
    absoluteDirection = math.degrees(math.atan2(relativeX, relativeY)) # range between -90 (most left) and 90 (most right)

    # Find the shortest direction to turn
    turn = [0, 0, 0]
    turn[0] = absoluteDirection - pioneerDirection
    turn[1] = absoluteDirection - pioneerDirection + 360
    turn[2] = absoluteDirection - pioneerDirection - 360
    directionDifference = 0
    minAngle = 360
    for angle in turn:
        if abs(angle) < minAngle:
            minAngle = abs(angle)
            directionDifference = angle

    return directionDifference # range between -180 (most left) and 180 (most right)


def location_match(location1, location2, errorRange=500):
    """ determines if the specified locations match within some range """
    range = errorRange # millimeter

    if abs(float(location1['x']) - float(location2['x'])) >= range:
        return False

    if abs(float(location1['y']) - float(location2['y'])) >= range:
        return False

    return True

def direction_match(direction1, direction2, errorRange=10):
    """ determines if the specified directions match within some range """
    range = errorRange # millimeter

    if abs(direction1 - direction2) >= range:
        return False

    return True

def get_closest_location(location, location_list):
    minDist = None
    minLoc = None
    for loc in location_list:
        dist_x = abs(float(loc['x']) - float(location['x']))
        dist_y = abs(float(loc['y']) - float(location['y']))
        dist = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        if minDist is None or dist < minDist:
            minDist = dist
            minLoc = loc
    return minLoc


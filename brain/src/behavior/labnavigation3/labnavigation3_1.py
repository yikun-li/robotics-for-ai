from __future__ import print_function

'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import math
import os

import basebehavior.behaviorimplementation
import rospy
import tf


class LabNavigation3_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):

        self.startNavigating = False
        self.data_path = '/brain/data/locations/lab.dat'
        # Dict containing <name: location> pairs
        self.storedLocations = {}
        self.read_stored_locations(os.environ['BORG'] + self.data_path)

        self.goto_movebase = self.ab.GotoMoveBase({'fileLocations': self.data_path})
        self.goto = self.ab.gotowrapper({})
        self.stuck = self.ab.sublabnavigation({})

        self.selected_behaviors = [
            ("goto_movebase", "True"),
            ("goto", "self.startNavigating == True"),
            ("stuck", "True"),
            ]

        self.state = 'enter'
        self.transform = tf.TransformListener()

        # Record history path
        self.path = []

        pass

    def implementation_update(self):

        if self.stuck.is_failed():
            print("Alice stuck!")
            self.transform.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(0.5))
            trans, rot = self.transform.lookupTransform('/map', '/base_link', rospy.Time(0))

            self.set_goal({trans[0]})
            self.stuck = self.ab.sublabnavigation({})

        if self.state == 'enter':
            self.state = 'goto_goal'
            self.set_goal('wp_g6')
            self.startNavigating = True

        elif self.state == 'goto_goal' and self.check_if_stuck():
            self.state = 'goal_visited'
            self.startNavigating = False

        elif self.state == 'goto_goal' and self.goto.is_finished():
            self.state = 'goal_visited'
            self.startNavigating = False

        elif self.state == 'goal_visited':
            self.set_finished()

        pass

    def check_if_stuck(self):
        self.transform.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(0.5))
        trans, rot = self.transform.lookupTransform('/map', '/base_link', rospy.Time(0))

        distance = math.pow((1 - trans[0]), 2) + math.pow((1 - trans[1]), 2)
        if distance <= 0.5:
            return True
        else:
            return False

    def set_goal(self, goal):
        self.goto = self.ab.gotowrapper({'goal': goal, 'error_range': -1})

    def read_stored_locations(self, fileName):
        """ Read file with stored locations during navigation training """

        # First line containing keys is omitted
        # Locations must be presented per line
        # Each location is stored as a dictionary {name: {x, y, angle}}
        try:
            fileHandle = open(fileName, 'r')
        except:
            print("Cannot open location file " + fileName)
            return

        firstLineSkipped = False
        for line in fileHandle.readlines():
            line = line.rstrip("\n")  # remove endline

            # Skip first line with keys
            if not firstLineSkipped:
                firstLineSkipped = True
                continue

                # Read location
            values = line.split(',')

            # Skip when line does not contain 4 arguments
            if len(values) != 4:
                continue

            # Skip comment lines
            if len(values[0]) > 0 and values[0][0] == '#':
                continue

            # Store location
            propDict = {'x': float(values[1]), 'y': float(values[2]), 'angle': float(values[3])}
            self.storedLocations[values[0]] = propDict
            print("Location loaded: " + values[0] + " " + str(propDict))

        fileHandle.close()

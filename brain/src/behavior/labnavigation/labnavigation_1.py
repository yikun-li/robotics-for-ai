'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import os

import basebehavior.behaviorimplementation
import tf


class LabNavigation_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):

        self.count = 1
        self.return_state = False
        self.prefix = ['wp_g', 'wp_r']

        self.startNavigating = False
        self.data_path = '/brain/data/locations/lab.dat'
        # Dict containing <name: location> pairs
        self.storedLocations = {}
        self.read_stored_locations(os.environ['BORG'] + self.data_path)

        self.goto_movebase = self.ab.GotoMoveBase({'fileLocations': self.data_path})
        self.goto = self.ab.gotowrapper({})

        self.selected_behaviors = [ \
            ("goto_movebase", "True"), \
            ("goto", "self.startNavigating == True"), \
            ]

        self.state = 'enter'
        self.transform = tf.TransformListener()

        pass

    def implementation_update(self):

        if self.state == 'enter':
            self.state = 'goto_goal'
            self.set_goal(self.get_goal())
            self.startNavigating = True

        elif self.state == 'goto_goal' and (self.goto.is_finished() or self.goto.is_failed()):
            self.state = 'goal_visited'
            self.startNavigating = False

        elif self.state == 'goal_visited' and not self.is_next_goal():
            self.set_finished()

        elif self.state == 'goal_visited':
            self.state = 'enter'
            self.next_goal()

        pass

    def set_goal(self, goal):
        self.goto = self.ab.gotowrapper({'goal': goal, 'error_range': -1})

    def get_goal(self):
        if self.return_state == False:
            return self.prefix[0] + str(self.count)
        else:
            return self.prefix[1] + str(self.count)

    def next_goal(self):
        if self.return_state == False and self.count == 6:
            self.count = 1
            self.return_state = True
        else:
            self.count += 1

    def is_next_goal(self):
        if self.count >= 6 and self.return_state == True:
            return False
        else:
            return True

    def read_stored_locations(self, fileName):
        """ Read file with stored locations during navigation training """

        # First line containing keys is omitted
        # Locations must be presented per line
        # Each location is stored as a dictionary {name: {x, y, angle}}
        try:
            fileHandle = open(fileName, 'r')
        except:
            print "Cannot open location file " + fileName
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
            print "Location loaded: " + values[0] + " " + str(propDict)

        fileHandle.close()

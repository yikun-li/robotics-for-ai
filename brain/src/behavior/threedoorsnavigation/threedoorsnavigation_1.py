'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import math
import os
import random

import basebehavior.behaviorimplementation
import rospy
import tf
from geometry_msgs.msg import PoseStamped


class ThreeDoorsNavigation_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        self.startNavigating = False
        self.data_path = '/brain/data/locations/3_doors_room.dat'
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
        self.randomly_get_doors_name = self.randomly_get_door_name()
        pass

    def implementation_update(self):

        if self.state == 'goto_recovery' and self.goto.is_finished():
            self.state = 'enter'
            self.startNavigating = False

        elif self.state == 'goto_recovery' and self.goto.is_failed():
            self.set_goal(self.randomly_get_recovery_point())

        if self.state == 'enter':
            self.set_goal(self.randomly_get_doors_name[0])
            self.startNavigating = True
            self.state = 'goto_door_entry'

        elif self.state == 'goto_door_entry' and self.goto.is_finished():
            self.state = 'goto_door_entry_visited'
            self.startNavigating = False

        elif self.state == 'goto_door_entry' and self.goto.is_failed():
            self.state = 'goto_recovery'
            self.set_goal(self.get_closest_recovery_point())

        elif self.state == 'goto_door_entry_visited':
            self.state = 'goto_door_entered'
            self.set_goal(self.randomly_get_doors_name[1])
            self.startNavigating = True

        elif self.state == 'goto_door_entered' and self.goto.is_finished():
            self.state = 'goto_door_entered_visited'
            self.startNavigating = False

        elif self.state == 'goto_door_entered' and self.goto.is_failed():
            self.state = 'goto_recovery'
            self.set_goal(self.get_closest_recovery_point())

        elif self.state == 'goto_door_entered_visited':
            self.set_finished()

        pass

    def set_goal(self, goal):
        self.goto = self.ab.gotowrapper({'goal': goal, 'error_range': -1})

    def randomly_get_door_name(self):
        doornum = random.randint(1, 3)

        self.transform.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(0.5))
        trans, rot = self.transform.lookupTransform('/map', '/base_link', rospy.Time(0))

        dict_distance = {}
        for i in self.storedLocations:
            if i.startswith('door' + str(doornum)) and i.endswith('entry'):
                dict_distance[i] = math.pow((self.storedLocations[i]['x'] - trans[0]), 2) + math.pow(
                    (self.storedLocations[i]['y'] - trans[1]), 2)
        min_door_name = min(dict_distance, key=dict_distance.get)

        return [min_door_name, min_door_name.replace('entry', 'entered')]

    def randomly_get_recovery_point(self):
        dict_recovery = {}
        for key, value in self.storedLocations.iteritems():
            if key.startswith('recovery'):
                dict_recovery[key] = value
        return random.choice(dict_recovery.values())

    def get_closest_recovery_point(self):

        self.transform.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(0.5))
        trans, rot = self.transform.lookupTransform('/map', '/base_link', rospy.Time(0))
        # print 'x: ', trans[0], 'y: ', trans[1]

        dict_distance = {}
        for i in self.storedLocations:
            if i.startswith('recovery'):
                dict_distance[i] = math.pow((self.storedLocations[i]['x'] - trans[0]), 2) + math.pow(
                    (self.storedLocations[i]['y'] - trans[1]), 2)
        min_revovery_point_name = min(dict_distance, key=dict_distance.get)

        return self.storedLocations[min_revovery_point_name]

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

    def find_behind_point(self, x=-2.0, y=0):
        self.transform.waitForTransform('/base_link', '/map', rospy.Time(0), rospy.Duration(0.2))
        new_point = PoseStamped()
        new_point.header.frame_id = 'base_link'
        new_point.header.stamp = rospy.Time(0)
        new_point.pose.position.x = x
        new_point.pose.position.y = y

        p = self.transform.transformPose('map', new_point)

        quaternion = (
            p.pose.orientation.x,
            p.pose.orientation.y,
            p.pose.orientation.z,
            p.pose.orientation.w
        )

        euler = tf.transformations.euler_from_quaternion(quaternion)

        return {'x': p.pose.position.x, 'y': p.pose.position.y, 'angle': math.degrees(euler[2])}

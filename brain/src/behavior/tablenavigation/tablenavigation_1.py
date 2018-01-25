from __future__ import print_function

import math
import operator
import os

import basebehavior.behaviorimplementation
import rospy
import tf
from geometry_msgs.msg import PoseStamped


class TableNavigation_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        if not hasattr(self, 'aim'):
            self.set_failed('No aim given.')

        self.startNavigating = False
        self.data_path = '/brain/data/locations/lab.dat'
        # /home/borg/sudo/brain/data/locations
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

    def implementation_update(self):
        if self.state == 'enter':
            self.set_goal(self.aim)
            self.state = 'navigate'
            self.startNavigating = True
            self.stuck = self.ab.sublabnavigation({})

        elif self.stuck.is_failed() or self.goto.is_failed():
            print("Alice stuck!!!")
            self.body.say('I am stuck. Try to go to recovery point.')
            self.state = 'recovery'
            self.stuck = self.ab.sublabnavigation({})

            self.closest_points = self.get_closest_all_level_recovery_points()
            print(self.closest_points)

            self.current_recovery_point = self.closest_points.pop(0)
            self.set_goal(self.current_recovery_point)

        elif self.state == 'recovery' and self.check_if_close_to_the_goal(goal=self.current_recovery_point):
            self.set_goal(self.aim)
            self.state = 'navigate'

        elif self.state == 'recovery' and (self.goto.is_failed() or self.stuck.is_failed()):
            self.stuck = self.ab.sublabnavigation({})
            if len(self.closest_points) != 0:
                self.current_recovery_point = self.closest_points.pop(0)
                self.set_goal(self.current_recovery_point)
            else:
                self.set_goal(self.aim)
                self.state = 'navigate'

        elif self.state == 'navigate' and self.goto.is_finished():
            self.body.say('I have arrived')
            self.set_finished()

    def check_if_close_to_the_goal(self, goal=None, x=None, y=None):
        self.transform.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(0.5))
        trans, rot = self.transform.lookupTransform('/map', '/base_link', rospy.Time(0))

        if goal:
            distance = math.pow((self.storedLocations[goal]['x'] - trans[0]), 2) + math.pow(
                (self.storedLocations[goal]['y'] - trans[1]), 2)

            if distance <= 0.2:
                return True
            else:
                return False

        else:
            distance = math.pow((x - trans[0]), 2) + math.pow((y - trans[1]), 2)

            if distance <= 0.5:
                return True
            else:
                return False

    def get_closest_all_level_recovery_points(self, num=3):
        self.transform.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(0.5))
        trans, rot = self.transform.lookupTransform('/map', '/base_link', rospy.Time(0))

        dict_distance = {}
        for i in self.storedLocations:
            if i.startswith('final_demo_recovery'):
                dict_distance[i] = math.pow((self.storedLocations[i]['x'] - trans[0]), 2) + math.pow(
                    (self.storedLocations[i]['y'] - trans[1]), 2)

        sorted_dis = sorted(dict_distance.items(), key=operator.itemgetter(1))

        ind = 1
        points = []
        for key in sorted_dis:
            if ind > num:
                break
            ind += 1
            points.append(key[0])
        return points

    def set_goal(self, goal):
        self.goto = self.ab.gotowrapper({'goal': goal, 'error_range': 0.03})

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

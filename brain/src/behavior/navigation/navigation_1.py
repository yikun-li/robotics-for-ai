from __future__ import print_function

import math
import operator
import os
import random

import basebehavior.behaviorimplementation
import rospy
import tf
from geometry_msgs.msg import PoseStamped


class Navigation_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        self.waypoint = ['1hallway', '1waypoint', '1arena']
        self.depart_point = -1
        self.aim_point = -1
        self.state_back_up = None
        self.stuck_position = None

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
        if self.state == 'go_to_arena' and (self.stuck.is_failed() or self.goto.is_failed()):
            print('Wait for 5s')
            self.state = 'stuck_wait'
            self.time = rospy.Time.now()
            self.set_goal(None)

        elif self.state == 'stuck_wait' and rospy.Time.now() - self.time > rospy.Duration(5):
            self.state = 'recovery'

        if ((self.stuck.is_failed() or self.goto.is_failed()) and self.state.startswith(
                'go_to')) or self.state == 'recovery':
            self.body.say('I am stuck, try to navigate to recovery point.')
            print("Alice stuck!!!")
            print('level 1')
            if self.state == 'recovery':
                self.state_back_up = 'go_to_arena'
            else:
                self.state_back_up = self.state

            self.state = 'level1_recovery'
            self.closest_points = self.get_closest_all_level_recovery_points(num=5)
            print(self.closest_points)
            self.current_recovery_point = self.closest_points.pop(0)
            self.set_goal(self.current_recovery_point)
            self.stuck = self.ab.sublabnavigation({})

        elif self.state == 'level1_recovery' and self.check_if_close_to_the_goal(goal=self.current_recovery_point):
            self.state = 'level2_recovery'
            print('level 2')
            self.closest_l2_points = self.get_closest_all_level_recovery_points(start=self.depart_point,
                                                                                aim=self.aim_point, num=4)
            # random.shuffle(self.closest_l2_points)
            print(self.closest_l2_points)
            self.current_recovery_point = self.closest_l2_points.pop()
            self.set_goal(self.current_recovery_point)

        elif self.state == 'level1_recovery' and (self.goto.is_failed() or self.stuck.is_failed()):
            self.stuck = self.ab.sublabnavigation({})
            if len(self.closest_points) != 0:
                self.current_recovery_point = self.closest_points.pop(0)
                self.set_goal(self.current_recovery_point)
            else:
                print('stuck')
                self.state = 'stuck'
                x = random.random() - 0.5
                y = random.random() - 0.5
                self.stuck_position = self.find_behind_point(x, y)
                self.set_goal(self.stuck_position)

        elif self.state == 'level2_recovery' and self.goto.is_finished():
            self.state = self.state_back_up
            self.set_goal(self.waypoint[self.aim_point])

        elif self.state == 'level2_recovery' and (self.goto.is_failed() or self.stuck.is_failed()):
            self.stuck = self.ab.sublabnavigation({})
            if len(self.closest_l2_points) != 0:
                self.set_goal(self.closest_l2_points.pop())
            else:
                print('stuck')
                self.state = 'stuck'
                x = random.randint(0, 2)
                y = random.randint(0, 2)
                self.stuck_position = self.find_behind_point(x, y)
                self.set_goal(self.stuck_position)

        elif self.state == 'stuck' and self.check_if_close_to_the_goal(x=self.stuck_position['x'],
                                                                       y=self.stuck_position['y']):
            self.state = self.state_back_up
            self.set_goal(self.waypoint[self.aim_point])

        if self.state == 'enter':
            self.state = 'go_to_hall'
            self.depart_point = 0
            self.aim_point = 0
            self.set_goal(self.waypoint[self.aim_point])
            self.startNavigating = True
            self.stuck = self.ab.sublabnavigation({})

        elif self.state == 'go_to_hall' and self.goto.is_finished():
            self.state = 'go_to_way'
            self.depart_point = 0
            self.aim_point = 1
            self.body.say('I am navigating')
            self.set_goal(self.waypoint[self.aim_point])

        elif self.state == 'go_to_way' and self.goto.is_finished():
            self.time = rospy.Time.now()
            self.state = 'wait3'
            self.body.say('I have arrived')

        elif self.state == 'wait3' and rospy.Time.now() - self.time > rospy.Duration(3):
            self.state = 'go_to_arena'
            self.depart_point = 1
            self.aim_point = 2
            self.body.say('I am navigating')
            self.set_goal(self.waypoint[self.aim_point])

        elif self.state == 'go_to_arena' and self.goto.is_finished():
            self.time = rospy.Time.now()
            self.state = 'wait5'
            self.body.say('I have arrived')

        elif self.state == 'wait5' and rospy.Time.now() - self.time > rospy.Duration(5):
            self.state = 'go_to_way2'
            self.depart_point = 2
            self.aim_point = 1
            self.set_goal(self.waypoint[self.aim_point])
            self.body.say('I am navigating')

        elif self.state == 'go_to_way2' and self.check_if_close_to_the_goal(self.waypoint[1]):
            self.state = 'go_to_hall2'
            self.depart_point = 1
            self.aim_point = 0
            self.set_goal(self.waypoint[self.aim_point])
            self.startNavigating = True

        elif self.state == 'go_to_hall2' and self.goto.is_finished():
            self.body.say('I have arrived')
            self.startNavigating = False
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

    def get_closest_all_level_recovery_points(self, start=-1, aim=-1, num=3):
        self.transform.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(0.5))
        trans, rot = self.transform.lookupTransform('/map', '/base_link', rospy.Time(0))

        dict_distance = {}
        for i in self.storedLocations:
            if start == -1 or aim == -1:
                if i.startswith('recovery'):
                    dict_distance[i] = math.pow((self.storedLocations[i]['x'] - trans[0]), 2) + math.pow(
                        (self.storedLocations[i]['y'] - trans[1]), 2)
            else:
                if start == 0:
                    way = 0
                else:
                    way = int(math.ceil((start + aim) / 2.0))
                if i.startswith('recovery_l2_w' + str(way)):
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

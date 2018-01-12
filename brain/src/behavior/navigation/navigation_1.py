'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import math
import os

import basebehavior.behaviorimplementation
import rospy
import tf
from geometry_msgs.msg import PoseStamped


class Navigation_x(basebehavior.behaviorimplementation.BehaviorImplementation):
    '''this is a behavior implementation template'''

    # this implementation should not define an __init__ !!!

    def implementation_init(self):

        self.count = 1
        self.return_state = False

        self.startNavigating = False
        self.data_path = '/brain/data/locations/lab.dat'
        # /home/borg/sudo/brain/data/locations
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
            self.state = 'goto_hall'
            self.set_goal('waypoint')
            self.startNavigating = True

        elif self.state == 'goto_hall' and self.goto.is_finished():
            self.state = 'goto_way'
            self.set_goal('waypoint')
            self.body.say('I am navigating')

        elif self.state == 'goto_hall' and self.goto.is_failed():
            self.set_goal('waypoint')


        elif self.state == 'goto_way' and self.goto.is_finished():
            self.time = rospy.Time.now()
            self.state = 'wait3'
            self.body.say('I have arrived')

        elif self.state == 'goto_way' and self.goto.is_failed():
            self.set_goal('hallway')
            self.body.say('I failed')
            self.body.say('I am navigating')

        elif self.state == 'wait3' and rospy.Time.now() - self.time > rospy.Duration(3):
            self.state = 'goto_arena'
            self.set_goal('arena')
            self.body.say('I am navigating')

        elif self.state == 'goto_arena' and self.goto.is_finished():
            self.time = rospy.Time.now()
            self.state = 'wait5'
            self.body.say('I have arrived')

        elif self.state == 'goto_arena' and self.goto.is_failed():
            self.set_goal('arena')
            self.body.say('I failed')

        elif self.state == 'wait5' and rospy.Time.now() - self.time > rospy.Duration(5):
            self.state = 'goto_way2'
            self.set_goal('waypoint')
            self.body.say('I am navigating')

        elif self.state == 'goto_goal' and self.check_if_close_to_the_goal('waypoint'):
            self.state = 'goal_hall2'
            self.set_goal('hallway')

        elif self.state == 'goto_hall2' and (self.goto.is_finished() or self.goto.is_failed()):
            self.startNavigating = False
            self.body.say('I have arrived')
            self.is_finished()

        pass

    def check_if_close_to_the_goal(self, goal):

        self.transform.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(0.5))
        trans, rot = self.transform.lookupTransform('/map', '/base_link', rospy.Time(0))

        distance = math.pow((self.storedLocations[goal]['x'] - trans[0]), 2) + math.pow(
            (self.storedLocations[goal]['y'] - trans[1]), 2)
        if distance <= 0.5:
            return True
        else:
            return False

    def set_goal(self, goal):
        self.goto = self.ab.gotowrapper({'goal': goal, 'error_range': -1})

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

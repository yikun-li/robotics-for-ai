import basebehavior.behaviorimplementation
import math
import time
import os
import tf
import util.pointcloud_utils
import util.reconfig as reconfig
from util.reconfig.configurations import *
import rospy
# ROTATE MOET TOCH WEL IN GOTOMOVEBASE, KAN NIET MEMORY ACCESSEN VAN UIT MOVEBASE.PY, DUS ALLES WEER OMGOOIEN!

class GotoMoveBase_x(basebehavior.behaviorimplementation.BehaviorImplementation):


    def implementation_init(self):
        
        self.movebase = self.body.movebase()
        self.pioneer = self.body.pioneer(0)
        
        # Data
        self.goalLocation = None        # Dict containing x, y, angle
        self.goalLocationName = ""      # Location name (only if provided)
        self.lastGoal = {}              # Last goal
        self.lastStatus = None          # Last status
        self.isAligned = False          # If robot is aligning to goal orientation
        self.isTurning = False          # If robot is busy turning
        self.startTimeAligning = 0      # Start time alignment
        self.startTimeTurning = 0       # Start time turning
        self.storedLocations = {}       # Dict containing <name: location> pairs
        self.doorConfDistance = 0       # If robot gets close to a door, it switches to the DOOR configuration
        self.tfListener = tf.TransformListener()

        # Read stored locations
        if not hasattr(self, "fileLocations"):
            self.fileLocations = False
            print "No location file specified, no predefined locations are loaded"
        elif len(self.fileLocations) > 0:
            self.read_stored_locations(os.environ['BORG'] + self.fileLocations)
            
        # Checks for turn parameter. Should the robot align to goal?
        if not hasattr(self, "align_to_goal"):
            self.align_to_goal = True
            
        # Check if error range was given
        if not hasattr(self, "error_range"):
            self.error_range = 0.3      # -1 is used as movebase default tolerance value
            
        # Check simulation parameter
        if not hasattr(self, "simulation"):
            self.simulation = False
        if self.simulation:
            self.error_range = -1
            self.align_to_goal = False
            
        # Apply parameters
        self.movebase.set_error_range(self.error_range)
        self.original_align_to_goal = self.align_to_goal
        
        # Flags
        self.cancelGoal = False
        self.goalReached = False
        
        # Params
        self.max_orientation_error = 10     # Maximum orientation error allowed
        self.alignTimeout = 10              # Timeout after which alignment should be done
        self.turnTimeout = 5                # Timeout after which turning should be done
        self.timeGuard = 1                  # Memory time guard (goto goal should be given in the last x seconds)

    def implementation_update(self):
        # Check for doors
        try:
            (x, y, _), _ = self.tfListener.lookupTransform('/base_link', '/map', rospy.Time(0))
            for location in self.storedLocations:
                # if door
                distance = math.sqrt((x*x - 0*0 + y*y - 0*0))
                if distance <= self.doorConfDistance:
                    reconfig.set_config(DOOR)
                    print('set config to DOOR')
                else:
                    if reconfig.current_config == DOOR:
                        reconfig.set_config(DEFAULT)
                        print('set config to DEFAULT')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # Listen for tasks
        if self.m.n_occurs('goto') > 0:
            (recogtime, properties) = self.m.get_last_observation('goto')
            if recogtime > (time.time() - self.timeGuard): # Was the command given in the last X seconds?
                if 'new_goal' in properties and self.goalLocation != properties['new_goal'] and self.goalLocationName != properties['new_goal']:
                    # New goal instruction received
                    self.set_new_goal(properties['new_goal'])
                    # Check for temporary error range parameter
                    if 'error_range' in properties:
                        self.movebase.set_error_range(properties['error_range'])
                    else:
                        self.movebase.set_error_range(self.error_range)
                    # Check for temporary align to goal parameter
                    if 'align_to_goal' in properties:
                        self.align_to_goal = properties['align_to_goal']
                    else:
                        self.align_to_goal = self.original_align_to_goal
                    # Check for align to a certain point parameter
                    if 'align_to_point' in properties and 'x' in properties['align_to_point'] and 'y' in properties['align_to_point']:
                        self.align_to_point = properties['align_to_point']
                        self.align_to_goal = True
                    else:
                        self.align_to_point = False
                    return
                elif 'cancel_goal' in properties and self.goalLocation != None:
                    # Cancel goal instruction received
                    self.cancel_goal()
                    return

        # Listen for move_base feedback
        if self.goalLocation and self.m.n_occurs('move_base') > 0:
            (recogtime, properties) = self.m.get_last_observation('move_base')
            if recogtime > (time.time() - self.timeGuard): #was the command given in the last X seconds?
                if 'status' in properties and self.lastStatus == "goal_received":
                    # Status update of move_base received
                    self.lastStatus = properties['status']
                    if self.lastStatus == "goal_reached":
                        # Reached the goal, check if the robot has to align (only if error_range has been set)
                        if self.error_range != -1 and self.align_to_goal and not self.isAligned and not self.isTurning:
                            # Rotate to goal
                            self.rotate_to_goal()
                            self.startTimeAligning = time.time()
                            return
                        else:
                            # Goal reached and no need for aligning (already done by movebase)
                            self.isAligned = True
                            return
                    elif self.lastStatus == "goal_cancelled" or self.lastStatus == "goal_rejected" or self.lastStatus == "goal_aborted":
                        # Can't reach the goal
                        print "Goal not reached"
                        self.m.add_item('goto', time.time(), {'status': 'goal_cancelled', 'goal_name': self.goalLocationName, 'goal': self.goalLocation})
                        # Reset goal
                        self.reset_goal()
                        return
                elif 'new_goal_received' in properties and self.lastStatus == "goal_given":
                    # New goal instruction received by move base
                    self.lastStatus = "goal_received"
                    return
            elif self.isTurning and not self.isAligned:
                # Check if orientation matches goal's orientation
                self.check_alignment()
                return
            elif self.isAligned:
                # Goal reached and aligned properly
                print "Goal reached"
                self.m.add_item('goto', time.time(), {'status': 'goal_reached', 'goal_name': self.goalLocationName, 'goal': self.goalLocation})
                # Reset goal
                self.reset_goal()
                return
    
    def cancel_goal(self):
        self.movebase.cancel_goal()
        self.reset_goal()
        
    def check_alignment(self):
        """ Checks if robot's current orientation matches goal orientation """
    
        # Get current orientation
        current_location = self.get_current_location()
        if current_location == None:
            self.logger.warning("Can't get current orientation as there are no odometry readings yet. Can't rotate to goal")
            self.isAligned = True
            return
        current_angle = current_location['angle']    
        
        # Get goal orientation
        goal_angle = self.goalLocation['angle']
        
        # Check if alignment timeout is reached
        if (time.time() - self.startTimeAligning) > self.alignTimeout:
            self.isAligned = True
            return
        
        # Check if current angle is close enough to goal angle
        if abs(goal_angle - current_angle) < self.max_orientation_error:
            # Because of noise, perfect alignment is not to be expected. If the difference
            # in angle is less then 10, maybe it's done turning, or has to turn just a
            # little bit more. Therefore, wait a short time to say that the goal is reached.
            time.sleep(1)
            self.isAligned = True
            self.pioneer.stop_robot()
        elif (time.time() - self.startTimeTurning) > self.turnTimeout:
            # If the noise is larger than 10 degrees, check alignment again.
            self.pioneer.stop_robot()
            self.rotate_to_goal()
    
    def reset_goal(self):
        self.goalLocation = None
        self.goalLocationName = ""
        self.lastStatus = None
        self.isAligned = False
        self.isTurning = False
    
    def get_current_goal(self):
        return self.goalLocation
        
    def get_current_location(self):
        reading = self.m.get_last_observation("odometry")
        if reading == None:
            return None
        return reading[1]
        
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
            line = line.rstrip("\n")    # remove endline

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
        
    def rotate_to_goal(self):
        """ Aligns the robot to its goal orientation when error_range is used """

        # Get current orientation
        time_ = time.time()
        while (time.time() - time_ < 1):
            current_location = self.get_current_location()
            #print "Cur location: " + str(current_location)
        
        if current_location == None:
            self.logger.warning("Can't get current orientation as there are no odometry readings yet. Can't rotate to goal")
            self.isAligned = True
            return
        current_angle = current_location['angle']   
        
        # Get goal orientation
        rotate = 0
        if self.align_to_point:
            # Determine angle based on point
            goal_point_dict = {'x': self.align_to_point['x'], 'y': self.align_to_point['y'], 'z': 0}
            point_from_base = util.pointcloud_utils.transform_point(goal_point_dict, "/map", "/base_link")
            # Check if transformation is successful
            if 'x' in point_from_base and 'y' in point_from_base:
                # Determine angle to turn
                rotate = math.degrees(math.atan2(point_from_base['y'], point_from_base['x']))
        else:
            # Rotate to goal orientation
            goal_angle = self.goalLocation['angle']
            rotate = (goal_angle - current_angle) % 360  #-22 - -240 =  218      -360  = -142
            if rotate < -180:
                rotate += 360
            elif rotate > 180:
                rotate -= 360
        
        print "rotating: " + str(rotate)
            
        self.pioneer.turn(rotate)
        self.isTurning = True
        self.startTimeTurning = time.time()
    
    def set_new_goal(self, location):
        """ Sends a navigation goal to move_base """
        
        # Check if location is dict or string
        if isinstance(location, basestring):
            # Location is a string, thus a location name
            # <name> must exist
            if not location.lower() in self.storedLocations:
                print "Location '" + location + "' not found, navigation failed"
                self.m.add_item('goto', time.time(), {'status': 'goal_cancelled', 'goal_name': location, 'goal': {}})
                return
            self.reset_goal()
            self.goalLocationName = location
            self.goalLocation = self.storedLocations[location.lower()]
            print "Navigating towards '" + location + "'"
        elif isinstance(location, dict) and 'x' in location and 'y' in location and 'angle' in location:
            # Location is a proper dict
            self.reset_goal()
            self.goalLocation = location
            self.goalLocationName = "<custom>"
            print "Navigating towards custom location"
        else:
            print "No appropriate navigation goal given, navigation failed"
            self.m.add_item('goto', time.time(), {'status': 'goal_cancelled', 'goal_name': location, 'goal': location})
            return
        
        self.lastStatus = "goal_given"
        self.movebase.set_new_goal(self.goalLocation)
        
    def get_distance_to_goal(self, location):
        """ Given a goal, returns distance """
        
        current_location = self.get_current_location()
        if current_location == None:
            print "Can't get current orientation as there are no odometry readings yet. Can't check distance"
            return False
        
        # Check if location is dict or string
        if isinstance(location, basestring):
            # Location is a string, thus a location name
            # <name> must exist
            if not location.lower() in self.storedLocations:
                print "Location %s unknown" % location
                return False
            
            # Get location
            location = self.storedLocations[location.lower()]
            
        # Determine distance, check if location is dict
        if isinstance(location, dict) and 'x' in location and 'y' in location:
            x_diff = current_location['x'] - location['x']
            y_diff = current_location['y'] - location['y']
            return math.sqrt(x_diff * x_diff + y_diff * y_diff)
        else:
            print "Location %s unknown" % str(location)
            return False
            
            
            
        



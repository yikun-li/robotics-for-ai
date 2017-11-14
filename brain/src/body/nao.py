import logging

import os
import util.nullhandler
import math
from ftplib import FTP, error_perm
import glob
import time

from util.euclid import Vector3 as V3

# Aldebaran Imports
try:
    from naoqi import ALProxy, ALModule, ALBroker
except:
    print "NAOQI Not available. Nao will not work, unless you are using FakeNao"
import motion

logging.getLogger('Borg.Brain.BodyController.Nao').addHandler(util.nullhandler.NullHandler())

class Nao(object):
    """
    Controls a nao
    """

    TO_RAD = math.pi / 180.0 
    ## Minimum and maximum ranges of joints
    __joint_range = {}

    def __init__(self, robot_ip, port=9559, nobody=False):
        self.logger = logging.getLogger('Borg.Brain.BodyController.Nao')
        self.__robot_ip = robot_ip
        self.__port = port
        self.__username = 'nao'
        self.__password = 'nao'
        self.__FM = ALProxy("ALFrameManager", robot_ip, int(port))
        try:
            # Local NaoQi does not have TTS, real robot does
            self.__TTS = ALProxy("ALTextToSpeech", robot_ip, int(port))
            self.__VisionTB = ALProxy("ALVisionToolbox", robot_ip, int(port))
            self.__Sonar = ALProxy("ALSonar", robot_ip, int(port))
            self.__Sonar.subscribe("Sonar",200,0.02)
            self.__BallTracker = ALProxy("ALRedBallTracker", robot_ip, int(port))
            self.simulation = False
        except:
            self.__TTS = None
            self.__VisionTB = None
            self.__Sonar = None
            self.__BallTracker = None
            self.simulation = True
        self.__Motion = ALProxy("ALMotion", robot_ip, int(port))
        self.__Memory = ALProxy("ALMemory", robot_ip, int(port))
        self.__Video = ALProxy("ALVideoDevice", robot_ip, int(port))
        self.__Leds = ALProxy("ALLeds", robot_ip, int(port))
        self.__behaviorIDs = {}
        self.__stop_crouch = True
        self.__nobody = nobody

        # Enable TTS notifications, just in case (so we can determine if the nao is currently speaking or not):
        if not self.__TTS == None:
            try:
                self.__TTS.enableNotifications()
            except Exception as e:
                #TODO: Verify that the generated exception is catched only because notifications is already enabled.
                print e
        
        #Create LED Groups for NAO eyes or ears
        self.setLedsGroup()

    def is_bumper_pressed(self):
        return self.__Memory.getData("RightBumperPressed", 0) == 1.0 or self.__Memory.getData("LeftBumperPressed", 0) == 1.0

    def tasks_finished(self):
        print "Tasklist"
        print self.__Motion.getTaskList()
        if self.__Motion.getTaskList():
            return False
        return True

    def __del__(self):
        self.logger.info("NAO controller stopping, de-enslaving NAO")
        self.set_stifness(['Body'], [0], [0.25])
        if self.__Sonar:
            self.__Sonar.unsubscribe("Sonar")
        #self.__broker.shutdown()

    def stop(self):
        if self.__nobody:
            return

        if self.__stop_crouch:
            self.sit_down()
        else:
            self.start_behavior('sitdown', True)
            time.sleep(10)
        print "De-enslaving Nao"
        self.set_stifness(['Body'], [0], [0.25])
    def setLedsGroup(self, names = None):
        if not names:
            # Create a new group
            names = [
            "Face/Led/Red/Left/0Deg/Actuator/Value",
            "Face/Led/Red/Left/90Deg/Actuator/Value",
            "Face/Led/Red/Left/180Deg/Actuator/Value",
            "Face/Led/Red/Left/270Deg/Actuator/Value",
            "Face/Led/Red/Right/0Deg/Actuator/Value",
            "Face/Led/Red/Right/90Deg/Actuator/Value",
            "Face/Led/Red/Right/180Deg/Actuator/Value",
            "Face/Led/Red/Right/270Deg/Actuator/Value"]
            self.__Leds.createGroup("MyGroup",names)
            
    def set_crouch_on_stop(self, crouch=True):
        self.__stop_crouch = crouch

    def move(self, Joint, Angle, Speed):
        self.__Motion.setAngles(Joint, Angle, Speed)

    def walk(self, X=0, Y=0, Teta=0):
        self.__Motion.walkTo(X, Y, Teta)
        
    def stopwalk(self):
        self.__Motion.stopWalk()
        
    def setWalkTargetVelocity(self, x , y, theta, frequency):
        self.__Motion.setWalkTargetVelocity(x , y, theta, frequency)

    def is_speaking(self):
        if self.simulation:
            return False
        else:
            is_done_speaking = self.__Memory.getData("ALTextToSpeech/TextDone")
            if not is_done_speaking == None:
                is_done_speaking = int(is_done_speaking)
                if is_done_speaking == 0:
                    return True
            return False

    def say(self, Text, filename=None):
        if self.__TTS:
            #self.__TTS.say(str(Text))
            if filename:
                self.__TTS.sayToFile(Text, filename)
            else:
                self.__TTS.post.say(Text)
        else:
            print "[Nao says] " + Text


    def get_sonar_distance(self):
#        sonarValue= "SonarLeftDetected"
        if self.__Sonar:
#            data = self.__Sonar.getOutputNames()
            data = {"left": self.__Memory.getData("SonarLeftDetected",0), "right" : self.__Memory.getData("SonarRightDetected",0)}
            return data

    def get_yaw_pitch(self, radians=True):
        #Get Nao's yaw and pitch neck angles in RADIANS:
        names  = "Body"
        useSensors  = True
        sensorAngle = self.__Motion.getAngles(names, useSensors)
        headYaw = sensorAngle[0]
        headPitch = sensorAngle[1]
        # If radians is false, return degrees
        if not radians:
            headYaw = headYaw / self.TO_RAD
            headPitch = headPitch / self.TO_RAD
        return [headYaw, headPitch]

    def get_robot_ip(self):
        '''
        Returns the IP address as specified in the constructor.
        '''
        return self.__robot_ip

    def get_port(self):
        '''
        Returns the port as specified in the constructor.
        '''
        return self.__port

    def start_behavior(self, behaviorname, local=False):
        """
        Start a behavior from choregraph which is stored on the robot.
        The local parameter specifies that the file should be loaded from the
        local filesystem instead of from the robot.
        """
        behaviorID = self.get_behavior_id(behaviorname, local)
        if behaviorID:
            self.__FM.playBehavior(behaviorID)

    def complete_behavior(self, behaviorname, local=False):
        """
        Start a behavior from choregraph which is stored on the robot. Waits for
        the behavior to finish. The behavior should call it's output, otherwise
        this method will get stuck
        The local parameter specifies that the file should be loaded from the
        local filesystem instead of from the robot.
        """
        behaviorID = self.get_behavior_id(behaviorname, local)
        if behaviorID:
            self.__FM.completeBehavior(behaviorID)

    def get_behavior_id(self, behaviorname, local=False):
        """
        Before a xml file can be run, a behavior has to be created. If this
        was already done before, just get the behavior id from the `behaviorIDs'
        dictionary. Otherwise, create the behavior right now.
        The local parameter specifies that the file should be loaded from the
        local filesystem instead of from the robot.
        """
        path = "" # path within the xml file
        if local:
            if self.simulation:
                # Use FrameManager when using a simulated robot as FTP won't work
                xmlfile = os.environ['BORG'] + "/Brain/Choregraphs/" + behaviorname + "/behavior.xar"
                self.logger.debug("Loading contents of local file %s as behavior %s" % (xmlfile, behaviorname))
                try:
                    contents = open(xmlfile, 'r').read()
                    id = self.__FM.newBehavior(path, contents)
                    self.__behaviorIDs[behaviorname] = id
                    return id
                except:
                    self.logger.error("Unable to load contents of local file %s - does it exist?" % xmlfile);
            else:
                # Use FTP to transfer behavior to robot as its much faster
                if not self.store_behavior_on_nao(behaviorname):
                    self.logger.error("Unable to send behavior to Nao. Not executing")
                    self.say("I cannot load the requested behavior")
                    return None
        xmlfile = "/home/nao/behaviors/" + behaviorname + "/behavior.xar"
        self.logger.debug("Loading contents of file %s on the robot as behavior %s" % (xmlfile, behaviorname))
        id = self.__FM.newBehaviorFromFile(xmlfile, path)
        print id
        self.__behaviorIDs[behaviorname] = id
        return id

    def store_behavior_on_nao(self, behaviorname):
        """
        This method will store a new behavior on the Nao by sending it over
        FTP. This works much faster than using FrameManager.
        """
        dirname = os.getenv("BORG") + "/Brain/Choregraphs/" + behaviorname
        filename = os.getenv("BORG") + "/Brain/Choregraphs/" + behaviorname + "/behavior.xar"

        if not os.path.exists(dirname):
            self.logger.error("Path to behavior (%s) does not exist" % dirname)
            return False

        if not os.access(dirname, os.X_OK) or \
           not os.access(dirname, os.R_OK):
            self.logger.error("Cannot open directory (%s)" % dirname)
            return False

        if not os.path.exists(filename):
            self.logger.error("Behavior file (%s) does not exist" % filename)
            return False

        if not os.access(filename, os.R_OK):
            self.logger.error("Behavior file (%s) is not readable" % filename)

        filelist = glob.glob(dirname + "/*")
            
        try:
            ftp = FTP(self.__robot_ip)
            ftp.login(self.__username, self.__password)
            ftp.set_pasv(True)
        except:
            self.logger.error("Unable to connecto to Nao's FTP server. Cannot " \
                              "send behavior")
            return False

        try:
            ftp.mkd('behaviors')
        except error_perm:
            pass

        try:
            ftp.cwd('behaviors')
        except:
            self.logger.error("Remote behaviors directory is not accessible.")
            return False

        try:
            ftp.mkd(behaviorname)
        except error_perm:
            pass

        try:
            ftp.cwd(behaviorname)
        except:
            self.logger.error("Remote directory (%s) does not exist and " \
                              "could not be created or is not accessible" \
                              % behaviorname)
            ftp.close()
            return False

        for filename in filelist:
            try:
                f = open(filename, 'rb')
            except IOError:
                self.logger.warning("Cannot open file %s for reading, " \
                                    "skipping" % filename)
                continue

            self.logger.info("Uploading file %s to Nao" % filename)
            basename = os.path.basename(filename)
            try:
                ftp.storbinary('STOR %s' % basename, f)
            except:
                self.logger.error("Could not store file (%s) on Nao" % filename)
                f.close()
                ftp.close()
                return False
            self.logger.info("Succesfully uploaded file %s to Nao" % filename)
            f.close()

        ftp.close()
        return True

    def set_stiffness(self,
                      names=['Body', 'Body', 'Body'],
                      stiffnessLists=[0.25, 0.5, 1.0],
                      timeLists=[0.5, 1.0, 1.5]):
        self.set_stifness(names, stiffnessLists, timeLists)

    def set_stifness(self,
                     names=['Body', 'Body', 'Body'],    # Part of the robot to apply to
                     stiffnessLists = [0.25, 0.5, 1.0], # Trajectory of stiffness levels
                     timeLists = [0.5, 1.0, 1.5]):      # Time
        """
        The stiffnessLiss is a list of stiffness levels. Each entry sets the
        stiffnesslevel that should be set at the time specified in the same
        element in the timeLists
        """
        for i in range(len(names)):
            self.__Motion.stiffnessInterpolation(names[i], stiffnessLists[i], timeLists[i])

    def get_range(self, name, radian=False):
        """
        This method wraps the getLimits function of naoqi; it caches the results
        because each call to ALMotion.getLimits takes a lot of time
        """
        if not name in self.__joint_range:
            limits = self.__Motion.getLimits(name)
            self.__joint_range[name] = limits[0]

        val = self.__joint_range[name]
        if not radian:
            val = (val[0] / self.TO_RAD,
                   val[1] / self.TO_RAD)

        return val

    def change_angles(self, names, angles, max_speed, disable_stiffness=False, radians=False):
        """
        This method will change the angles for the joints in the list of names.
        To make sure the joints actually move, stiffness is set on these joints.
        CAUTION: This method sometimes results in very sudden movements of the NAO.
                 If you experience this and want to avoid it, call set_stiffness
                 on these joints before calling set_angles, as the sudden motion
                 results from the stifness being increased in a very short
                 amount of time. set_stiffness allows to do this in a more
                 subtle fashion.
        """
        if not radians:
            angles = [x * self.TO_RAD for x in angles]

        # Perform te movement
        self.__Motion.changeAngles(names, angles, max_speed)

    def set_angles(self, names, angles, max_speed, disable_stiffness=False, radians=False):
        """
        This method will set the angles for the joints in the list of names.
        To make sure the joints actually move, stiffness is set on these joints.
        CAUTION: This method sometimes results in very sudden movements of the NAO.
                 If you experience this and want to avoid it, call set_stiffness
                 on these joints before calling set_angles, as the sudden motion
                 results from the stifness being increased in a very short
                 amount of time. set_stiffness allows to do this in a more
                 subtle fashion.
        """
        if not radians:
            angles = [x * self.TO_RAD for x in angles]

        # Perform te movement
        self.__Motion.setAngles(names, angles, max_speed)

    def get_angles(self, names, radians=False, use_sensors=False):
        useSensors  = False     # Cannot use sensors in simulation :(
        angles = self.__Motion.getAngles(names, useSensors)
        if not radians:
            angles = [x / self.TO_RAD for x in angles]
        return angles

    def open_hand(self, hand):
        return self.__Motion.openHand(hand)

    def close_hand(self, hand):
        return self.__Motion.closeHand(hand)

    def get_proxy(self, which = "motion"):
        if which == "motion":
            return self.__Motion
        elif which == "tts":
            return self.__TTS
        elif which == "video":
            return self.__Video
        elif which == "frame":
            return self.__FM
        elif which == "memory":
            return self.__Memory
        elif which == "vision":
            return self.__VisionTB
        elif which == "balltracker":
            return self.__BallTracker

    def emergency(self):
        """Disable the NAO stiffness so it stops moving"""
        self.logger.warn("Emergency button pressed")
        self.set_stifness(['Body'], [0], [0.25])
        #self.say("My emergency button has been pressed. I am now in emergency mode")
    
    def emergencyLeds(self, mode):
        
        if mode:
            # Switch the new group on
            self.__Leds.on("MyGroup")
        else:
            # Switch the new group on
            self.__Leds.off("MyGroup")
    
    def point_to_object(self, headYaw, point_down=False, radian=False):   
        """
        This method makes the Nao point in the direction it is currently facing.
        """    
        if radian:
            headYaw = headYaw / self.TO_RAD
            
        # Determine whether left or right arm must be used and set stiffness
        side = "L" if headYaw >= 0 else "R"
        #self.set_stiffness([side+"Arm"], [1.0], [0.5])  
        self.set_stiffness(["RArm"],[1.0],[0.5])
        self.set_stiffness(["LArm"],[1.0],[0.5])
        self.__Motion.openHand(side+"Hand")  
        
        if headYaw >= 76:
            print "Point angle out of reach: " + str(headYaw) + ". Pointing to 75 degrees"
            headYaw = 75
        elif headYaw <= -76:
            headYaw = -75
            print "Point angle out of reach: " + str(headYaw) + ". Pointing to -75 degrees"
        
        # Determine joint angles
        shoulderPitch = (0 if not point_down else 45)
        shoulderRoll = headYaw
        elbowRoll = 16 * (-1 if headYaw >= 0 else 1)        # turn elbow joint 16 degrees inwards, necessary to fix head/shoulder offset
        wristYaw = 60 * (-1 if headYaw >= 0 else 1)         # turn wrist, so hand looks like it is in 'grabbing' position
        
        # Send command to change joint angles
        jointAngles  = [shoulderPitch, shoulderRoll, 0, elbowRoll, wristYaw, 1]
        jointAngles = [x * self.TO_RAD for x in jointAngles]
        pMaxSpeedFraction = 0.2
        self.__Motion.angleInterpolationWithSpeed(side+"Arm", jointAngles, pMaxSpeedFraction)

    def init_pose(self, kneeAngle=20, torsoAngle=0, wideAngle=0, start_wide=False, skip_legs=False):
        """
        This method initializes the grabbing pose. It is adapted from the 
        example in the Aldebaran NAO documentation
        """
        # Enable stiffness to move
        if skip_legs:
            self.set_stiffness()
        else:
            self.set_stifness(names=['LArm', 'RArm', 'Head'],
                              stiffnessLists=[1.0, 1.0, 1.0],
                              timeLists=[0.5, 0.5, 0.5])

        ###############################################################################
        # PREPARE THE ANGLES
        ###############################################################################
        # Define The Initial Position
        Head     = [0, 0]
        if start_wide:
            LeftArm  = [0,  90, -30, -20]
        else:
            LeftArm  = [75,  15, -30, -20]
        
        if skip_legs:
            wideAngle = 0
            kneeAngle = 2.16 / self.TO_RAD
            torsoAngle = 0

        LeftLeg  = [0,  wideAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2, -wideAngle]
        RightLeg = [0, -wideAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2,  wideAngle]

        if start_wide:
            RightArm = [0, -90,  30,  20]
        else:
            RightArm = [75, -15,  30,  20]
        
        if start_wide:
            LeftArm  += [-90, 1 / self.TO_RAD]
            RightArm += [90, 1 / self.TO_RAD]
        else:
            LeftArm  += [-60, 1 / self.TO_RAD]
            RightArm += [60, 1 / self.TO_RAD]

        # Gather the joints together
        pTargetAngles = Head + LeftArm + LeftLeg + RightLeg + RightArm

        # Convert to radians
        pTargetAngles = [x * self.TO_RAD for x in pTargetAngles]

        ###############################################################################
        # SEND THE COMMANDS
        ###############################################################################
        # We use the "Body" name to signify the collection of all joints
        pNames = "Body"

        # We set the fraction of max speed
        pMaxSpeedFraction = 0.2

        # Ask motion to do this with a blocking call
        self.__Motion.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)

        # Disable stiffness of the arms, but not the legs
        self.set_stifness(['LArm', 'RArm', 'Head'], [0, 0, 0], [0.25, 0.25, 0.25])

    def sit_down(self):
        """
        This method lets the NAO sit down in a stable crouching position and 
        removes stiffness when done.
        """
        return 
        # Enable stiffness to move
        self.set_stifness(['Body'], [0.25], [0.25])

        ###############################################################################
        # PREPARE THE ANGLES
        ###############################################################################
        # Define The Sitting Position
        pNames  = ['LAnklePitch', 'LAnkleRoll', 'LHipPitch', 'LHipRoll', 'LHipYawPitch', 'LKneePitch', 
                   'RAnklePitch', 'RAnkleRoll', 'RHipPitch', 'RHipRoll', 'RHipYawPitch', 'RKneePitch',
                   'LShoulderPitch', 'RShoulderPitch', 'HeadYaw', 'HeadPitch']

        pAngles = [        -1.21,        0.036,      -0.700,     -0.027,         -0.143,         2.16,
                           -1.21,       -0.036,      -0.700,      0.027,         -0.143,         2.16,
                            0.96,         0.96,           0,          0]


        ###############################################################################
        # SEND THE COMMANDS
        ###############################################################################
        # We set the fraction of max speed
        pMaxSpeedFraction = 0.2

        # Ask motion to do this with a blocking call
        self.__Motion.angleInterpolationWithSpeed(pNames, pAngles, pMaxSpeedFraction)

        # Disable stiffness 
        self.set_stifness(['Body'], [0], [0.25])

    def look_straight(self):
        self.set_angles(['HeadYaw', 'HeadPitch'], [0, 0], 0.2, radians=True)

    def localize_object_in_image(self, rect, distance=None, width=None, camera=0, lookat=True, space=motion.SPACE_NAO):
        """
        This method will take a rectangle in relative measures, between 0 and 1,
        and will calculate the coordinates in the given Space, based on the distance
        specified.

        The result is a position in the selected space(motion.SPACE_NAO, motion.SPACE_TORSO 
        or motion.SPACE_WORLD). All are, initially:
        - the x axis forwards from the Nao
        - the y axis to the right of the Nao
        - the z axis upwards from the Nao

        Nao Space has the origin at the ground between the legs. Torso Space has
        origin in the center of the torso. World Space has the origin at the
        ground between the legs and is left behind as the Nao moves.

        Parameters:
        rect: The rectangle on the image (in relative coordinates) where the
              object is
        distance: The distance in centimeters to the object, in Torso space
        camera: Which camera is being used; 0 for the forward camera, 1
                for the alternative camera.
        lookat: When True, the Nao will look at the object specified
        space: The space in which to return the coordinates, either motion.SPACE_NAO, 
               motion.SPACE_TORSO or motion.SPACE_WORLD
        """
        if not width and not distance:
            raise Exception("No distance and no object width specified; " \
                            + "one is required to calculate position")
        ###############################################################################
        # SET UP PARAMETERS
        ###############################################################################

        # Get the physical location and orientation of the camera
        if 0 == camera: # Head camera
            CAMERA = "CameraTop"
        else:           # Chin camera
            CAMERA = "CameraBottom"

        # Axes for convenience
        axis_x = V3(1, 0, 0)
        axis_y = V3(0, 1, 0)
        axis_z = V3(0, 0, 1)

        # Get camera position from the Aldebaran Motion proxy
        CAMERA_X, CAMERA_Y, CAMERA_Z, CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW = \
            self.__Motion.getPosition(CAMERA, space, True)

        # We use distances in centimeters, Motion proxy returns in meters
        camera_pos = V3(CAMERA_X * 100, CAMERA_Y * 100, CAMERA_Z * 100)

        # Calculate corners (coordinates in relative values in image)
        left, top, rel_width, rel_height = rect
        corners = [(left, top), 
                   (left, top + rel_height),
                   (left + rel_width, top),
                   (left + rel_width, top + rel_height)]

        ###############################################################################
        # CALCULATE OBJECT POSITION IN SELECTED SPACE
        ###############################################################################

        # If the width is specified, calculate the distance to the object
        if width:
            left_top = corners[0]
            right_top = corners[2]
            
            l_yaw, l_pitch = self.__Video.getAngPosFromImgPos([left_top[0], left_top[1]])
            r_yaw, r_pitch = self.__Video.getAngPosFromImgPos([right_top[0], right_top[1]])

            l_sel_space = V3(1, 0, 0).rotate_around(axis_z, l_yaw + CAMERA_YAW)     \
                                     .rotate_around(axis_y, r_pitch - CAMERA_PITCH) \
                                     .rotate_around(axis_x, CAMERA_ROLL)
            r_sel_space = V3(1, 0, 0).rotate_around(axis_z, r_yaw + CAMERA_YAW)     \
                                     .rotate_around(axis_y, r_pitch - CAMERA_PITCH) \
                                     .rotate_around(axis_x, CAMERA_ROLL)

            # Calculate the distance based on the calculated points
            distance = l_sel_space.x * (width / abs(l_sel_space.y - r_sel_space.y))

        # When the distance is known, calculate the distance to the camera
        x_distance = distance - camera_pos.x

        min_x = min_y = min_z = float('inf')
        max_x = max_y = max_z = float('-inf')
        for c in corners:
            yaw, pitch = self.__Video.getAngPosFromImgPos([c[0], c[1]])

            # Rotate according to yaw, pitch and roll of camera
            obj_sel_space = V3(1, 0, 0).rotate_around(axis_z, yaw + CAMERA_YAW)     \
                                       .rotate_around(axis_y, pitch + CAMERA_PITCH) \
                                       .rotate_around(axis_x, CAMERA_ROLL)

            # Calculate and set distance, if specified
            factor = x_distance / obj_sel_space.x
            obj_sel_space = (obj_sel_space * factor) + camera_pos

            # Find extremes of object position
            min_x = min(obj_sel_space.x, min_x)
            max_x = max(obj_sel_space.x, max_x)
            min_y = min(obj_sel_space.y, min_y)
            max_y = max(obj_sel_space.y, max_y)
            min_z = min(obj_sel_space.z, min_z)
            max_z = max(obj_sel_space.z, max_z)

        # Calculate dimensions
        width  = abs(max_y - min_y)
        height = abs(max_z - min_z)
        distance = min_x

        # Calculate position of the center of the object
        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0
        center_z = (min_z + max_z) / 2.0
        position = V3(center_x, center_y, center_z)

        # Look at the object
        if lookat:
            # Calculate angles of center of object in camera space
            center_h = (corners[2][0] + corners[0][0]) / 2.0
            center_v = (corners[1][1] + corners[0][1]) / 2.0
            self.look_at(center_h, center_v)

        # Construct return value
        cur_object = {"position": (position.x, position.y, position.z),
                      "width"   : width,
                      "height"  : height,
                      "distance": distance}
        return cur_object

    def look_at(self, x, y):
        """
        This method looks at the relative image position ([0-1]) in the camera
        image.
        """
        center_yaw, center_pitch = self.__Video.getAngPosFromImgPos([x, y])

        # Get orientation of the head, used to look at the object
        HEAD_YAW, HEAD_PITCH = self.get_angles(['HeadYaw', 'HeadPitch'], True)

        yaw = HEAD_YAW + center_yaw
        pitch = HEAD_PITCH + center_pitch
        self.set_angles(['HeadYaw', 'HeadPitch'], [yaw, pitch], 0.2, radians=True)
        


#########
# NOTES #
#########

# Nao's head movement range in RADIANS when reading them directly from ALMemory:
#
# YAW:
# left   (+):  2.08566856384
# right  (-): -2.08566856384
#
# PITCH:
# Bottom (+):  0.514872133732
# Top    (-): -0.671951770782
#
# The robot needs to have the head pitch at least
# at 0.06 radians to be able to turn the head
# completely to both sides (left or right). If its
# head pith is smaller (i.e. looking further up) the
# ethernet cable in his head will get stocked with
# its shoulders.
#
# Changes in the order of 1/1000 of a radian (aprox 0.057 degrees)
# are not noticeable anymore, it is not necessary to
# be more precise than that when giving it commands.
#

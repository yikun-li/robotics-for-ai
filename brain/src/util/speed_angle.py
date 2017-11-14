import math
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Util.SpeedAngle').addHandler(util.nullhandler.NullHandler())
"""
This class converts coordinates of detected objects
to speed values and x angle for the moving the pioneer behaviors.

It is useful if you want to use pioneer.head_direction(speed, angle)
in your behavior
"""
    
class SpeedAngle():
    
    __width = None
    __height = None
    __camera_angle = None
    __focal_distance = None
    
    def __init__(self, cameratype=None, width=None, height=None):
        """
        Constructor
        Set cameratype = None if you want to specify custom W+H
            @param cameratype - type of camera (HD, kinect)
            @param width - width of camera
            @param height - height of camera
        """
        self.logger = logging.getLogger('Borg.Brain.Util.SpeedAngle')
        if cameratype == 'HD':
            self.__width = 1280
            self.__height = 720
            #Angle calculated for the logitech HD-c510 (aprox 48.45 degrees on the horizontal):
            self.__camera_angle = 27.307594906640663
        elif cameratype == 'kinect':
            self.__width = 320
            self.__height = 420
            self.__camera_angle = 45 #THIS IS JUST SOME NUMBER !!!!!!!!!!!!!!
            
        elif cameratype == None and width != None and height != None:
            self.__width = width
            self.__height = height
            self.__camera_angle = 60 # just a hardcoded guess, works :)
        else:
            self.logger.error("This camera type is not supported...")
            self.logger.error("Possible values are: 'HD', 'kinect' or you can specify (None, width, height)")
            self.__del__()
            return
            
        self.__compute_focal_distanceance()

    def __del__(self):
        """ D3strukt00r """
        self.camera = None
    
    # Getterz
    def get_width(self):
        return self.__width
    def get_height(self):
        return self.__height
    def get_focal_distanceance(self):
        return self.__focal_distance

    # Funktzionen
    def __compute_focal_distanceance(self):
        """ Sets the focal distance """
        hypot = math.hypot(self.__width / 2, self.__height / 2)
        angle_h = math.radians(self.__camera_angle)
        self.__focal_distance = hypot / math.tan(angle_h)
    
    def get_x_angle(self, xCoord):
        """ Returns horizontal angle in degrees from the center of the camera """
        radians = math.atan2(xCoord - (self.__width / 2), self.__focal_distance)
        degrees = math.degrees(radians)
        return degrees
        
    def get_y_angle(self, yCoord):
        """ Returns vertical angle in degrees from the center of the camera """
        radians = math.atan2(yCoord - (self.__height / 2), self.__focal_distance)
        degrees = math.degrees(radians) 
        return degrees      
            
    def get_speed(self, obj_width, obj_height):
        """ 
        Returns a speed value normalized between [0,1],
        according to the size of the object in the image
        """
        objsize = ( obj_width * obj_height )
        camerasize = ( self.__width * self.__height )
               
        return ( 1.0 - ( float(objsize) / float(camerasize) ) )
    
    def set_parameters(width, height, camera):
        if width == None or height == None or camera == None:
            self.logger.error("Please specify all values")
            return
        else:
            self.__width = width
            self.__height = height
            self.__camera_angle = camera
            
            self.__compute_focal_distanceance()
                

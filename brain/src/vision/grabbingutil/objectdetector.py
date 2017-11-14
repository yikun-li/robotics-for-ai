import cv
import sys
from geometry import Rectangle
from segmentizer import HistogramSegmentizer

class ObjectDetector:
    
    def __init__(self):
        self.threshold = (25, 25, 25, 0)
        self.resolution = (640, 480)
        #todo: make some training method for these values, especially self.resolution[1] * 0.9
        self.region_of_interest = Rectangle(0, 0, self.resolution[0], self.resolution[1] * 0.7)
        self.verbose = False
        
        #todo: same goes for these two points
        self.left_arm_point = (240, 300)
        self.right_arm_point = (410, 300)
        self.mininum_surface = 220
        
    def __detect(self, img):
        '''Detect objects in the image. It will classify the left and
        right arms and filter out small 'objects'. Returns an instance
        of Observation.'''
        storage = cv.CreateMemStorage(0)
        contours = cv.FindContours(img, storage, cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_SIMPLE)
        observation = self.Observation()
        
        # for every group of lines
        while contours:
            area = abs(cv.ContourArea(contours))
            
            rect = Rectangle.from_cv_rect(cv.BoundingRect(contours))
	    rect.contour = contours
            contours = contours.h_next()
        
            # Find the arms based on their expected position. If the found
            # rectangle contains the pixel, it might be the arm you're looking
            # for.
            
            #small stuff, which has almost no surface, is ignored
            if area < self.mininum_surface:
                continue
            
            #left arm
            elif rect.contains(self.left_arm_point):
                observation.left_arm = rect
        
            #right arm
            elif rect.contains(self.right_arm_point):
                observation.right_arm = rect
            
            #anything else that isn't eeny teeny tiny small, is an object.
            else:
                observation.objects.append(rect)
            
        return observation
     
    def __preprocess(self, img):
        # smoothing the image to remove noise that was present in (and created by) the 
        # gradient, and the sum of the gradient and the original image
        smoothed = cv.CreateImage(self.resolution, img.depth, img.channels)
        cv.Smooth(img, smoothed, cv.CV_GAUSSIAN, 3, 3)

        if self.verbose:
            cv.ShowImage("img_objects", img)
        
        # Cut off the NAO itself, no need to detect it or next to it as
        # there is no possible way the object is floating next to the
        # table, or the table is in the same position as the Nao. If this
        # would be true, then Arnoud and Jelmer are not the ones who wrote
        # this software.
        cv.SetImageROI(img, self.region_of_interest.as_cv_rect())
        cv.ShowImage("image",img)
        
        return img
    
    def detect(self, img):
        obj_img = self.__preprocess(img)
        return self.__detect(obj_img)

    class Observation:
        '''Structure to store the object observation'''

        def __init__(self):
            self.left_arm = None
            self.right_arm = None
            self.objects = []

        def __repr__(self):
            return "<Observation [left_arm: %s] [right_arm: %s] [objects: %s]>" % (self.left_arm, self.right_arm, self.objects)

        def draw(self, img):
            '''Draw the observation into an image. Left and right arm are colored red and green. Objects are white'''
            if self.left_arm:
                self.__draw_rect(img, self.left_arm, (0, 0, 255, 0))
            if self.right_arm:
                self.__draw_rect(img, self.right_arm, (0, 255, 0, 0))
            for obj in self.objects:
                self.__draw_rect(img, obj, (255,255,255,0))

        def __draw_rect(self, display, rect, colour):
            '''Little helper function that actually draws the rectangles'''
            cv.Rectangle(display, rect.top_left, rect.bottom_right, colour, 3, cv.CV_AA)
            cv.Circle(display, rect.center, 3, (255, 255, 0, 0), 2, cv.CV_AA)

	    for i,p in enumerate(rect.contour):
		if i == 0:
		    continue
		cv.Line(display, rect.contour[i - 1], p, colour, 1, cv.CV_AA)

if __name__ == "__main__":
    segmentizer = HistogramSegmentizer()
    
    detector = ObjectDetector()
    
    for img_path in sys.argv[1:]:
        print "Image %s" % img_path
        img = cv.LoadImage(img_path, cv.CV_LOAD_IMAGE_GRAYSCALE)
        
        layers = segmentizer.segmentize(img)
        
        if not layers:
            print "Segmentizer failed"
            continue
        
        observation = detector.detect(layers['objects'])
        print observation
    
        #display = clone_color_image(img)
        display = cv.LoadImage(sys.argv[2])
        observation.draw(display)
        
        cv.Circle(display, detector.left_arm_point, 3, (0, 0, 255, 0), 2, cv.CV_AA)
        cv.Circle(display, detector.right_arm_point, 3, (0, 255, 0, 0), 2, cv.CV_AA)
        
        cv.ShowImage("observation", display)
        
        if cv.WaitKey(0) == 27:
            break

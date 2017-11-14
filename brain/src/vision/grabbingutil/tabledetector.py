import cv
import sys
import math
from geometry import Point
from segmentizer import HistogramSegmentizer
from util import clone_color_image

#from scipy import polyfit
# Since I don't have scipy I made a polyfit-compatible function that uses
# opencv FitLine. It works, but I don't know if it is faster or better or
# worse or anything.
def polyfit(x, y, number_of_variables):
    line = cv.FitLine(
        [(x[n], y[n]) for n in range(len(x))],
        cv.CV_DIST_L2,
        0, 0.01, 0.01)
    
    a = line[1] / line[0]
    b = line[3] - a * line[2]
    return (a, b)

class TableDetector:
    
    def __init__(self):
        self.verbose = True
        self.outlier_threshold = 50

    def detect(self, img):
        (dx, dy, width, height) = cv.GetImageROI(img)

        # generate a set of points
        points = [Point(x, height - 1) for x in range(0, width - 1, 5)]

        # move points as far up as possible
        self.__drop_points(points, img)

        points = self.__remove_unmoved_points(points, height - 1)

        # remove points that got stuck in noise
        points = self.__remove_outliers(points, self.outlier_threshold)

        if self.verbose:
            self.__draw_verbose(img, points)

        # if there are no points left (like, shitty image or no table)
        # return nothing
        if len(points) == 0:
            return None

        table = self.__extract_table(points, (width, height))
        
        if self.verbose:
            print table

        return table

    def __draw_verbose(self, bin_img, points):
	seg_img = clone_color_image(bin_img)
	
        if len(points):
            # an extrapolated line!
            (a,b) = polyfit([p.x if p.x > 0 else 1 for p in points], [p.y for p in points], 1)
            f = lambda x: a * x + b

            (dx, dy, width, height) = cv.GetImageROI(seg_img)

            # draw the line (by drawing points, how inefficient!)
            for x in range(width):
                y = f(x)
                cv.Circle(seg_img,
                    (int(dx + x), int(dy + y)),
                    2,
                    (0, 0, 255,0),
                    2,
                    cv.CV_AA)

        # calculate the color of the points
        for idx, point in enumerate(points):
            if idx == 0 or idx == len(points) - 1:
                setattr(point, "colour", (255,255,255,0))
            else:
                setattr(point, "colour", (
                    point.distance(points[idx - 1]) * 2,
                    255,
                    point.distance(points[idx + 1]) * 2,
                    0))

        # draw the individual pionts
        for point in points:
            cv.Circle(seg_img,
                point.move((dx,dy)).as_cv_point(),
                3,
                point.colour,
                2,
                cv.CV_AA)

        cv.ShowImage("TableDetector", seg_img)
        cv.WaitKey(10)

    def __drop_points(self, points, seg_img):
        (dx, dy, width, height) = cv.GetImageROI(seg_img)
        for point in points:
            for y in range(point.y - 1, 0, -1):
                channel = cv.Get2D(seg_img, dy + y, dx + point.x)
                #prev_channel = cv.Get2D(seg_img, dy + y + 1, dx + point.x)
                #diff = channel[0] - prev_channel[0]
                #if diff > 1:
                #seg_img is binary, so values should be 0 or 255
                if channel[0] > 128: 
                    break

                point.y = y

    def __remove_unmoved_points(self, points, initial_y):
        return [pt for pt in points if pt.y != initial_y]

    def __remove_outliers(self, points, threshold = 50):
        '''Remove the outliers by fitting an ax * b line on the data points, and
        filter all the points as that they do not differ too much from this
        prediction.'''
        while len(points):
            (a,b) = polyfit([p.x for p in points], [p.y for p in points], 1)
            f = lambda x: a * x + b
            largest = max([(abs(p.y - f(p.x)), p) for p in points])

            if largest[0] > threshold:
                points.remove(largest[1])
            else:
                break

        return points

    def __extract_table(self, points, dim):
        (a,b) = polyfit([p.x if p.x > 0 else 1 for p in points], [p.y for p in points], 1)
        return self.Observation(a,b, dim)

    class Observation:
        '''A recognized table, which can be used to query the angle of and distance
        towards the table. Returned by TableDetector.observe.'''
    
        def __init__(self, a, b, dim = (640,480)):
            '''y = ax + b, dim are the image dimensions'''
            self.a = a
            self.b = b
            self.__dim = dim

        def __repr__(self):
            return "<Table angle:%.3f; Table distance:%.3f>" % (math.degrees(self.angle), self.distance)

        @property
        def angle(self):
            '''angle of the table. Negative if table is rotated to the left.'''
            return math.atan2(self.a, 1)

        @property
        def distance(self):
            '''distance towards the table in pixels. dim is the width and height of
            the camera image'''
            f = lambda x: self.a * x + self.b
            if math.cos(abs(self.angle)) == 0:
                return max(self.__dim[1] - self.b, 0)
            return max(self.__dim[1] * math.cos(abs(self.angle)) - (f(self.__dim[0] / 2) * math.cos(abs(self.angle))), 0)

def test_tabledetector():
    detector = TableDetector()
    detector.verbose = True
    
    segmentizer = HistogramSegmentizer()
    segmentizer.verbose = True
    
    for img_path in sys.argv[1:]:
        print img_path
        img = cv.LoadImage(img_path, cv.CV_LOAD_IMAGE_GRAYSCALE)
        
        layers = segmentizer.segmentize(img)
        if not layers:
            print "Segmentizer failed, skipping"
        else:
            table = detector.detect(layers['table'])
        
        if cv.WaitKey(0) == -1:
            break

if __name__ == "__main__":
    test_tabledetector()

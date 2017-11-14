import cv
import sys
from util import copy_in_range, create_empty_image
from statistics import find_peaks, find_hill

def between(point, bounds):
    return point >= bounds[0] and point <= bounds[1]

class HistogramSegmentizer:
    '''Segmentizer that splits an image based on peaks in the histogram of the
    input image. It has an hard-coded range of where it expects the peak of
    the table. You can change it by altering the expectations dict property.'''
    def __init__(self, verbose = False):
        self.n_bins = 256
        self.hist_range = [0,255]
        self.ranges = {}
        self.expectations = {
            'table': (20, 60)
        }
        self.verbose = verbose
        
    def __calculate_histogram(self, src):
        hist = cv.CreateHist([self.n_bins], cv.CV_HIST_ARRAY, [self.hist_range], 1)
        cv.CalcHist([src], hist)
        return [cv.QueryHistValue_1D(hist, n) for n in range(self.n_bins)]

    def __render_histogram(self, values, dimensions = (512, 100)):
        hist_img = cv.CreateImage(dimensions, cv.IPL_DEPTH_8U, 3)
        cv.Rectangle(hist_img, (0,0), dimensions, (0, 0, 0), cv.CV_FILLED)

        max_value = max(values)
        # print values

        bar_width = dimensions[0] / len(values)
        height = dimensions[1]

        for idx, value in enumerate(values):
            intensity = int(round((value / max_value) * height))
            cv.Rectangle(hist_img,
                         (idx * bar_width, height - intensity),
                         ((idx + 1) * bar_width, height),
                         (255, 255, 255), 
                         cv.CV_FILLED)

        if 'table' in self.ranges:
            low, high = self.ranges['table']
            cv.Rectangle(hist_img,
                (low * bar_width, 0),
                (high * bar_width, dimensions[1] - 1),
                (0, 255, 255),
                1)

        return hist_img
    
    def segmentize(self, input_img):
        histogram = self.__calculate_histogram(input_img)
        (peaks, bottoms) = find_peaks(histogram, max(histogram) / 8)
        
        for peak in peaks:
            if between(peak, self.expectations['table']):
                self.ranges['table'] = find_hill(histogram, peak, 0.01)
                break #stop searching, we want the first peak since that is the highest large object.
        
        if not 'table' in self.ranges:
            if self.verbose:
                cv.ShowImage("histogram", self.__render_histogram(histogram))
            return None
        
        self.ranges['objects'] = (1, self.ranges['table'][0] - 1)
        
        print self.ranges
        
        if self.verbose:
            cv.ShowImage("histogram", self.__render_histogram(histogram))
        
        objects_img = create_empty_image((input_img.width, input_img.height))
        cv.SetImageROI(objects_img, cv.GetImageROI(input_img))
        cv.InRangeS(input_img,
            cv.Scalar(self.ranges['objects'][0]),
            cv.Scalar(self.ranges['objects'][1]),
            objects_img)
        
        table_img = create_empty_image((input_img.width, input_img.height))
        cv.SetImageROI(table_img, cv.GetImageROI(input_img))
        cv.InRangeS(input_img,
            cv.Scalar(self.ranges['table'][0]),
            cv.Scalar(self.ranges['table'][1]),
            table_img)
    
        return {'objects': objects_img, 'table': table_img}

class ConstantSegmentizer:
    '''Segmentizer that uses hard-coded ranges to split an image.
    You can change the ranges by altering the ranges dict property.'''
    
    def __init__(self):
        self.ranges = {
            'objects': (0, 30),
            'table': (30, 45),
            'head': (0, 10)
        }
    
    def segmentize(self, img):
        '''Extract the depth level that only contains the table, ignoring the
        objects and arms (i.e. as if they are holes in the table)'''
        
        layers = {}
        
        for name, (low, high) in self.ranges.iteritems():
            layer = cv.CreateImage((img.width, img.height), cv.IPL_DEPTH_8U, 1)
	    cv.SetImageROI(layer, cv.GetImageROI(img))
            cv.InRangeS(img, low, high, layer)
            layers[name] = layer
        
        return layers

if __name__ == "__main__":
    
    #segmentizer = ConstantSegmentizer()
    segmentizer = HistogramSegmentizer()
    segmentizer.verbose = True
    
    for path in sys.argv[1:]:
        input_img = cv.LoadImage(path, cv.CV_LOAD_IMAGE_GRAYSCALE)
	cv.ShowImage("Input", input_img)
	
        layers = segmentizer.segmentize(input_img)
        
        if not layers:
            print "Segmentizer failed"
        else:
            for name, img in layers.iteritems():
                cv.ShowImage(name, img)
            
        if cv.WaitKey(0) in (-1, 27):
            break

